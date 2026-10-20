# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/evaluate_coupled_bound_vortex_circulation.py
#
# Created:  Oct 2026, H. Hussien

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from RCAIDE.Framework.Core                           import Data
from RCAIDE.Library.Methods.Aerodynamics.Common.Lift  import compute_airfoil_aerodynamics
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory import biot_savart_velocity_induction, initialize_wake_geometry
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.evaluate_bound_vortex_circulation import compute_lifting_line_inflow_and_tip_loss

# ----------------------------------------------------------------------------------------------------------------------
#  evaluate_coupled_bound_vortex_circulation
# ----------------------------------------------------------------------------------------------------------------------
def evaluate_coupled_bound_vortex_circulation(rotors, wake_inputs_list, conditions):
    """
    Jacobi-coupled bound-circulation solve across multiple lifting-line rotors sharing the
    same vehicle body frame. Each rotor's induced velocity sums Biot-Savart contributions
    from every rotor's bound vortices and (prescribed) trailing wake, not just its own --
    the off-diagonal terms BEMT/single-rotor LL cannot capture.

    No coordinate transform is needed between rotors: nodes_14c/nodes_34c for every rotor are
    already expressed in the shared vehicle body frame by initialize_lifting_line(), so a
    target rotor's collocation points and a source rotor's bound/wake segments can be fed
    directly into biot_savart_velocity_induction() regardless of which rotor is which.

    PRESCRIBED WAKE ONLY: free_wake() is never called from here, regardless of any rotor's
    wake_inputs.free_wake flag -- wake geometry is always regenerated with
    initialize_wake_geometry() each outer CT iteration (the non-free-wake branch of
    evaluate_bound_vortex_circulation()). Coupled free-wake relaxation (which needs per-rotor
    hub-frame transforms and wake-age time-alignment across rotors) is deliberately out of
    scope for this step.

    For a single rotor (len(rotors) == 1) every cross term (i != j) is zero and this reduces
    algorithmically to evaluate_bound_vortex_circulation() -- same formulas, same loop order.
    That reduction is the regression check for this function.

    Parameters
    ----------
    rotors : list of RCAIDE Rotor objects (Lifting_Line_Theory fidelity)
        Each already run through initialize_lifting_line() + initialize_wake_geometry()
        (i.e. rotor.blades.bound / rotor.blades.wake are populated) -- see
        lifting_line_performance.prepare_wake_inputs().
    wake_inputs_list : list of Data, same order and length as `rotors`
        Each produced by lifting_line_performance.prepare_wake_inputs(rotor, conditions).
        vc_correction, CT_iter and include_wake must agree across all entries -- these rotors
        are assumed to belong to one vehicle-level interaction group set up consistently.
    conditions : Data
        Same conditions object passed to prepare_wake_inputs() for every rotor in `rotors`.

    Returns
    -------
    None
        Writes converged gamma/Cl/Cd/alpha/U/Ua/Ut/Wa/Wt/F/va/vt/lamdaw onto each rotor's
        rotor.blades.bound (and rotor.blades.wake.gamma), exactly like
        evaluate_bound_vortex_circulation() does for a single rotor.
    """
    n_rot = len(rotors)

    vc_correction = wake_inputs_list[0].vc_correction
    CT_iter       = wake_inputs_list[0].CT_iter
    include_wake  = wake_inputs_list[0].include_wake
    for wi in wake_inputs_list:
        assert wi.vc_correction == vc_correction, "All coupled rotors must share vc_correction"
        assert wi.CT_iter       == CT_iter,       "All coupled rotors must share CT_iter"
        assert wi.include_wake  == include_wake,  "All coupled rotors must share include_wake"

    # ------------------------------------------------------------------------------------------------------------------
    #  Per-rotor unpack + midpoint averaging (identical to evaluate_bound_vortex_circulation's
    #  own unpack block, just stacked into a list of per-rotor Data() instead of bare locals)
    # ------------------------------------------------------------------------------------------------------------------
    st = []
    for rotor, wake_inputs in zip(rotors, wake_inputs_list):
        s = Data()
        s.tag = rotor.tag

        U             = wake_inputs.velocity_total
        Ua            = wake_inputs.velocity_axial
        Ut            = wake_inputs.velocity_tangential
        T_body2thrust = wake_inputs.T_body2thrust
        ctrl_pts      = wake_inputs.ctrl_pts
        Nr            = wake_inputs.Nr
        beta          = wake_inputs.twist_distribution
        c             = wake_inputs.chord_distribution
        r             = wake_inputs.radius_distribution
        a_sound       = wake_inputs.speed_of_sound
        nu            = wake_inputs.kinematic_viscosity

        relax_Gammab = np.asarray(wake_inputs.relax_Gammab, dtype=float).reshape(-1)
        relax_CT     = np.asarray(wake_inputs.relax_CT, dtype=float).reshape(-1)
        if relax_Gammab.size == 1:
            relax_Gammab = np.full(ctrl_pts, relax_Gammab[0])
        if relax_CT.size == 1:
            relax_CT = np.full(ctrl_pts, relax_CT[0])
        s.relax_Gammab = relax_Gammab[:, np.newaxis, np.newaxis]
        s.relax_CT     = relax_CT[:, np.newaxis]

        s.mu     = wake_inputs.mu
        s.mu_max = wake_inputs.mu_max

        nodes_14c = wake_inputs.nodes_14c
        nodes_34c = wake_inputs.nodes_34c

        B        = rotor.number_of_blades
        R        = rotor.tip_radius
        tc       = rotor.thickness_to_chord
        a_loc    = rotor.airfoil_polar_stations
        airfoils = rotor.airfoils
        r_1d     = rotor.radius_distribution

        psi = rotor.blades.bound.psi

        psi     = 0.5*(psi[:-1, :]          + psi[1:, :]          )
        Ua      = 0.5*(Ua[:,  :-1, :]       + Ua[:,   1:, :]      )
        Ut      = 0.5*(Ut[:,  :-1, :]       + Ut[:,   1:, :]      )
        U       = 0.5*(U[:,   :-1, :]       + U[:,    1:, :]      )
        beta    = 0.5*(beta[:, :-1,:]       + beta[:, 1:, :]      )
        c       = 0.5*(c[:,   :-1, :]       + c[:,    1:, :]      )
        r       = 0.5*(r[:,   :-1, :]       + r[:,    1:, :]      )
        a_sound = 0.5*(a_sound[:,   :-1, :] + a_sound[:,    1:, :])
        nu      = 0.5*(nu[:,   :-1, :]      + nu[:,    1:, :]     )
        tc      = 0.5*(tc[:-1]           + tc[1:]           )

        a_loc = a_loc[:-1]

        s.rho   = conditions.freestream.density[:, 0, None]
        s.T     = conditions.freestream.temperature[:, 0, None, None]
        omega   = wake_inputs.omega

        if CT_iter:
            diff_r      = np.diff(r_1d)
            s.deltar_3d = diff_r[np.newaxis, :, np.newaxis] * np.ones((ctrl_pts, Nr-1, B))

        if include_wake:
            s.N_wake = rotor.blades.wake.N_wake

        CW   = omega[:, 0] > 0
        CW_3 = CW[:, np.newaxis, np.newaxis]

        s.A_disc    = (np.pi * R**2)
        omegar      = omega*R
        s.omegar_sq = omegar**2

        valid_cp = (s.mu <= s.mu_max)
        if np.any(~valid_cp):
            print(f"[{rotor.tag}] Skipping control point(s) {np.where(~valid_cp)[0].tolist()} -- "
                  f"edgewise advance ratio mu={s.mu[~valid_cp]} exceeds mu_max={s.mu_max}. "
                  f"Out of valid range for this method; not iterating on these.")
        s.valid_cp = valid_cp

        s.tang_hat_thrust_y = np.where(CW_3, -np.cos(psi), np.cos(psi))
        s.tang_hat_thrust_z = np.where(CW_3, -np.sin(psi), -np.sin(psi))
        s.CW_3              = CW_3

        Cl, _, _, _, _, _, _, _ = compute_airfoil_aerodynamics(
            beta, c, r, R, B, Ua, Ut, a_sound, nu, airfoils, a_loc, ctrl_pts, Nr-1, B, tc, use_2d_analysis=True)

        s.Gamma_b = 0.5*U*c*Cl

        s.P_colloc = nodes_34c.reshape(ctrl_pts, (Nr-1)*B, 3)
        s.A_bound  = nodes_14c[:, :-1, :, :].reshape(ctrl_pts, (Nr-1)*B, 3)
        s.B_bound  = nodes_14c[:, 1:,  :, :].reshape(ctrl_pts, (Nr-1)*B, 3)
        s.rCb      = rotor.blades.bound.rCb

        s.U             = U   # midpoint-averaged velocity_total -- NOT recomputed as sqrt(Ua**2+Ut**2)
                              # post-averaging, since sqrt doesn't commute with averaging
        s.Ua            = Ua
        s.Ut            = Ut
        s.T_body2thrust = T_body2thrust
        s.ctrl_pts      = ctrl_pts
        s.Nr            = Nr
        s.beta          = beta
        s.c             = c
        s.r             = r
        s.a_sound       = a_sound
        s.nu            = nu
        s.B             = B
        s.R             = R
        s.tc            = tc
        s.a_loc         = a_loc
        s.airfoils      = airfoils
        s.max_iter_Gammab = wake_inputs.max_iter_Gammab
        s.max_iter_CT     = wake_inputs.max_iter_CT
        s.tol             = wake_inputs.tol
        s.aerofoil_aero   = wake_inputs.aerofoil_aero
        s.thrust_coeff    = wake_inputs.thrust_coeff_initial_guess

        st.append(s)

    ctrl_pts = st[0].ctrl_pts   # shared across rotors (same conditions / mission segment)

    # ------------------------------------------------------------------------------------------------------------------
    #  Pre-compute bound influence matrices K_bound[i][j] -- target rotor i's collocation
    #  points, induced by source rotor j's bound vortex segments. i==j is today's single-rotor
    #  self-induction matrix; off-diagonal i!=j is the cross-rotor coupling term. Fixed for the
    #  whole solve (bound geometry doesn't change).
    # ------------------------------------------------------------------------------------------------------------------
    K_bound = [[None]*n_rot for _ in range(n_rot)]
    for i in range(n_rot):
        Mi = (st[i].Nr-1)*st[i].B
        for j in range(n_rot):
            Nj  = (st[j].Nr-1)*st[j].B
            Kij = np.zeros((ctrl_pts, Mi, Nj, 3))
            for cp in range(ctrl_pts):
                rCb_flat = st[j].rCb[cp, :, :].reshape(Nj)
                Kij[cp]  = biot_savart_velocity_induction(
                    st[i].P_colloc[cp], st[j].A_bound[cp], st[j].B_bound[cp], rCb_flat, vc_correction)
            K_bound[i][j] = Kij

    # ------------------------------------------------------------------------------------------------------------------
    #  Outer loop: CT (CT_iter=True) or single pass (CT_iter=False)
    # ------------------------------------------------------------------------------------------------------------------
    conv         = False
    n_outer      = max(s.max_iter_CT for s in st) if CT_iter else 1
    max_iter_Gab = max(s.max_iter_Gammab for s in st)

    K_wake = [[None]*n_rot for _ in range(n_rot)] if include_wake else None

    # -- Simple mode (CT_iter=False): wake geometry never changes, so K_wake is built once
    #    here instead of every outer iteration (mirrors evaluate_bound_vortex_circulation()'s
    #    own not-CT_iter pre-loop block) --
    if include_wake and not CT_iter:
        for i in range(n_rot):
            Mi = (st[i].Nr-1)*st[i].B
            for j in range(n_rot):
                Nj     = st[j].N_wake*st[j].B
                A_wake = rotors[j].blades.wake.nodes_body[:, :-1, :, :].reshape(ctrl_pts, Nj, 3)
                B_wake = rotors[j].blades.wake.nodes_body[:, 1:,  :, :].reshape(ctrl_pts, Nj, 3)
                rCvf   = rotors[j].blades.wake.rCvf
                Kij    = np.zeros((ctrl_pts, Mi, Nj, 3))
                for cp in range(ctrl_pts):
                    rCvf_flat = np.repeat(rCvf[cp], st[j].B)
                    Kij[cp]   = biot_savart_velocity_induction(
                        st[i].P_colloc[cp], A_wake[cp], B_wake[cp], rCvf_flat, vc_correction)
                K_wake[i][j] = Kij

    # Per-rotor "last computed this outer iteration" results -- declared here so they survive
    # past the inner loop (used by the CT check below, then by final storage after the loop).
    Gamma_b_new = [None]*n_rot
    Wa_l = [None]*n_rot; Wt_l = [None]*n_rot; W_l = [None]*n_rot
    Cl_l = [None]*n_rot; Cd_l = [None]*n_rot; Cdval_l = [None]*n_rot
    alpha_l = [None]*n_rot; alpha_disc_l = [None]*n_rot
    Ma_l = [None]*n_rot; Re_l = [None]*n_rot; Re_disc_l = [None]*n_rot
    F_l  = [None]*n_rot; lamdaw_l = [None]*n_rot
    ua_ind_l = [None]*n_rot; ut_ind_l = [None]*n_rot
    Gamma_wake_l = [None]*n_rot
    residual_Gamma_b = [0.0]*n_rot
    residual_CT      = [float('nan')]*n_rot
    diverged         = False

    for it in range(n_outer):

        # -- CT_iter mode: rebuild K_wake[i][j] each outer iteration (wake geometry changes) --
        if CT_iter and include_wake:
            for i in range(n_rot):
                Mi = (st[i].Nr-1)*st[i].B
                for j in range(n_rot):
                    Nj     = st[j].N_wake*st[j].B
                    A_wake = rotors[j].blades.wake.nodes_body[:, :-1, :, :].reshape(ctrl_pts, Nj, 3)
                    B_wake = rotors[j].blades.wake.nodes_body[:, 1:,  :, :].reshape(ctrl_pts, Nj, 3)
                    rCvf   = rotors[j].blades.wake.rCvf
                    Kij    = np.zeros((ctrl_pts, Mi, Nj, 3))
                    for cp in range(ctrl_pts):
                        rCvf_flat = np.repeat(rCvf[cp], st[j].B)
                        Kij[cp]   = biot_savart_velocity_induction(
                            st[i].P_colloc[cp], A_wake[cp], B_wake[cp], rCvf_flat, vc_correction)
                    K_wake[i][j] = Kij

        # -- Inner Gamma_b loop (Jacobi: every rotor uses every other rotor's PREVIOUS-iteration
        #    Gamma_b/Gamma_wake, then all rotors' Gamma_b are relaxed-updated together) --
        conv1_all = [False]*n_rot

        for it1 in range(max_iter_Gab):

            Gamma_bound_flat = [st[j].Gamma_b.reshape(ctrl_pts, (st[j].Nr-1)*st[j].B) for j in range(n_rot)]

            if include_wake:
                for j in range(n_rot):
                    gw = np.max(st[j].Gamma_b, axis=1, keepdims=True) * np.ones((ctrl_pts, st[j].N_wake, st[j].B))
                    Gamma_wake_l[j] = gw.reshape(ctrl_pts, st[j].N_wake*st[j].B)

            for i in range(n_rot):
                s  = st[i]
                Mi = (s.Nr-1)*s.B

                # Step 4a: bound vortex induction, summed over ALL source rotors j. The self
                # term (j==i) is left exactly as evaluate_bound_vortex_circulation() computes it
                # -- no weighting -- since that single-rotor formula's CW-branched Wa/Wt
                # combination below already encodes rotor i's own handedness correctly (not
                # revisited here). A CROSS term (j!=i) is weighted by sigma_j = -1 (CW) / +1
                # (CCW) -- source rotor j's OWN handedness, not target rotor i's -- matching
                # testing_LL/VF/model15's validated multi-rotor convention. Without this, a CW
                # neighbor and a CCW neighbor would contribute with no sign distinction at all,
                # which is physically undefined, not just "a different convention."
                v_induced_bound_body = np.zeros((ctrl_pts, Mi, 3))
                for j in range(n_rot):
                    contrib = np.zeros((ctrl_pts, Mi, 3))
                    for cp in range(ctrl_pts):
                        contrib[cp] = np.einsum('mnk,n->mk', K_bound[i][j][cp], Gamma_bound_flat[j][cp])
                    if j != i:
                        contrib = contrib * np.where(st[j].CW_3, -1.0, 1.0)
                    v_induced_bound_body += contrib
                v_induced_bound_body = v_induced_bound_body.reshape(ctrl_pts, s.Nr-1, s.B, 3)

                # Step 4b: wake induction, summed over ALL source rotors j -- same self/cross
                # sigma treatment as the bound term above.
                v_induced_wake_body = np.zeros((ctrl_pts, Mi, 3))
                if include_wake:
                    for j in range(n_rot):
                        contrib = np.zeros((ctrl_pts, Mi, 3))
                        for cp in range(ctrl_pts):
                            contrib[cp] = np.einsum('mnk,n->mk', K_wake[i][j][cp], Gamma_wake_l[j][cp])
                        if j != i:
                            contrib = contrib * np.where(st[j].CW_3, -1.0, 1.0)
                        v_induced_wake_body += contrib
                v_induced_wake_body = v_induced_wake_body.reshape(ctrl_pts, s.Nr-1, s.B, 3)

                v_induced_body = v_induced_bound_body + v_induced_wake_body

                # Transform body -> rotor i's own thrust frame
                v_induced_thrust = np.einsum('cij,crbj->crbi', s.T_body2thrust, v_induced_body)

                ua_ind = v_induced_thrust[:, :, :, 0]
                ut_ind = (v_induced_thrust[:, :, :, 1]*s.tang_hat_thrust_y +
                          v_induced_thrust[:, :, :, 2]*s.tang_hat_thrust_z)

                # Same CW-branched combination as evaluate_bound_vortex_circulation() -- sign
                # convention deliberately left exactly as-is; not revisited here (see that
                # function's docstring note). i==j self term reduces to the single-rotor case.
                Wa = np.where(s.CW_3, s.Ua - ua_ind, s.Ua + ua_ind)
                Wt = np.where(s.CW_3, s.Ut - ut_ind, s.Ut + ut_ind)
                W  = np.sqrt(Wa**2 + Wt**2)

                if s.aerofoil_aero == 1:
                    alpha      = s.beta - np.arctan2(Wa, Wt)
                    Ma         = W / s.a_sound
                    Re         = (W * s.c) / s.nu
                    alpha_disc = alpha
                    Re_disc    = Re
                    Cl       = (2.*np.pi/6.) * np.sin(6.*alpha)
                    KT_cond      = np.logical_and(Ma < 1., Cl > 0)
                    Cl[KT_cond]  = Cl[KT_cond] / ((1. - Ma[KT_cond]*Ma[KT_cond])**0.5 +
                                   (Ma[KT_cond]*Ma[KT_cond] / (1. + (1. - Ma[KT_cond]*Ma[KT_cond])**0.5)) * Cl[KT_cond]/2.)
                    Cdval    = 0.0087 - 0.0216*alpha + 0.4*alpha**2
                    Tw_Tinf  = 1. + 1.78*(Ma*Ma)
                    Tp_Tinf  = 1. + 0.035*(Ma*Ma) + 0.45*(Tw_Tinf-1.)
                    Tp       = Tp_Tinf * s.T
                    Rp_Rinf  = (Tp_Tinf**2.5)*(Tp+110.4)/(s.T+110.4)
                    Cd       = ((1/Tp_Tinf)*(1/Rp_Rinf)**0.2)*Cdval
                elif s.aerofoil_aero == 2:
                    Cl, Cdval, alpha, alpha_disc, Ma, _, Re, Re_disc = compute_airfoil_aerodynamics(
                        s.beta, s.c, s.r, s.R, s.B, Wa, Wt, s.a_sound, s.nu, s.airfoils, s.a_loc,
                        ctrl_pts, s.Nr-1, s.B, s.tc, use_2d_analysis=True)
                    Tw_Tinf  = 1. + 1.78*(Ma*Ma)
                    Tp_Tinf  = 1. + 0.035*(Ma*Ma) + 0.45*(Tw_Tinf-1.)
                    Tp       = Tp_Tinf * s.T
                    Rp_Rinf  = (Tp_Tinf**2.5)*(Tp+110.4)/(s.T+110.4)
                    Cd       = ((1/Tp_Tinf)*(1/Rp_Rinf)**0.2)*Cdval

                reverse_flow      = Wt < 0
                Cl[reverse_flow]  = 0.
                Cd[reverse_flow]  = 0.

                lamdaw, F, _ = compute_lifting_line_inflow_and_tip_loss(s.r, s.R, Wa, Wt, s.B)
                Cl = Cl * F

                Gb_new = 0.5*W*s.c*Cl

                bad_cp = s.valid_cp & ~np.all(np.isfinite(Gb_new) & (np.abs(Gb_new) < 1e4), axis=(1, 2))
                if np.any(bad_cp):
                    print(f"[{s.tag}] Gamma_b diverged at control point(s) {np.where(bad_cp)[0].tolist()} "
                          f"at inner iteration {it1+1}. Stopping.")
                    diverged = True

                res_i = np.max(np.abs(Gb_new - s.Gamma_b)[s.valid_cp]) if np.any(s.valid_cp) else 0.0
                residual_Gamma_b[i] = res_i

                s.Gamma_b[s.valid_cp] = s.Gamma_b[s.valid_cp] + s.relax_Gammab[s.valid_cp]*(Gb_new[s.valid_cp] - s.Gamma_b[s.valid_cp])

                conv1_all[i] = res_i < s.tol

                Gamma_b_new[i]  = Gb_new
                Wa_l[i] = Wa; Wt_l[i] = Wt; W_l[i] = W
                Cl_l[i] = Cl; Cd_l[i] = Cd; Cdval_l[i] = Cdval
                alpha_l[i] = alpha; alpha_disc_l[i] = alpha_disc
                Ma_l[i] = Ma; Re_l[i] = Re; Re_disc_l[i] = Re_disc
                F_l[i] = F; lamdaw_l[i] = lamdaw
                ua_ind_l[i] = ua_ind; ut_ind_l[i] = ut_ind

            if diverged:
                break

            if all(conv1_all):
                break

        if diverged:
            break

        for i in range(n_rot):
            if not conv1_all[i]:
                print(f"[{st[i].tag}] Gamma_b did not converge. Residual =", residual_Gamma_b[i])

        # -- CT check (CT_iter mode only), per rotor --
        if CT_iter:
            ct_converged_all = [True]*n_rot
            for i in range(n_rot):
                s = st[i]
                epsilon = Cd_l[i] / (Cl_l[i] + 1e-300)
                epsilon[np.abs(Cl_l[i]) <= 1e-6] = 10.0 * np.sign(Cl_l[i][np.abs(Cl_l[i]) <= 1e-6])

                blade_T_distribution = s.rho[:, :, None] * (Gamma_b_new[i]*(Wt_l[i] - epsilon*Wa_l[i])) * s.deltar_3d
                thrust               = np.sum(blade_T_distribution, axis=(1, 2))[:, None]
                Ct_rotor_new         = thrust / (s.rho * s.A_disc * s.omegar_sq)

                bad_cp_ct = s.valid_cp & ~np.isfinite(Ct_rotor_new[:, 0])
                if np.any(bad_cp_ct):
                    print(f"[{s.tag}] CT diverged to NaN at control point(s) {np.where(bad_cp_ct)[0].tolist()} "
                          f"at outer iteration {it+1}. Stopping.")
                    diverged = True
                    continue

                tol_CT    = s.get('tol_CT', s.tol)
                res_ct_i  = np.max(np.abs(Ct_rotor_new - s.thrust_coeff)[s.valid_cp]) if np.any(s.valid_cp) else 0.0
                residual_CT[i] = res_ct_i
                ct_converged_all[i] = res_ct_i < tol_CT

                old_CT           = np.broadcast_to(np.asarray(s.thrust_coeff, dtype=float), Ct_rotor_new.shape).copy()
                new_CT           = old_CT.copy()
                new_CT[s.valid_cp] = old_CT[s.valid_cp] + s.relax_CT[s.valid_cp]*(Ct_rotor_new[s.valid_cp] - old_CT[s.valid_cp])
                s.thrust_coeff   = new_CT
                wake_inputs_list[i].thrust_coeff_initial_guess = new_CT

            if diverged:
                break

            if all(ct_converged_all):
                conv = True
                break

            # Regenerate wake geometry per rotor -- prescribed wake only, free_wake() is never
            # called from this coupled path regardless of any rotor's wake_inputs.free_wake flag.
            if include_wake:
                for i in range(n_rot):
                    initialize_wake_geometry(rotors[i], wake_inputs_list[i], conditions)
        else:
            conv = True
            break

    if not conv and CT_iter:
        for i in range(n_rot):
            print(f"[{st[i].tag}] CT did not converge. Residual =", residual_CT[i])

    # ------------------------------------------------------------------------------------------------------------------
    #  Store converged results -- identical fields to evaluate_bound_vortex_circulation()
    # ------------------------------------------------------------------------------------------------------------------
    for i in range(n_rot):
        rotor = rotors[i]
        s     = st[i]
        rotor.blades.bound.gamma      = s.Gamma_b
        rotor.blades.bound.Cl         = Cl_l[i]
        rotor.blades.bound.Cdval      = Cdval_l[i]
        rotor.blades.bound.Cd         = Cd_l[i]
        rotor.blades.bound.alpha      = alpha_l[i]
        rotor.blades.bound.alpha_disc = alpha_disc_l[i]
        rotor.blades.bound.Ma         = Ma_l[i]
        rotor.blades.bound.Re         = Re_l[i]
        rotor.blades.bound.Re_disc    = Re_disc_l[i]
        rotor.blades.bound.U          = s.U
        rotor.blades.bound.Ua         = s.Ua
        rotor.blades.bound.Ut         = s.Ut
        rotor.blades.bound.W          = W_l[i]
        rotor.blades.bound.Wa         = Wa_l[i]
        rotor.blades.bound.Wt         = Wt_l[i]
        rotor.blades.bound.F          = F_l[i]
        rotor.blades.bound.va         = ua_ind_l[i]
        rotor.blades.bound.vt         = ut_ind_l[i]
        rotor.blades.bound.lamdaw     = lamdaw_l[i]

        if include_wake:
            rotor.blades.wake.gamma = Gamma_wake_l[i].reshape(ctrl_pts, s.N_wake, s.B)

    return
