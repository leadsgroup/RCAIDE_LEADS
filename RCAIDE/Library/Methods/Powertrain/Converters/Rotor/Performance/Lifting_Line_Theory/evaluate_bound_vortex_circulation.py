# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/evaluate_bound_vortex_circulation.py
#
# Created:  Jun 2026, H. Hussien
# Modified: Oct 2026, H. Hussien -- multi-rotor: coupled Gamma_b solve with rotor-rotor
#           interaction (prescribed wake only -- free_wake's multi-rotor extension is not
#           wired in yet, so CT_iter mode always regenerates the prescribed wake geometry,
#           regardless of wake_inputs.free_wake).

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from RCAIDE.Framework.Core                           import Data
from RCAIDE.Library.Methods.Aerodynamics.Common.Lift  import compute_airfoil_aerodynamics
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory import biot_savart_velocity_induction, initialize_wake_geometry, free_wake

# ----------------------------------------------------------------------------------------------------------------------
#  compute_lifting_line_inflow_and_tip_loss
# ----------------------------------------------------------------------------------------------------------------------
def compute_lifting_line_inflow_and_tip_loss(r, R, Wa, Wt, B, et1=1, et2=1, et3=1):
    """
    Lifting-line-specific inflow ratio and Prandtl tip loss factor.

    Kept separate from RCAIDE.Library.Methods.Aerodynamics.Common.Lift.compute_inflow_and_tip_loss
    (used by BEMT) because lamdaw here is not r/R-scaled -- BEMT's tip-loss iteration was
    validated against the r/R-scaled, floored version, and sharing this function caused BEMT
    to stop converging on cases that used to work.

    Inputs/Outputs match compute_inflow_and_tip_loss (see that function's docstring).
    """
    lamdaw = Wa / Wt

    tipfactor = B/2.0 * ((R/r)**et1 - 1)**et2 / np.abs(lamdaw)**et3

    piece = np.exp(-tipfactor)

    Ftip  = (2./np.pi) * np.arccos(piece)

    return lamdaw, Ftip, piece

# ----------------------------------------------------------------------------------------------------------------------
#  evaluate_bound_vortex_circulation
# ----------------------------------------------------------------------------------------------------------------------
def evaluate_bound_vortex_circulation(rotor, wake_inputs, conditions):
    """
    Iterates on bound circulation Gamma_b of a rotor, or of every rotor in a list (multi-rotor
    coupling), using Biot-Savart induction from all bound vortex segments and all wake
    filaments of ALL rotors (self-induction + rotor-rotor interaction), evaluated at each
    rotor's 3/4c collocation points.

    Two iteration modes controlled by wake_inputs.CT_iter:
        False : inner Gamma_b loop only (simple, fast)
        True  : outer CT loop + inner Gamma_b loop (self-consistent wake geometry). The wake
                geometry regenerated each outer iteration is always the PRESCRIBED model
                (initialize_wake_geometry) -- free_wake's multi-rotor extension is not wired
                in yet, so wake_inputs.free_wake is ignored here.

    Coupling
    --------
    The Gamma_b of all rotors is solved together (Jacobi iteration): each inner iteration first
    computes the induced velocity at every rotor from the current Gamma_b of every rotor, then
    updates each rotor's Gamma_b with its own relaxation factor. Converged when every rotor's
    residual is below its own tol.

    Sign convention: a CCW rotor is the y-mirror of a CW rotor, so with Gamma_b > 0 on both the
    Biot-Savart velocity K*Gamma of the two has opposite handedness. The sign is carried by the
    SOURCE rotor: sigma = -1 (CW), +1 (CCW), and the effective velocity from source j is
    sigma_j * K_ij * Gamma_j. At the induced rotor, Wa = Ua + v_x and Wt = Ut - v.t_hat (t_hat =
    direction of blade motion). This convention (not a per-receiver np.where(CW,...) flip) is
    the one that was settled by solving a real two-rotor CW+CCW case together; kept as-is here.

    Parameters
    ----------
    rotor : Data, list, or dict of Data
        Rotor object(s). Reads rotor.blades.bound (see initialize_lifting_line) and
        writes the converged gamma, cl, cd, alpha, U, Ua, Ut back onto each rotor.
    wake_inputs : Data, or list of Data (one per rotor, same order as rotor)
        velocity_total      : (ctrl_pts, Nr, B)    total local velocity [m/s]
        velocity_axial      : (ctrl_pts, Nr, B)    axial local velocity [m/s]
        velocity_tangential : (ctrl_pts, Nr, B)    tangential local velocity [m/s]
        ctrl_pts            : int
        Nr                  : int
        twist_distribution  : (ctrl_pts, Nr, B)  total blade pitch [rad]
        chord_distribution  : (ctrl_pts, Nr, B)  chord [m]
        radius_distribution : (ctrl_pts, Nr, B)  radial stations [m]
        speed_of_sound      : (ctrl_pts, Nr, B)
        kinematic_viscosity : (ctrl_pts, Nr, B)
        nodes_14c           : (ctrl_pts, Nr, B, 3) 1/4c nodes, body frame inducing nodes
        nodes_34c           : (ctrl_pts, Nr, B, 3) 3/4c nodes, body frame induced  nodes
        max_iter_Gammab, max_iter_CT, tol, tol_CT, relax_Gammab, relax_CT : iteration controls
        mu, mu_max          : per-control-point edgewise advance ratio and its valid-range cap
        vc_correction       : int    -- vortex core correction model (of the SOURCE filament)
        CT_iter             : bool
        include_wake        : bool
        aerofoil_aero       : int
        thrust_coeff_initial_guess : float or (ctrl_pts,1) -- CT (used/updated in CT_iter mode)
    conditions : Data

    Returns
    -------
        None

    **Major Assumptions**
        * The trailing wake is modeled as a single dominant vortex per blade, carrying the
          peak bound circulation across the span (``Gamma_wake = max(Gamma_b, axis=radial)``),
          not a full Helmholtz-consistent shed sheet that varies filament-by-filament with
          ``dGamma_b/dr``. This is a deliberate simplification, not a placeholder.

    **Theory**
        Local circulation from blade element theory:

        .. math::

            \\Gamma_b = 0.5 \\, W \\, c \\, C_l

        where W is the local relative velocity, c is the local chord, and
        :math:`C_l` is the local lift coefficient from the airfoil polar at
        the local effective angle of attack.

    References
    ----------
    [1] J. Katz and A. Plotkin, Low-Speed Aerodynamics, 2nd ed.,
        Cambridge University Press, 2001, Section 2.12.
    [2] W. Johnson, Rotorcraft Aeromechanics, Cambridge University Press, 2013, Section 9.9.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.lifting_line_performance
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.Biot_Savart_velocity_induction
    """
    # a single rotor is itself a Data/dict, so identify it by its fields, not by isinstance(dict)
    rotor_list = [rotor] if hasattr(rotor, 'number_of_blades') else (list(rotor.values()) if isinstance(rotor, dict) else list(rotor))
    wake_inputs_list = [wake_inputs] if len(rotor_list) == 1 and not isinstance(wake_inputs, (list, tuple)) else list(wake_inputs)
    n_rot = len(rotor_list)

    T = conditions.freestream.temperature[:, 0, None, None]

    # ------------------------------------------------------------------------------------------------------------------
    #  Unpack -- per rotor
    # ------------------------------------------------------------------------------------------------------------------
    S = [_setup_rotor(r, wi, conditions) for r, wi in zip(rotor_list, wake_inputs_list)]

    ctrl_pts = S[0].ctrl_pts
    CT_iter  = any(s.CT_iter for s in S)

    # ------------------------------------------------------------------------------------------------------------------
    #  Pre-compute bound influence matrices K_bound[i][j] (geometry fixed throughout):
    #  bound vortices of rotor j acting on the collocation points of rotor i
    # ------------------------------------------------------------------------------------------------------------------
    K_bound = [[_influence(si.P_colloc, sj.A_bound, sj.B_bound, sj.rCb_flat, sj.vc_correction) for sj in S] for si in S]

    # ------------------------------------------------------------------------------------------------------------------
    #  Outer loop: CT (CT_iter=True) or single pass (CT_iter=False)
    # ------------------------------------------------------------------------------------------------------------------
    conv      = False
    n_outer   = max(s.max_iter_CT for s in S) if CT_iter else 1
    max_inner = max(s.max_iter_Gammab for s in S)

    for it in range(n_outer):

        # -- K_wake[i][j]: wake of rotor j acting on rotor i. Rebuilt each outer iteration (wake
        #    geometry changes with CT); in simple mode there is a single outer pass so it's built once --
        K_wake = [[None]*n_rot for _ in range(n_rot)]
        for j, sj in enumerate(S):
            if not sj.include_wake:
                continue
            A_wake, B_wake, rCvf_flat = _wake_filaments(sj)
            for i, si in enumerate(S):
                K_wake[i][j] = _influence(si.P_colloc, A_wake, B_wake, rCvf_flat, sj.vc_correction)

        # -- Inner Gamma_b loop, all rotors together --
        conv1    = False
        diverged = False
        for it1 in range(max_inner):

            # Source strengths from the current Gamma_b of every rotor
            Gamma_bound_flat = [s.Gamma_b.reshape(s.ctrl_pts, (s.Nr-1)*s.B) for s in S]
            Gamma_wake_flat  = [None]*n_rot
            for j, sj in enumerate(S):
                if sj.include_wake:
                    # Use max Gamma_b instead of shed station value (same simplification as the
                    # single-rotor path)
                    Gamma_wake         = np.max(sj.Gamma_b, axis=1, keepdims=True) * np.ones((sj.ctrl_pts, sj.N_wake, sj.B))
                    Gamma_wake_flat[j] = Gamma_wake.reshape(sj.ctrl_pts, sj.N_wake*sj.B)

            # Bound + wake induction from every rotor, at every rotor
            v_by = [[None]*n_rot for _ in range(n_rot)]
            for i, si in enumerate(S):
                for j, sj in enumerate(S):
                    v_ij = np.einsum('cmnk,cn->cmk', K_bound[i][j], Gamma_bound_flat[j])
                    if sj.include_wake:
                        v_ij = v_ij + np.einsum('cmnk,cn->cmk', K_wake[i][j], Gamma_wake_flat[j])
                    v_by[i][j] = sj.sigma * v_ij.reshape(si.ctrl_pts, si.Nr-1, si.B, 3)

            bad = False
            for i, si in enumerate(S):
                v_eff_body = sum(v_by[i])
                _blade_aero(si, v_eff_body, T)

                bad_cp = si.valid_cp & ~np.all(np.isfinite(si.Gamma_b_new) & (np.abs(si.Gamma_b_new) < 1e4), axis=(1, 2))
                if np.any(bad_cp):
                    print(f"{si.tag}: Gamma_b diverged at control point(s) {np.where(bad_cp)[0].tolist()} "
                          f"at inner iteration {it1+1}. Stopping.")
                    bad = True
                    break

                residual_Gamma_b    = np.max(np.abs(si.Gamma_b_new - si.Gamma_b)[si.valid_cp]) if np.any(si.valid_cp) else 0.0
                si.Gamma_b[si.valid_cp]  = si.Gamma_b[si.valid_cp] + si.relax_Gammab[si.valid_cp]*(si.Gamma_b_new[si.valid_cp] - si.Gamma_b[si.valid_cp])
                si.residual_Gamma_b = residual_Gamma_b
                si.v_by             = v_by[i]

            if bad:
                diverged = True
                break

            if all(s.residual_Gamma_b < s.tol for s in S):
                conv1 = True
                break

        if diverged:
            break   # stop the outer CT loop too -- no point continuing once it has blown up

        if conv1:
            for s in S:
                print(f"{s.tag}: Gamma_b converged in {it1+1} iteration(s), residual = {s.residual_Gamma_b:.3e}")
        else:
            for s in S:
                print(f"{s.tag}: Gamma_b did NOT converge after {max_inner} iteration(s). Residual =", s.residual_Gamma_b)

        # -- CT check (CT_iter mode only) --
        if CT_iter:
            all_conv = True
            for s in S:
                if not s.CT_iter:
                    continue
                epsilon                     = s.Cd / (s.Cl + 1e-300)
                epsilon[np.abs(s.Cl) <= 1e-6] = 10.0 * np.sign(s.Cl[np.abs(s.Cl) <= 1e-6])

                blade_T_distribution = s.rho[:, :, None] * (s.Gamma_b_new*(s.Wt - epsilon*s.Wa)) * s.deltar_3d
                thrust                = np.sum(blade_T_distribution, axis=(1, 2))[:, None]   # (ctrl_pts, 1)
                s.Ct_rotor_new        = thrust / (s.rho_0 * s.A * s.omegar_sq)

                bad_cp_ct = s.valid_cp & ~np.isfinite(s.Ct_rotor_new[:, 0])
                if np.any(bad_cp_ct):
                    print(f"{s.tag}: CT diverged to NaN at control point(s) {np.where(bad_cp_ct)[0].tolist()} "
                          f"at outer iteration {it+1}. Stopping.")
                    all_conv = False
                    continue

                s.residual_CT = np.max(np.abs(s.Ct_rotor_new - s.wake_inputs.thrust_coeff_initial_guess)[s.valid_cp]) if np.any(s.valid_cp) else 0.0
                all_conv &= s.residual_CT < s.tol_CT

            if all_conv:
                conv = True
                for s in S:
                    if s.CT_iter:
                        print(f"{s.tag}: CT converged in {it+1} outer iteration(s), residual = {s.residual_CT:.3e}")
                break

            for s in S:
                if s.CT_iter and np.any(s.valid_cp):
                    old_CT           = np.broadcast_to(np.asarray(s.wake_inputs.thrust_coeff_initial_guess, dtype=float), s.Ct_rotor_new.shape).copy()
                    new_CT           = old_CT.copy()
                    new_CT[s.valid_cp] = old_CT[s.valid_cp] + s.relax_CT[s.valid_cp]*(s.Ct_rotor_new[s.valid_cp] - old_CT[s.valid_cp])
                    s.wake_inputs.thrust_coeff_initial_guess = new_CT

            # Regenerate prescribed wake geometry for every rotor that includes a wake -- the
            # fallback shape if nothing wants free_wake, and the starting shape free_wake
            # further relaxes otherwise. bound.gamma is set for every source rotor (not just
            # the ones being relaxed), since free_wake uses every rotor's current bound
            # circulation as a fixed source for the others -- see that function's docstring.
            wake_rotors      = [s.rotor       for s in S if s.include_wake]
            wake_wake_inputs = [s.wake_inputs for s in S if s.include_wake]
            for s in S:
                if s.include_wake:
                    s.rotor.blades.bound.gamma = s.Gamma_b
            if wake_rotors:
                initialize_wake_geometry(wake_rotors, wake_wake_inputs, conditions)
                # regeneration resets wake.gamma to zero -- restore it, or the next outer
                # iteration's/free_wake's K_wake rebuild would see these wakes with no strength
                for s in S:
                    if s.include_wake:
                        Gamma_wake = np.max(s.Gamma_b, axis=1, keepdims=True)   # (ctrl_pts, 1, B)
                        s.rotor.blades.wake.gamma = Gamma_wake * np.ones((s.ctrl_pts, s.N_wake, s.B))

                if any(wi.free_wake for wi in wake_wake_inputs):
                    free_wake(wake_rotors, wake_wake_inputs, conditions)
        else:
            conv = True   # simple mode always exits after one outer pass

    if not conv and CT_iter:
        for s in S:
            if s.CT_iter:
                print(f"{s.tag}: CT did NOT converge after {n_outer} outer iteration(s). Residual =", s.residual_CT)

    # ------------------------------------------------------------------------------------------------------------------
    #  Store converged results -- per rotor
    # ------------------------------------------------------------------------------------------------------------------
    for s in S:
        bound = s.rotor.blades.bound
        bound.gamma      = s.Gamma_b
        bound.Cl         = s.Cl
        bound.Cdval      = s.Cdval
        bound.Cd         = s.Cd
        bound.alpha      = s.alpha
        bound.alpha_disc = s.alpha_disc
        bound.Ma         = s.Ma
        bound.Re         = s.Re
        bound.Re_disc    = s.Re_disc
        bound.U          = s.U
        bound.Ua         = s.Ua
        bound.Ut         = s.Ut
        bound.W          = s.W
        bound.Wa         = s.Wa
        bound.Wt         = s.Wt
        bound.F          = s.F
        bound.va         = s.ua_ind
        bound.vt         = s.ut_ind
        bound.lamdaw     = s.lamdaw

        if s.include_wake:
            s.rotor.blades.wake.gamma = np.max(s.Gamma_b, axis=1, keepdims=True) * np.ones((s.ctrl_pts, s.N_wake, s.B))

    return

# ----------------------------------------------------------------------------------------------------------------------
#  _setup_rotor -- per-rotor unpack, midpoint averaging, initial guess, bound geometry
# ----------------------------------------------------------------------------------------------------------------------
def _setup_rotor(rotor, wake_inputs, conditions):
    """Per-rotor unpack, mirroring the single-rotor function's top-of-function block exactly,
    plus the rotation-sense/basis-vector/sigma setup needed for multi-rotor coupling."""
    s             = Data()
    s.rotor       = rotor
    s.tag         = rotor.tag
    s.wake_inputs = wake_inputs

    U               = wake_inputs.velocity_total
    Ua              = wake_inputs.velocity_axial
    Ut              = wake_inputs.velocity_tangential
    T_body2thrust   = wake_inputs.T_body2thrust
    ctrl_pts        = wake_inputs.ctrl_pts
    Nr              = wake_inputs.Nr
    beta            = wake_inputs.twist_distribution
    c               = wake_inputs.chord_distribution
    r               = wake_inputs.radius_distribution
    a_sound         = wake_inputs.speed_of_sound
    nu              = wake_inputs.kinematic_viscosity
    max_iter_Gammab = wake_inputs.max_iter_Gammab
    max_iter_CT     = wake_inputs.max_iter_CT
    tol             = wake_inputs.tol

    relax_Gammab = np.asarray(wake_inputs.relax_Gammab, dtype=float).reshape(-1)
    relax_CT     = np.asarray(wake_inputs.relax_CT, dtype=float).reshape(-1)
    if relax_Gammab.size == 1:
        relax_Gammab = np.full(ctrl_pts, relax_Gammab[0])
    if relax_CT.size == 1:
        relax_CT = np.full(ctrl_pts, relax_CT[0])
    relax_Gammab = relax_Gammab[:, np.newaxis, np.newaxis]
    relax_CT     = relax_CT[:, np.newaxis]

    mu     = wake_inputs.mu
    mu_max = wake_inputs.mu_max

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
    tc      = 0.5*(tc[:-1]              + tc[1:]              )

    a_loc = a_loc[:-1]

    rho   = conditions.freestream.density[:, 0, None]
    rho_0 = rho
    omega = wake_inputs.omega

    if wake_inputs.CT_iter:
        diff_r    = np.diff(r_1d)
        deltar_3d = diff_r[np.newaxis, :, np.newaxis] * np.ones((ctrl_pts, Nr-1, B))
        s.deltar_3d = deltar_3d

    s.include_wake = bool(wake_inputs.include_wake)
    if s.include_wake:
        s.N_wake = rotor.blades.wake.N_wake

    # rotation sense is a fixed rotor property, not a sign on omega (same convention already
    # established in initialize_lifting_line.py/initialize_wake_geometry.py/free_wake.py)
    CW = np.full(omega[:, 0].shape, rotor.clockwise_rotation, dtype=bool)
    CW_3 = CW[:, np.newaxis, np.newaxis]
    s.CW    = CW
    # handedness of this rotor's vortex system as a SOURCE -- settled by solving a real
    # two-rotor CW+CCW case together (see module docstring); kept as-is
    s.sigma = -1.0 if bool(rotor.clockwise_rotation) else 1.0

    A         = (np.pi * R**2)
    omegar    = omega*R
    omegar_sq = omegar**2

    valid_cp = (mu <= mu_max)
    if np.any(~valid_cp):
        print(f"{rotor.tag}: Skipping control point(s) {np.where(~valid_cp)[0].tolist()} -- "
              f"edgewise advance ratio mu={mu[~valid_cp]} exceeds mu_max={mu_max}. "
              f"Out of valid range for this method; not iterating on these.")

    radial_hat_thrust_y = np.where(CW_3, -np.sin(psi), np.sin(psi))
    radial_hat_thrust_z = np.where(CW_3,  np.cos(psi), np.cos(psi))
    tang_hat_thrust_y   = np.where(CW_3, -np.cos(psi), np.cos(psi))
    tang_hat_thrust_z   = np.where(CW_3, -np.sin(psi), -np.sin(psi))

    Cl, _, _, _, _, _, _, _ = compute_airfoil_aerodynamics(
        beta, c, r, R, B, Ua, Ut, a_sound, nu, airfoils, a_loc, ctrl_pts, Nr-1, B, tc, use_2d_analysis=True)

    Gamma_b = 0.5*U*c*Cl

    P_colloc = nodes_34c.reshape(ctrl_pts, (Nr-1)*B, 3)
    A_bound  = nodes_14c[:, :-1, :, :].reshape(ctrl_pts, (Nr-1)*B, 3)
    B_bound  = nodes_14c[:, 1:,  :, :].reshape(ctrl_pts, (Nr-1)*B, 3)
    rCb      = rotor.blades.bound.rCb
    rCb_flat = rCb.reshape(ctrl_pts, (Nr-1)*B)

    s.ctrl_pts, s.Nr, s.B, s.R          = ctrl_pts, Nr, B, R
    s.U, s.Ua, s.Ut, s.beta, s.c, s.r   = U, Ua, Ut, beta, c, r
    s.a_sound, s.nu, s.tc               = a_sound, nu, tc
    s.a_loc, s.airfoils, s.r_1d         = a_loc, airfoils, r_1d
    s.psi                               = psi
    s.T_body2thrust                     = T_body2thrust
    s.max_iter_Gammab, s.max_iter_CT    = max_iter_Gammab, max_iter_CT
    s.tol                               = tol
    s.tol_CT                            = wake_inputs.get('tol_CT', tol)
    s.relax_Gammab, s.relax_CT          = relax_Gammab, relax_CT
    s.mu, s.mu_max                      = mu, mu_max
    s.rho, s.rho_0                      = rho, rho_0
    s.omega, s.A                        = omega, A
    s.omegar, s.omegar_sq               = omegar, omegar_sq
    s.valid_cp                          = valid_cp
    s.radial_hat_thrust_y, s.radial_hat_thrust_z = radial_hat_thrust_y, radial_hat_thrust_z
    s.tang_hat_thrust_y, s.tang_hat_thrust_z     = tang_hat_thrust_y, tang_hat_thrust_z
    s.Gamma_b                           = Gamma_b
    s.P_colloc                          = P_colloc
    s.A_bound, s.B_bound, s.rCb_flat     = A_bound, B_bound, rCb_flat
    s.vc_correction                     = wake_inputs.vc_correction
    s.aerofoil_aero                     = wake_inputs.aerofoil_aero
    s.CT_iter                           = bool(wake_inputs.CT_iter)
    s.residual_Gamma_b                  = float('nan')
    s.residual_CT                       = float('nan')
    return s

# ----------------------------------------------------------------------------------------------------------------------
#  _wake_filaments -- current wake filaments of one rotor (body frame)
# ----------------------------------------------------------------------------------------------------------------------
def _wake_filaments(s):
    nodes_body = s.rotor.blades.wake.nodes_body
    N_wake     = s.N_wake
    A_wake     = nodes_body[:, :-1, :, :].reshape(s.ctrl_pts, N_wake*s.B, 3)
    B_wake     = nodes_body[:, 1:,  :, :].reshape(s.ctrl_pts, N_wake*s.B, 3)
    rCvf       = s.rotor.blades.wake.rCvf
    rCvf_flat  = np.repeat(rCvf, s.B, axis=1)
    return A_wake, B_wake, rCvf_flat

# ----------------------------------------------------------------------------------------------------------------------
#  _influence -- Biot-Savart influence of filaments (A->B, core rc) at points P, per ctrl_pt
# ----------------------------------------------------------------------------------------------------------------------
def _influence(P, A, B, rc, vc_correction):
    ctrl_pts = P.shape[0]
    K = np.zeros((ctrl_pts, P.shape[1], A.shape[1], 3))
    for cp in range(ctrl_pts):
        K[cp] = biot_savart_velocity_induction(P[cp], A[cp], B[cp], rc[cp], vc_correction)
    return K

# ----------------------------------------------------------------------------------------------------------------------
#  _blade_aero -- sectional aerodynamics of one rotor for a given effective induced velocity (body frame)
# ----------------------------------------------------------------------------------------------------------------------
def _blade_aero(s, v_eff_body, T):
    ctrl_pts, Nr, B, R = s.ctrl_pts, s.Nr, s.B, s.R
    beta, c, r, a_sound, nu = s.beta, s.c, s.r, s.a_sound, s.nu

    v_eff_thrust = np.einsum('cij,crbj->crbi', s.T_body2thrust, v_eff_body)

    ua_ind = v_eff_thrust[:,:,:,0]
    ut_ind = (v_eff_thrust[:,:,:,1]*s.tang_hat_thrust_y +
              v_eff_thrust[:,:,:,2]*s.tang_hat_thrust_z)

    # Fixed combination (no per-receiver CW/CCW flip here) -- handedness is carried entirely by
    # the per-SOURCE sigma already baked into v_eff_body upstream. See module docstring.
    Wa = s.Ua + ua_ind
    Wt = s.Ut - ut_ind
    W  = np.sqrt(Wa**2 + Wt**2)

    s.ua_ind = ua_ind
    s.ut_ind = ut_ind

    if s.aerofoil_aero == 1:
        alpha      = beta - np.arctan2(Wa, Wt)
        Ma         = W / a_sound
        Re         = (W * c) / nu
        alpha_disc = alpha
        Re_disc    = Re
        Cl       = (2.*np.pi/6.) * np.sin(6.*alpha)
        KT_cond      = np.logical_and(Ma < 1., Cl > 0)
        Cl[KT_cond]  = Cl[KT_cond] / ((1. - Ma[KT_cond]*Ma[KT_cond])**0.5 +
                       (Ma[KT_cond]*Ma[KT_cond] / (1. + (1. - Ma[KT_cond]*Ma[KT_cond])**0.5)) * Cl[KT_cond]/2.)
        Cdval    = 0.0087 - 0.0216*alpha + 0.4*alpha**2
        Tw_Tinf  = 1. + 1.78*(Ma*Ma)
        Tp_Tinf  = 1. + 0.035*(Ma*Ma) + 0.45*(Tw_Tinf-1.)
        Tp       = Tp_Tinf * T
        Rp_Rinf  = (Tp_Tinf**2.5)*(Tp+110.4)/(T+110.4)
        Cd       = ((1/Tp_Tinf)*(1/Rp_Rinf)**0.2)*Cdval
    elif s.aerofoil_aero == 2:
        Cl, Cdval, alpha, alpha_disc, Ma, _, Re, Re_disc = compute_airfoil_aerodynamics(
            beta, c, r, R, B, Wa, Wt, a_sound, nu, s.airfoils, s.a_loc, ctrl_pts, Nr-1, B, s.tc, use_2d_analysis=True)
        Tw_Tinf  = 1. + 1.78*(Ma*Ma)
        Tp_Tinf  = 1. + 0.035*(Ma*Ma) + 0.45*(Tw_Tinf-1.)
        Tp       = Tp_Tinf * T
        Rp_Rinf  = (Tp_Tinf**2.5)*(Tp+110.4)/(T+110.4)
        Cd       = ((1/Tp_Tinf)*(1/Rp_Rinf)**0.2)*Cdval

    # Reverse-flow region: Wt<0 near the root on the retreating side at high mu -- not a
    # numerical edge case, a genuinely different aerodynamic regime this method doesn't model.
    # Zeroing lift/drag here is a standard engineering approximation (low dynamic pressure,
    # small contribution to net thrust/torque), not a numerical hack.
    reverse_flow        = Wt < 0
    Cl[reverse_flow]    = 0.
    Cd[reverse_flow]    = 0.

    lamdaw, F, _ = compute_lifting_line_inflow_and_tip_loss(r, R, Wa, Wt, B)

    Cl = Cl * F

    s.Gamma_b_new = 0.5*W*c*Cl
    s.Cl, s.Cdval, s.Cd, s.alpha, s.alpha_disc = Cl, Cdval, Cd, alpha, alpha_disc
    s.Ma, s.Re, s.Re_disc, s.W, s.Wa, s.Wt     = Ma, Re, Re_disc, W, Wa, Wt
    s.F, s.lamdaw                              = F, lamdaw
