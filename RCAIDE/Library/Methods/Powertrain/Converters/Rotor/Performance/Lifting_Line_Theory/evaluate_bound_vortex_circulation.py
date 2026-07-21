# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/evaluate_bound_vortex_circulation.py
#
# Created:  Jun 2026, H. Hussien

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from RCAIDE.Framework.Core                           import Data, orientation_product, orientation_transpose
from RCAIDE.Library.Methods.Aerodynamics.Common.Lift  import compute_airfoil_aerodynamics
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory import biot_savart_velocity_induction, initialize_wake_geometry

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
    Iterates on bound circulation Gamma_b using Biot-Savart induction from all
    bound vortex segments on all blades, evaluated at 3/4c collocation points.
    No trailing or shed wake is modeled in this version.

    Parameters
    ----------
    rotor : Data
        Rotor object. Reads rotor.blades.bound (see initialize_lifting_line) and
        writes the converged gamma, cl, cd, alpha, U, Ua, Ut back onto it.
    wake_inputs : Data
        velocity_total      : (ctrl_pts, B, Nr)    total local velocity [m/s]
        velocity_axial      : (ctrl_pts, B, Nr)    axial local velocity [m/s]
        velocity_tangential : (ctrl_pts, B, Nr)    tangential local velocity [m/s]
        ctrl_pts            : int
        Nr                  : int
        twist_distribution  : (ctrl_pts, B, Nr)  total blade pitch [rad]
        chord_distribution  : (ctrl_pts, B, Nr)  chord [m]
        radius_distribution : (ctrl_pts, B, Nr)  radial stations [m]
        speed_of_sounds     : (ctrl_pts, B, Nr)
        dynamic_viscosities : (ctrl_pts, B, Nr)
        nodes_14c           : (ctrl_pts, B, Nr, 3) 1/4c nodes, body frame inducing nodes
        nodes_34c           : (ctrl_pts, B, Nr, 3) 3/4c nodes, body frame induced  nodes
    conditions : Data
        Flight conditions with:
            - freestream : Data
                Freestream properties
                    - density : array_like
                        Air density [kg/m³]
                    - dynamic_viscosity : array_like
                        Dynamic viscosity [kg/(m·s)]
                    - speed_of_sound : array_like
                        Speed of sound [m/s]
                    - temperature : array_like
                        Temperature [K]
            - frames : Data
                Reference frames
                    - body : Data
                        Body frame
                        - transform_to_inertial : array_like
                            Rotation matrix from body to inertial frame
                    - inertial : Data
                        Inertial frame
                        - velocity_vector : array_like
                            Velocity vector in inertial frame [m/s]
            - energy : Data
                Energy conditions
                    - converters : dict
                        Converter energy conditions indexed by tag
                        - commanded_thrust_vector_angle : array_like
                            Commanded thrust vector angle [rad]
                        - blade_pitch_command : array_like
                            Blade pitch command [rad]
                        - omega : array_like
                            Angular velocity [rad/s]
                        - throttle : array_like
                            Throttle setting [0-1]
                        - design_flag : bool
                            Flag indicating design condition

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

        Iterations steps:
            1. use alpha = beta
            2. use the function compute_airfoiul_aerodynamics to get cl for all blades for all segments
            3.compute Gamma_b as initial guesses
            4. update the velocity induction to have Biot Savart effect into it using all Gamma_b for all the blades.
            5. get the new alpha_e
            6. compute_airfoil_aerodynamics to get the new value cl
            7. then update Gamma_b using the relaxation factor
            8. iterate from 4 to 7 till Gamma_b converges

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
    # ------------------------------------------------------------------------------------------------------------------
    #  Unpack
    # ------------------------------------------------------------------------------------------------------------------
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
    max_iter_Gammab = wake_inputs.max_iter_Gammab # 50
    max_iter_CT     = wake_inputs.max_iter_CT # 50
    tol             = wake_inputs.tol # 1e-4

    # relax may arrive as a scalar (flat, one value for every control point) or a
    # (ctrl_pts,)-per-point array -- normalize to (ctrl_pts,1,1) so it's always indexable by
    # valid_cp below and broadcasts against Gamma_b (ctrl_pts, Nr-1, B).
    relax = np.asarray(wake_inputs.relax, dtype=float).reshape(-1)
    if relax.size == 1:
        relax = np.full(ctrl_pts, relax[0])
    relax = relax[:, np.newaxis, np.newaxis]
    mu              = wake_inputs.mu       # (ctrl_pts,) -- per-control-point edgewise advance ratio
    mu_max          = wake_inputs.mu_max   # scalar threshold

    nodes_14c = wake_inputs.nodes_14c   # (ctrl_pts, Nr, B, 3), body frame inducing location
    nodes_34c = wake_inputs.nodes_34c   # (ctrl_pts, Nr, B, 3), body frame induced location

    B        = rotor.number_of_blades
    R        = rotor.tip_radius
    tc       = rotor.thickness_to_chord
    a_loc    = rotor.airfoil_polar_stations
    airfoils = rotor.airfoils
    r_1d     = rotor.radius_distribution   # (Nr,)

    psi = rotor.blades.bound.psi   # (Nr, B)

    psi     = 0.5*(psi[:-1, :]          + psi[1:, :]          )   # (Nr-1, B)
    Ua      = 0.5*(Ua[:,  :-1, :]       + Ua[:,   1:, :]      )   # (ctrl_pts, Nr-1, B)
    Ut      = 0.5*(Ut[:,  :-1, :]       + Ut[:,   1:, :]      )   # (ctrl_pts, Nr-1, B)
    U       = 0.5*(U[:,   :-1, :]       + U[:,    1:, :]      )   # (ctrl_pts, Nr-1, B)
    beta    = 0.5*(beta[:, :-1,:]       + beta[:, 1:, :]      )   # (ctrl_pts, Nr-1, B)
    c       = 0.5*(c[:,   :-1, :]       + c[:,    1:, :]      )   # (ctrl_pts, Nr-1, B)
    r       = 0.5*(r[:,   :-1, :]       + r[:,    1:, :]      )   # (ctrl_pts, Nr-1, B)
    a_sound = 0.5*(a_sound[:,   :-1, :] + a_sound[:,    1:, :])   # (ctrl_pts, Nr-1, B)
    nu      = 0.5*(nu[:,   :-1, :]      + nu[:,    1:, :]     )   # (ctrl_pts, Nr-1, B)
    tc      = 0.5*(tc[:-1]           + tc[1:]           )   # (Nr-1, )

    a_loc = a_loc[:-1] # taken as the value at the inboard node
    
    # ------------------------------------------------------------------------------------------------------------------
    #  Unpack -- CT_iter mode only
    # ------------------------------------------------------------------------------------------------------------------
    rho   = conditions.freestream.density[:, 0, None]
    T     = conditions.freestream.temperature[:, 0, None, None]
    rho_0 = rho
    omega = wake_inputs.omega
    #commanded_TV = conditions.energy.converters[rotor.tag].commanded_thrust_vector_angle
    #pitch_c      = conditions.energy.converters[rotor.tag].blade_pitch_command
    #eta          = conditions.energy.converters[rotor.tag].throttle
    #design_flag  = conditions.energy.converters[rotor.tag].design_flag
    if wake_inputs.CT_iter:
        diff_r    = np.diff(r_1d)                                                     # (Nr-1,)
        deltar_3d = diff_r[np.newaxis, :, np.newaxis] * np.ones((ctrl_pts, Nr-1, B))  # (ctrl_pts, Nr-1, B)
    '''
    # ------------------------------------------------------------------------------------------------------------------
    #  Wake shedding radius
    # ------------------------------------------------------------------------------------------------------------------
    if wake_inputs.include_wake:
        r_R_shed = wake_inputs.get('r_R_shed', 1.0)
        R_shed   = r_R_shed * R
        i_shed      = np.argmin(np.abs(r_1d - R_shed))            # nearest node
        R_shed = r_1d[i_shed]
    '''
    if wake_inputs.include_wake:
        N_wake   = rotor.blades.wake.N_wake

    CW = omega[:, 0] > 0   # (ctrl_pts,) -- per-control-point rotation sense
    CW_3 = CW[:, np.newaxis, np.newaxis]   # (ctrl_pts, 1, 1) -- broadcast helper

    A         = (np.pi * R**2)
    omegar    = omega*R
    omegar_sq = omegar**2

    # ------------------------------------------------------------------------------------------------------------------
    #  Skip control points outside the model's valid advance-ratio range
    #  (edgewise mu, not the axial/climb component -- see lifting_line_performance.py).
    #  These are excluded from the convergence checks below so a single garbage
    #  control point can't block the whole batch from registering as converged.
    # ------------------------------------------------------------------------------------------------------------------
    valid_cp = (mu <= mu_max)   # (ctrl_pts,)
    if np.any(~valid_cp):
        print(f"Skipping control point(s) {np.where(~valid_cp)[0].tolist()} -- "
              f"edgewise advance ratio mu={mu[~valid_cp]} exceeds mu_max={mu_max}. "
              f"Out of valid range for this method; not iterating on these.")

    # ------------------------------------------------------------------------------------------------------------------
    #  Unit vectors in thrust frame -- (ctrl_pts, Nr-1, B)
    #  z-components don't actually depend on CW, but are still routed through
    #  np.where(CW_3, ...) so all four carry the same (ctrl_pts, Nr-1, B) shape
    #  as the y-components -- otherwise these silently stay (Nr-1, B) with no
    #  ctrl_pts axis, which corrupts ut_ind/Wt (but not ua_ind/Wa) downstream.
    # ------------------------------------------------------------------------------------------------------------------
    radial_hat_thrust_y = np.where(CW_3, -np.sin(psi), np.sin(psi))
    radial_hat_thrust_z = np.where(CW_3,  np.cos(psi), np.cos(psi))
    tang_hat_thrust_y   = np.where(CW_3, -np.cos(psi), np.cos(psi))
    tang_hat_thrust_z   = np.where(CW_3, -np.sin(psi), -np.sin(psi))

    # ------------------------------------------------------------------------------------------------------------------
    #  Initial guess: Cl from freestream, Gamma_b = 0.5*U*c*Cl
    # ------------------------------------------------------------------------------------------------------------------
    Cl, _, _, _, _, _, _, _ = compute_airfoil_aerodynamics(
        beta, c, r, R, B, Ua, Ut, a_sound, nu, airfoils, a_loc, ctrl_pts, Nr-1, B, tc, use_2d_analysis=True)
    
    Gamma_b = 0.5*U*c*Cl

    # ------------------------------------------------------------------------------------------------------------------
    #  Pre-compute bound influence matrix K_bound (geometry fixed throughout)
    # ------------------------------------------------------------------------------------------------------------------
    P_colloc = nodes_34c.reshape(ctrl_pts, (Nr-1)*B, 3)

    A_bound  = nodes_14c[:, :-1, :, :].reshape(ctrl_pts, (Nr-1)*B, 3)
    B_bound  = nodes_14c[:, 1:,  :, :].reshape(ctrl_pts, (Nr-1)*B, 3)
    rCb      = rotor.blades.bound.rCb

    K_bound = np.zeros((ctrl_pts, (Nr-1)*B, (Nr-1)*B, 3))
    for cp in range(ctrl_pts):
        rCb_flat    = rCb[cp, :, :].reshape((Nr-1)*B)
        K_bound[cp] = biot_savart_velocity_induction(
            P_colloc[cp], A_bound[cp], B_bound[cp], rCb_flat, wake_inputs.vc_correction)

    # ------------------------------------------------------------------------------------------------------------------
    #  Pre-compute wake influence matrix K_wake
    #  In CT_iter mode: K_wake is rebuilt each outer CT iteration (wake geometry changes)
    #  In simple mode:  K_wake is computed once before the loop
    # ------------------------------------------------------------------------------------------------------------------
    if wake_inputs.include_wake:
        K_wake = np.zeros((ctrl_pts, (Nr-1)*B, N_wake*B, 3))
        if not wake_inputs.CT_iter:
            # compute wake geometry once
            A_wake = rotor.blades.wake.nodes_body[:, :-1, :, :].reshape(ctrl_pts, N_wake*B, 3)
            B_wake = rotor.blades.wake.nodes_body[:, 1:,  :, :].reshape(ctrl_pts, N_wake*B, 3)
            rCvf   = rotor.blades.wake.rCvf
            for cp in range(ctrl_pts):
                rCvf_flat   = np.repeat(rCvf[cp], B)
                K_wake[cp]  = biot_savart_velocity_induction(
                    P_colloc[cp], A_wake[cp], B_wake[cp], rCvf_flat, wake_inputs.vc_correction)

    # ------------------------------------------------------------------------------------------------------------------
    #  Outer loop: CT (CT_iter=True) or single pass (CT_iter=False)
    # ------------------------------------------------------------------------------------------------------------------
    conv     = False
    n_outer  = max_iter_CT if wake_inputs.CT_iter else 1

    for it in range(n_outer):

        # -- CT_iter mode: rebuild K_wake each outer iteration --
        if wake_inputs.CT_iter and wake_inputs.include_wake:
            A_wake = rotor.blades.wake.nodes_body[:, :-1, :, :].reshape(ctrl_pts, N_wake*B, 3)
            B_wake = rotor.blades.wake.nodes_body[:, 1:,  :, :].reshape(ctrl_pts, N_wake*B, 3)
            rCvf   = rotor.blades.wake.rCvf
            for cp in range(ctrl_pts):
                rCvf_flat  = np.repeat(rCvf[cp], B)
                K_wake[cp] = biot_savart_velocity_induction(
                    P_colloc[cp], A_wake[cp], B_wake[cp], rCvf_flat, wake_inputs.vc_correction)

        # -- Inner Gamma_b loop --
        conv1     = False
        diverged  = False
        for it1 in range(max_iter_Gammab):

            # Step 4a: bound vortex induction
            Gamma_bound = Gamma_b.reshape(ctrl_pts, (Nr-1)*B)
            v_induced_bound_body = np.zeros((ctrl_pts, (Nr-1)*B, 3))
            for cp in range(ctrl_pts):
                v_induced_bound_body[cp] = np.einsum('mnk,n->mk', K_bound[cp], Gamma_bound[cp])
            v_induced_bound_body = v_induced_bound_body.reshape(ctrl_pts, Nr-1, B, 3)

            # Step 4b: wake induction
            v_induced_wake_body = np.zeros((ctrl_pts, (Nr-1)*B, 3))
            if wake_inputs.include_wake:
                '''
                if R_shed_near >= r_1d[-1]:
                    Gamma_wake = Gamma_b[:, -1:, :]
                elif R_shed_near <= r_1d[0]:
                    Gamma_wake = -Gamma_b[:, 0:1, :]
                else:
                    Gamma_wake = Gamma_b[:, i_shed-1:i_shed, :] - Gamma_b[:, i_shed:i_shed+1, :]   # difference
                '''

                # Use max Gamma_b instead of shed station value
                Gamma_wake      = np.max(Gamma_b, axis=1, keepdims=True)   # (ctrl_pts, 1, B)

                Gamma_wake      = Gamma_wake * np.ones((ctrl_pts, N_wake, B))
                Gamma_wake_flat = Gamma_wake.reshape(ctrl_pts, N_wake*B)

                for cp in range(ctrl_pts):
                    v_induced_wake_body[cp] = np.einsum('mnk,n->mk', K_wake[cp], Gamma_wake_flat[cp])
                v_induced_wake_body = v_induced_wake_body.reshape(ctrl_pts, Nr-1, B, 3)

            v_induced_body = v_induced_bound_body + v_induced_wake_body

            # Transform body -> thrust frame
            v_induced_thrust = np.einsum('cij,crbj->crbi', T_body2thrust, v_induced_body)

            # Project onto axial, tangential
            # radial_hat_thrust_*/tang_hat_thrust_* are already (ctrl_pts, Nr-1, B) -- no extra axis needed
            ua_ind = v_induced_thrust[:,:,:,0]
            ur_ind = (v_induced_thrust[:,:,:,1]*radial_hat_thrust_y +
                       v_induced_thrust[:,:,:,2]*radial_hat_thrust_z)
            ut_ind = (v_induced_thrust[:,:,:,1]*tang_hat_thrust_y +
                      v_induced_thrust[:,:,:,2]*tang_hat_thrust_z)

            # Sign checked against BEMT_Helmholtz_performance.py's Wa = va + Ua, Wt = Ut - vt:
            # these look like opposite signs on Wa, but they're not -- BEMT's va is *defined* as
            # va = Wa - Ua (positive-for-downwash, from its own PSI/inflow-angle iteration), while
            # ua_ind here comes directly out of the Biot-Savart law on the bound circulation and
            # comes out negative for the same physical downwash (ua_ind = -va), given this
            # circulation/segment-direction convention. Verified numerically on the converged
            # design case: Ua~3.4, ua_ind~-53, giving Wa~56.5 > Ua, i.e. accelerated flow through
            # a thrusting disk, matching momentum theory. Using "+" here instead (to textually
            # match BEMT) would make Wa go negative -- confirmed wrong, not a fix.
            Wa = np.where(CW_3, Ua - ua_ind, Ua + ua_ind)
            Wt = np.where(CW_3, Ut - ut_ind, Ut + ut_ind)
                
            W  = np.sqrt(Wa**2 + Wt**2)

            # Aerodynamics
            if wake_inputs.aerofoil_aero == 1:
                # Simplified analytic aero 
                alpha      = beta - np.arctan2(Wa, Wt)
                Ma         = W / a_sound
                Re         = (W * c) / nu
                alpha_disc = alpha
                Re_disc    = Re
                Cl       = (2.*np.pi/6.) * np.sin(6.*alpha)  # Cl_a = 2 * pi
                # Karman-Tsien compressibility correction -- same formula/guard as BET_calculations.py
                KT_cond      = np.logical_and(Ma < 1., Cl > 0)
                Cl[KT_cond]  = Cl[KT_cond] / ((1. - Ma[KT_cond]*Ma[KT_cond])**0.5 +
                               (Ma[KT_cond]*Ma[KT_cond] / (1. + (1. - Ma[KT_cond]*Ma[KT_cond])**0.5)) * Cl[KT_cond]/2.)
                Cdval    = 0.0087 - 0.0216*alpha + 0.4*alpha**2
                Tw_Tinf  = 1. + 1.78*(Ma*Ma)
                Tp_Tinf  = 1. + 0.035*(Ma*Ma) + 0.45*(Tw_Tinf-1.)
                Tp       = Tp_Tinf * T
                Rp_Rinf  = (Tp_Tinf**2.5)*(Tp+110.4)/(T+110.4)
                Cd       = ((1/Tp_Tinf)*(1/Rp_Rinf)**0.2)*Cdval
            elif wake_inputs.aerofoil_aero == 2:
                Cl, Cdval, alpha, alpha_disc, Ma, _, Re, Re_disc = compute_airfoil_aerodynamics(
                    beta, c, r, R, B, Wa, Wt, a_sound, nu, airfoils, a_loc, ctrl_pts, Nr-1, B, tc, use_2d_analysis=True)
                Tw_Tinf  = 1. + 1.78*(Ma*Ma)
                Tp_Tinf  = 1. + 0.035*(Ma*Ma) + 0.45*(Tw_Tinf-1.)
                Tp       = Tp_Tinf * T
                Rp_Rinf  = (Tp_Tinf**2.5)*(Tp+110.4)/(T+110.4)
                Cd       = ((1/Tp_Tinf)*(1/Rp_Rinf)**0.2)*Cdval

            # Prandtl tip loss
            lamdaw, F, _ = compute_lifting_line_inflow_and_tip_loss(r, R, Wa, Wt,  B)

            Cl = Cl * F

            # Relaxed Gamma_b update
            Gamma_b_new = 0.5*W*c*Cl

            # Only flag divergence among VALID control points -- an out-of-range (high mu)
            # control point is expected to potentially blow up and shouldn't halt the whole batch.
            bad_cp = valid_cp & ~np.all(np.isfinite(Gamma_b_new) & (np.abs(Gamma_b_new) < 1e4), axis=(1, 2))   # (ctrl_pts,)
            if np.any(bad_cp):
                print(f"Gamma_b diverged at control point(s) {np.where(bad_cp)[0].tolist()} "
                      f"at inner iteration {it1+1}. Stopping.")
                diverged = True
                break

            residual_Gamma_b = np.max(np.abs(Gamma_b_new - Gamma_b)[valid_cp]) if np.any(valid_cp) else 0.0

            # Only update Gamma_b for valid control points -- invalid ones stay frozen at
            # their initial freestream-only guess rather than being iteratively (and pointlessly) refined.
            Gamma_b[valid_cp] = Gamma_b[valid_cp] + relax[valid_cp]*(Gamma_b_new[valid_cp] - Gamma_b[valid_cp])

            if residual_Gamma_b < tol:
                print("Gamma_b converged after", it1+1, "iterations")
                conv1 = True
                break

        if diverged:
            break   # stop the outer CT loop too -- no point continuing once it has blown up

        if not conv1:
            print("Gamma_b did not converge. Residual =", residual_Gamma_b)

        # -- CT check (CT_iter mode only) --
        if wake_inputs.CT_iter:
            epsilon                    = Cd / (Cl + 1e-300)
            epsilon[np.abs(Cl) <= 1e-6] = 10.0 * np.sign(Cl[np.abs(Cl) <= 1e-6])

            blade_T_distribution = rho[:, :, None] * (Gamma_b_new*(Wt - epsilon*Wa)) * deltar_3d
            thrust               = np.sum(blade_T_distribution, axis=(1, 2))[:, None]  # (ctrl_pts, 1)
            Ct_rotor_new         = thrust / (rho_0 * A * omegar_sq)

            bad_cp_ct = valid_cp & ~np.isfinite(Ct_rotor_new[:, 0])
            if np.any(bad_cp_ct):
                print(f"CT diverged to NaN at control point(s) {np.where(bad_cp_ct)[0].tolist()} "
                      f"at outer iteration {it+1}. Stopping.")
                break

            print("CT", Ct_rotor_new)

            residual_CT = np.max(np.abs(Ct_rotor_new - wake_inputs.thrust_coeff_initial_guess)[valid_cp]) if np.any(valid_cp) else 0.0
            if residual_CT < (0.1*tol):
                print("CT converged after", it+1, "outer iterations")
                conv = True
                break

            # Only carry forward CT for valid control points -- invalid ones keep their
            # existing (unrefined) CT rather than feeding a meaningless value into the next
            # wake-geometry rebuild. wake_inputs.thrust_coeff_initial_guess may still be the raw
            # scalar default on the first outer iteration, so broadcast it to Ct_rotor_new's
            # shape before copying.
            new_CT = np.broadcast_to(np.asarray(wake_inputs.thrust_coeff_initial_guess, dtype=float), Ct_rotor_new.shape).copy()
            new_CT[valid_cp] = Ct_rotor_new[valid_cp]
            wake_inputs.thrust_coeff_initial_guess = new_CT   # (ctrl_pts, 1) -- one CT per control point

            if wake_inputs.include_wake:
                initialize_wake_geometry(rotor, wake_inputs, conditions)
        else:
            conv = True   # simple mode always exits after one outer pass

    if not conv and wake_inputs.CT_iter:
        print("CT did not converge. Residual =", residual_CT)

    # ------------------------------------------------------------------------------------------------------------------
    #  Store converged results
    # ------------------------------------------------------------------------------------------------------------------
    rotor.blades.bound.gamma      = Gamma_b 
    rotor.blades.bound.Cl         = Cl 
    rotor.blades.bound.Cdval      = Cdval
    rotor.blades.bound.Cd         = Cd
    rotor.blades.bound.alpha      = alpha 
    rotor.blades.bound.alpha_disc = alpha_disc
    rotor.blades.bound.Ma         = Ma
    rotor.blades.bound.Re         = Re
    rotor.blades.bound.Re_disc    = Re_disc
    rotor.blades.bound.U          = U
    rotor.blades.bound.Ua         = Ua
    rotor.blades.bound.Ut         = Ut
    rotor.blades.bound.W          = W
    rotor.blades.bound.Wa         = Wa
    rotor.blades.bound.Wt         = Wt
    rotor.blades.bound.F          = F
    rotor.blades.bound.va         = ua_ind
    rotor.blades.bound.vt         = ut_ind
    rotor.blades.bound.lamdaw     = lamdaw

    if wake_inputs.include_wake:
        rotor.blades.wake.gamma = Gamma_wake.reshape(ctrl_pts, N_wake, B)

    return