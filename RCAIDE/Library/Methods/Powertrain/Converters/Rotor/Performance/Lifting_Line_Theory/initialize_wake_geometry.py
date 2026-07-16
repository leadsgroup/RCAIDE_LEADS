# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/initialize_wake_geometry.py
#
# Created:  Jun 2026, H. Hussien
#
# Initializes the tip vortex wake geometry. Three models selectable via wake_inputs.wake_model:
#   1 -- Simple: single-rate axial convection using momentum theory lam + muzs
#   2 -- Landgrebe (1972): piecewise axial convection k1/k2 + lam
#   3 -- Kocurek-Tangler (1977): same as Landgrebe with KT empirical constants
#
# References
# ----------
# [1] Landgrebe, A.J., JAHS Vol. 17 No. 4, 1972.
# [2] Kocurek, J.D. and Tangler, J.L., JAHS Vol. 22 No. 1, 1977.
# [3] W. Johnson, Rotorcraft Aeromechanics, Cambridge University Press, 2013.
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from RCAIDE.Framework.Core import Data, orientation_transpose

# ----------------------------------------------------------------------------------------------------------------------
#  initialize_wake_geometry
# ----------------------------------------------------------------------------------------------------------------------
def initialize_wake_geometry(rotor, wake_inputs, conditions):
    """
    Initializes the tip vortex wake geometry.

    Parameters
    ----------
    rotor : Data
    wake_inputs : Data
        wake_model    : int   -- 1=simple, 2=Landgrebe, 3=KT
        V_thrust      : (ctrl_pts, 3)
        T_body2thrust : (ctrl_pts, 3, 3)
        omega         : (ctrl_pts, 1)
        dpsi          : float
        n_turns       : float
        CT            : float or (ctrl_pts,) array_like
        r_R_shed      : float  -- wake shedding fraction (default 1.0 = tip)
        lamb_oseen_*  : float  -- Lamb-Oseen core growth parameters
    conditions : Data

    Returns
    -------
    None -- populates rotor.blades.wake
    """
    # ------------------------------------------------------------------------------------------------------------------
    #  Unpack
    # ------------------------------------------------------------------------------------------------------------------
    B        = rotor.number_of_blades
    R        = rotor.tip_radius
    r_1d     = rotor.radius_distribution                   # (Nr,)
    rho      = conditions.freestream.density
    nu       = conditions.freestream.dynamic_viscosity[:, 0] / rho[:, 0]
    sweep    = rotor.sweep_distribution                    # (Nr,)
    c        = rotor.chord_distribution                    # (Nr,)
    beta_0   = rotor.twist_distribution                    # (Nr,)
    Nr       = len(r_1d)
    rc       = rotor.rc

    commanded_TV = conditions.energy.converters[rotor.tag].commanded_thrust_vector_angle
    pitch_c      = conditions.energy.converters[rotor.tag].blade_pitch_command
    ctrl_pts     = len(conditions.frames.inertial.velocity_vector)

    V_thrust      = wake_inputs.V_thrust
    T_body2thrust = wake_inputs.T_body2thrust
    omega         = wake_inputs.omega
    dpsi          = wake_inputs.get('dpsi',    np.radians(15.0))
    n_turns       = wake_inputs.get('n_turns', 5.0)
    wake_model    = wake_inputs.get('wake_model', 1)

    # CT may arrive as a scalar default (first call) or a per-control-point
    # array (subsequent CT-convergence iterations) -- normalize to (ctrl_pts,)
    CT = np.asarray(wake_inputs.get('CT', 0.005), dtype=float).reshape(-1)
    if CT.size == 1:
        CT = np.full(ctrl_pts, CT[0])

    lamb_oseen_alpha             = wake_inputs.lamb_oseen_alpha
    lamb_oseen_delta             = wake_inputs.lamb_oseen_delta
    lamb_oseen_sigma             = wake_inputs.lamb_oseen_sigma
    lamb_oseen_core_growth_delay = wake_inputs.lamb_oseen_core_growth_delay
    lamb_oseen_rc_0              = wake_inputs.lamb_oseen_rc_0

    T_thrust2body = orientation_transpose(T_body2thrust)
    
    # ------------------------------------------------------------------------------------------------------------------
    #  Step 1: Wake age array
    # ------------------------------------------------------------------------------------------------------------------
    wakeage = np.arange(0.0, 2.0*np.pi*n_turns + dpsi, dpsi)   # (N_wake+1,) number of nodes
    N_wake  = len(wakeage) - 1 # number of filaments

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 2: Advance ratios
    # ------------------------------------------------------------------------------------------------------------------
    Uh     = V_thrust[:, 0]
    Vh     = V_thrust[:, 1]
    Wh     = V_thrust[:, 2]
    omega  = np.where(omega == 0, 1e-6, omega)
    omegaR = omega[:, 0] * R
    muxs   = Uh / np.abs(omegaR)
    muys   = Vh / np.abs(omegaR)
    muzs   = Wh / np.abs(omegaR)

    CW = omega[:, 0] > 0   # (ctrl_pts,) -- per-control-point rotation sense
    CW_3 = CW[:, np.newaxis, np.newaxis]   # (ctrl_pts, 1, 1) -- broadcast helper

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 3: Induced inflow ratio (momentum theory)
    # ------------------------------------------------------------------------------------------------------------------
    T_thrust = CT * rho[:, 0] * omegaR**2 * (np.pi * R**2)
    viavg    = np.sqrt(np.maximum(T_thrust / (2.0 * rho[:, 0] * np.pi * R**2), 1e-6))
    lam      = viavg / np.abs(omegaR)

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 4: Wake contraction and axial convection -- MODEL DEPENDENT
    # ------------------------------------------------------------------------------------------------------------------
    if wake_model == 1:
        # Simple: Landgrebe k3/k4 contraction + single-rate axial convection
        Lambda    = 0.145 + 27.0 * CT   # (ctrl_pts,)
        #bad_CT = CT <= 0
        #if np.any(bad_CT):
        #    print(f"Warning: control point(s) {np.where(bad_CT)[0].tolist()} have "
        #          f"CT={CT[bad_CT]} driving the wake-contraction rate "
        #          f"Clamping CT to a small positive floor, 1e-3.")
        #Lambda = np.where(bad_CT, 0.145 + 27.0 * 1e-3, Lambda)
        wcf    = 0.78 + (1.0 - 0.78) * np.exp(-Lambda[:, np.newaxis] * wakeage[np.newaxis, :])   # (ctrl_pts, N_wake+1)
    elif wake_model in (2, 3):
        # Landgrebe (2) or Landgrebe - Kocurek & Tangler (3): empirical constants from total tip pitch
        theta_tip_deg = (np.degrees(rotor.twist_distribution[-1]))

        if wake_model == 2:
            # Landgrebe - Source: DATTA's lecture notes
            k1 = 0.25 * (CT + 0.001 * theta_tip_deg)                 # near-wake axial rate (ctrl_pts,)
            k2 = (1.41 + 0.0141 * theta_tip_deg) * np.sqrt(CT/2)     # far-wake axial rate (ctrl_pts,)
            k3 = 0.145 + 27.0 * CT                                   # contraction rate (ctrl_pts,)
            k4 = 0.78
        else:
            ## Landgrebe - Kocurek & Tangler
            BB  = (-0.000729 * theta_tip_deg)
            CC  = (-2.3 + 0.206  * theta_tip_deg)
            mm  = (1.-0.25 * np.exp(-0.04 * theta_tip_deg))
            nn  = (0.5-0.0172* theta_tip_deg)
            CT0 = B**nn * (-BB/CC)**(1/mm)   # scalar -- depends only on rotor geometry
            k1  = -(BB + CC * (CT/B**nn)**mm)   # (ctrl_pts,)
            below_CT0 = CT < CT0
            if np.any(below_CT0):
                print(f"Warning: {np.sum(below_CT0)} control point(s) have CT below the "
                      f"Kocurek-Tangler far-wake threshold CT0={CT0:.5f} "
                      f"(tip pitch {theta_tip_deg:.2f} deg) -- k2 is undefined below CT0. "
                      f"Clamping k2 to 0 for those points.")
            k2 = -(-np.maximum(CT - CT0, 0.0)**(0.5))   # (ctrl_pts,) -- clamped, avoids NaN below CT0
            k3 = 4.*(CT)**0.5                            # (ctrl_pts,)
            k4 = 0.78

        phi_break = 2.0 * np.pi / B

        # Piecewise axial convection + momentum theory base rate -- (ctrl_pts, N_wake+1)
        x_landgrebe = np.where(
            wakeage[np.newaxis, :] < phi_break,
            k1[:, np.newaxis] * wakeage[np.newaxis, :],
            k1[:, np.newaxis] * phi_break + k2[:, np.newaxis] * (wakeage[np.newaxis, :] - phi_break)
        ) + (lam[:, np.newaxis] + muxs[:, np.newaxis]) * wakeage[np.newaxis, :] # (ctrl_pts, N_wake+1) non-dimensional x/R

        # Radial contraction
        wcf = k4 + (1.0 - k4) * np.exp(-k3[:, np.newaxis] * wakeage[np.newaxis, :])   # (ctrl_pts, N_wake+1)

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 5: Shed point -- Closest to the nearest point
    # ------------------------------------------------------------------------------------------------------------------
    r_R_shed    = wake_inputs.get('r_R_shed', 1.0)
    R_shed      = r_R_shed * R
    nodes       = rotor.blades.bound.nodes_hub_14c   # (ctrl_pts, Nr, B, 3)
    i_shed      = np.argmin(np.abs(r_1d - R_shed))   # gets the closest node
    R_shed_near = r_1d[i_shed]
    if R_shed_near >= r_1d[-1]:
        x_tip = nodes[:, -1, :, 0]
        y_tip = nodes[:, -1, :, 1]
        z_tip = nodes[:, -1, :, 2]
        R_shed = r_1d[-1]
    elif R_shed_near <= r_1d[0]:
        x_tip = nodes[:, 0, :, 0]
        y_tip = nodes[:, 0, :, 1]
        z_tip = nodes[:, 0, :, 2]
        R_shed = r_1d[0]
    else:
        x_tip  = nodes[:, i_shed, :, 0]
        y_tip  = nodes[:, i_shed, :, 1]
        z_tip  = nodes[:, i_shed, :, 2]
        R_shed = r_1d[i_shed]

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 6: Build wake node positions in hub/thrust frame  (ctrl_pts, N_wake+1, B, 3)
    #
    #  Sign conventions:
    #  -- Negative sign on y_c: wake is shed opposite to CW rotation direction
    #     (the tip vortex trails behind the blade, unwinding counter to blade sweep).
    #  -- Rotation is in the y-z plane about the x/axial axis.
    #     Negative sign on y_rot enforces CW shedding direction.
    #  -- Axial convection in +x: lam > 0 means downwash in +x (thrust direction).
    #     muxs > 0 (climb) reduces net downwash: x_conv += R*(lam - muxs)*wa.
    #  -- In-plane convection in +z: forward flight velocity Wh in +z sweeps
    #     the wake in +z. Sign confirmed: positive Wh -> wake moves in +z.
    # ------------------------------------------------------------------------------------------------------------------
    # shapes: wakeage (N+1,), wcf (ctrl_pts, N+1), x_tip/y_tip/z_tip (ctrl_pts, B), lam/muxs/muys/muzs (ctrl_pts,)

    nodes_hub = np.zeros((ctrl_pts, N_wake+1, B, 3))

    if wake_model == 1:
        # Simple: rotate contracted tip position by wake age in y-z plane
        cwa = np.cos(wakeage)
        swa = np.sin(wakeage)

        y_c   = -y_tip[:, np.newaxis, :] * wcf[:, :, np.newaxis]         # (ctrl_pts, N+1, B)
        z_c   =  z_tip[:, np.newaxis, :] * wcf[:, :, np.newaxis]

        y_rot = np.where(CW_3, -(cwa[np.newaxis, :, np.newaxis] * y_c - swa[np.newaxis, :, np.newaxis] * z_c),
                               -(cwa[np.newaxis, :, np.newaxis] * y_c + swa[np.newaxis, :, np.newaxis] * z_c))

        z_rot = np.where(CW_3,  (swa[np.newaxis, :, np.newaxis] * y_c + cwa[np.newaxis, :, np.newaxis] * z_c),
                               -(swa[np.newaxis, :, np.newaxis] * y_c - cwa[np.newaxis, :, np.newaxis] * z_c))

        # axial convection -- broadcast (ctrl_pts,) with (N+1,) and (B,) -> (ctrl_pts, N+1, B)
        x_conv = (x_tip[:, np.newaxis, :] +
                  R_shed * (lam[:, np.newaxis, np.newaxis] + muxs[:, np.newaxis, np.newaxis]) *
                  wakeage[np.newaxis, :, np.newaxis])

        y_conv = (y_rot +
                  R_shed * (muys[:, np.newaxis, np.newaxis]) * wakeage[np.newaxis, :, np.newaxis])
        z_conv = (z_rot +
                  R_shed * (muzs[:, np.newaxis, np.newaxis]) * wakeage[np.newaxis, :, np.newaxis])

    elif wake_model in (2, 3):
        # Landgrebe/KT: azimuthal position from psi_blade - wakeage
        psi_blade = rotor.blades.bound.psi[-1, :]   # (B,) tip azimuth

        psi_wake     = psi_blade[np.newaxis, :] - wakeage[:, np.newaxis]   # (N+1, B)
        r_contracted = R_shed * wcf                                          # (ctrl_pts, N+1)

        y_wake = -r_contracted[:, :, np.newaxis] * np.sin(psi_wake)[np.newaxis, :, :]   # (ctrl_pts, N+1, B)
        z_wake =  r_contracted[:, :, np.newaxis] * np.cos(psi_wake)[np.newaxis, :, :]

        x_conv = (x_tip[:, np.newaxis, :] +
                  R_shed * x_landgrebe[:, :, np.newaxis])              # (ctrl_pts, N+1, B)
        y_conv = (y_wake +
                  R_shed * muys[:, np.newaxis, np.newaxis] * wakeage[np.newaxis, :, np.newaxis])
        z_conv = (z_wake +
                  R_shed * muzs[:, np.newaxis, np.newaxis] * wakeage[np.newaxis, :, np.newaxis])

    nodes_hub[:, :, :, 0] = x_conv
    nodes_hub[:, :, :, 1] = y_conv
    nodes_hub[:, :, :, 2] = z_conv

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 7: Transform to body frame
    # ------------------------------------------------------------------------------------------------------------------
    hub_origin = np.array(rotor.origin[0]).reshape(1, 1, 1, 3)
    nodes_body = np.einsum('cij,cwbj->cwbi', T_thrust2body, nodes_hub) + hub_origin

    # ------------------------------------------------------------------------------------------------------------------
    #  Step 8: Lamb-Oseen core radius
    # ------------------------------------------------------------------------------------------------------------------
    wakeage_core = np.maximum(wakeage[:-1] - (lamb_oseen_core_growth_delay), 0.0) # radians

    rCvf = np.sqrt(
        (lamb_oseen_rc_0 * R)**2 +
        4 * lamb_oseen_alpha * lamb_oseen_delta * lamb_oseen_sigma *
        (nu[:, None] / np.abs(omega[:, 0, None])) * wakeage_core[None, :]
    )   # (ctrl_pts, N_wake)

    # ------------------------------------------------------------------------------------------------------------------
    #  Store on rotor.blades.wake
    # ------------------------------------------------------------------------------------------------------------------
    rotor.blades.wake = Data()

    rotor.blades.wake.nodes_hub  = nodes_hub   # (ctrl_pts, N_wake+1, B, 3)
    rotor.blades.wake.nodes_body = nodes_body  # (ctrl_pts, N_wake+1, B, 3)
    rotor.blades.wake.wakeage    = wakeage     # (N_wake+1,)
    rotor.blades.wake.N_wake     = N_wake
    rotor.blades.wake.wcf        = wcf         # (ctrl_pts, N_wake+1)
    rotor.blades.wake.muxs       = muxs          # (ctrl_pts,)
    rotor.blades.wake.muys       = muys          # (ctrl_pts,)
    rotor.blades.wake.muzs       = muzs          # (ctrl_pts,)
    rotor.blades.wake.lam        = lam         # (ctrl_pts,)
    rotor.blades.wake.rCvf       = rCvf        # (ctrl_pts, N_wake)
    rotor.blades.wake.gamma      = np.zeros((ctrl_pts, N_wake, B)) # filled after Gamma_b converges

    return