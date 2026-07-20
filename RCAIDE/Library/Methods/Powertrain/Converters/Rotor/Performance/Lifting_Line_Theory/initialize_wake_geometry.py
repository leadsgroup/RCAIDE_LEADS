# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/initialize_wake_geometry.py
#
# Created:  Jun 2026, H. Hussien
# Modified: Jul 2026, H. Hussien -- merged hover + forward-flight modules into one file
# Modified: Jul 2026, H. Hussien -- regime dispatch (mu_edge vs mu_edgewise_threshold) and
#           CW/CCW rotation sense made per-control-point instead of batch-wide, so a single
#           call can mix hover/FF regimes and CW/CCW rotors across ctrl_pts, matching the
#           per-control-point conventions used throughout the rest of this method.
#
# Initializes the tip vortex wake geometry. AUTO-DISPATCHED PER CONTROL POINT: computes the
# in-plane (edgewise) advance ratio mu_edge = sqrt(muys**2 + muzs**2) for each control point;
# any control point at or below wake_inputs.mu_edgewise_threshold (default 1e-3) uses
# wake_inputs.wake_model_hov, the rest use wake_inputs.wake_model_FF -- which of the two model
# FAMILIES applies is NOT user-selectable, it is driven by each control point's own flight
# condition. The caller sets both wake_model_hov and wake_model_FF up front (each with its own
# default -- see the function docstring); whichever one matches a given control point's regime
# is the one that actually determines that point's wake geometry.
#
#   Hover-tailored models (wake_inputs.wake_model_hov, default 1):
#     1 -- Simple: single-rate axial convection using momentum theory lam + muxs
#     2 -- Landgrebe (1972): piecewise axial convection k1/k2 + lam
#     3 -- Kocurek-Tangler (1977): same as Landgrebe with KT empirical constants
#
#   Forward-flight models (wake_inputs.wake_model_FF, default 5; auto-dispatched when mu_edge
#   exceeds the threshold):
#     4 -- Undistorted: rigid skewed helix. The wake is translated by the edgewise/axial advance
#          ratios but the induced-velocity field across the disk is NOT modeled -- no axial
#          distortion.
#     5 -- Distorted, following the Beddoes (1985) model: same rigid skew as (4), plus a
#          closed-form, piecewise-linear (Glauert-type) non-uniform inflow distribution that
#          distorts the wake axially.
#     6 -- Modified Beddoes, per van der Wall (2000) [6]: same as (5) plus two additional terms:
#          8*E/(15*pi) and 2*muzs*y_star rotation-relative rate term.
#
# References
# ----------
# [1] Landgrebe, A.J., JAHS Vol. 17 No. 4, 1972.
# [2] Kocurek, J.D. and Tangler, J.L., JAHS Vol. 22 No. 1, 1977.
# [3] W. Johnson, Rotorcraft Aeromechanics, Cambridge University Press, 2013.
# [4] J. G. Leishman, Principles of Helicopters, Cambridge University Press, 2006.
# [5] Beddoes, T.S., "A Wake Model for High Resolution Airloads," 2nd International Conference
#     on Basic Rotorcraft Research, Research Triangle Park, NC, 1985.
# [6] van der Wall, B. G., "The Effect of HHC on the Vortex Convection in the Wake of a
#     Helicopter Rotor," Aerospace Science and Technology, 4 (2000), pp. 321-336.
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from scipy import optimize
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
        wake_model_hov : int  -- hover-regime model, used per control point when its mu_edge
                                  is at/below mu_edgewise_threshold (default 1). 1=Simple,
                                  2=Landgrebe, 3=Kocurek-Tangler
        wake_model_FF  : int  -- forward-flight-regime model, used per control point when its
                                  mu_edge exceeds mu_edgewise_threshold (default 5).
                                  4=undistorted, 5=Beddoes distorted, 6=modified Beddoes
        mu_edgewise_threshold : float -- in-plane advance ratio at/above which a control point
                                  is routed to wake_model_FF instead of wake_model_hov (default
                                  1e-3). This is a per-control-point regime switch driven by the
                                  actual flight condition (mu_edge), not a model request.
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
    #  Unpack -- common to all models
    # ------------------------------------------------------------------------------------------------------------------
    B        = rotor.number_of_blades
    R        = rotor.tip_radius
    r_1d     = rotor.radius_distribution
    rho      = conditions.freestream.density
    nu       = conditions.freestream.dynamic_viscosity[:, 0] / rho[:, 0]
    ctrl_pts = rho.shape[0]

    V_thrust       = wake_inputs.V_thrust
    T_body2thrust  = wake_inputs.T_body2thrust
    omega          = wake_inputs.omega
    omega          = np.where(omega == 0, 1e-6, omega)
    dpsi           = wake_inputs.get('dpsi',    np.radians(15.0))
    n_turns        = wake_inputs.get('n_turns', 5.0)
    wake_model_hov = wake_inputs.get('wake_model_hov', 1)   # 1=Simple, 2=Landgrebe, 3=KT
    wake_model_FF  = wake_inputs.get('wake_model_FF',  5)   # 4=undistorted, 5=Beddoes, 6=modified Beddoes
    sigma          = wake_inputs.get('sigma',    0.08)

    # CT may arrive as a scalar default (first call) or a per-control-point array (subsequent
    # CT-convergence iterations) -- normalize to (ctrl_pts,)
    CT = np.asarray(wake_inputs.get('CT', 0.0065), dtype=float).reshape(-1)
    if CT.size == 1:
        CT = np.full(ctrl_pts, CT[0])

    lamb_oseen_alpha             = wake_inputs.lamb_oseen_alpha
    lamb_oseen_delta             = wake_inputs.lamb_oseen_delta
    lamb_oseen_sigma             = wake_inputs.lamb_oseen_sigma
    lamb_oseen_core_growth_delay = wake_inputs.lamb_oseen_core_growth_delay
    lamb_oseen_rc_0              = wake_inputs.lamb_oseen_rc_0

    T_thrust2body = orientation_transpose(T_body2thrust)

    # ------------------------------------------------------------------------------------------------------------------
    #  Wake age array
    # ------------------------------------------------------------------------------------------------------------------
    wakeage = np.arange(0.0, 2.0*np.pi*n_turns + dpsi, dpsi)   # (N_wake+1,) number of nodes
    N_wake  = len(wakeage) - 1 # number of filaments

    # ------------------------------------------------------------------------------------------------------------------
    #  Advance ratios -- (ctrl_pts,), shared by both regimes (needed for the per-control-point
    #  mu_edge dispatch decision below regardless of which branch ends up running for a point).
    # ------------------------------------------------------------------------------------------------------------------
    Uh     = V_thrust[:, 0]
    Vh     = V_thrust[:, 1]
    Wh     = V_thrust[:, 2]
    omegaR = omega[:, 0] * R
    muxs   = Uh / np.abs(omegaR)
    muys   = Vh / np.abs(omegaR)
    muzs   = Wh / np.abs(omegaR)
    mu     = np.sqrt(Vh**2 + Wh**2) / np.abs(omegaR)   # in-plane (edgewise) advance ratio

    CW   = omega[:, 0] > 0   # (ctrl_pts,) -- per-control-point rotation sense
    CW_3 = CW[:, np.newaxis, np.newaxis]   # (ctrl_pts, 1, 1) -- broadcast helper

    nodes_hub = np.zeros((ctrl_pts, N_wake+1, B, 3))

    # ------------------------------------------------------------------------------------------------------------------
    #  Induced inflow ratio -- Glauert forward-flight solution, shared by both regimes.
    # ------------------------------------------------------------------------------------------------------------------
    # lam = mu*tan(alpha_TPP) + CT/(2*sqrt(mu**2+(lam+muxs)**2)) is implicit in lam. It's already
    # in lam = f(lam) form, so solve it with a fixed-point iteration. Seed with the momentum-
    # theory hover value: it's the exact fixed point at mu=0 AND muxs=0, and a good starting
    # guess otherwise. At mu=0 with muxs != 0 this correctly reduces to the climb/descent-
    # corrected hover inflow equation (lam^2 + lam*muxs - CT/2 = 0) -- unlike a plain hover
    # momentum-theory formula, which would ignore muxs's effect on lam itself. Verified
    # numerically identical to the old hover-only formula at muxs=0.
    #
    # Tip-path-plane angle of attack -- angle between the rotor disk and the freestream. In
    # classical rotorcraft usage this comes from a trim solve; this model has none, and in a
    # propeller-like installation "tilted 90 deg vs 0 deg" isn't well defined the way it is
    # for a helicopter rotor. Taking alpha_TPP = 0 for now (freestream assumed in-plane).
    alpha_TPP = 0.0

    T_thrust = CT * rho[:, 0] * omegaR**2 * (np.pi * R**2)
    viavg    = np.sqrt(np.maximum(T_thrust / (2.0 * rho[:, 0] * np.pi * R**2), 1e-6))
    lam0     = viavg / np.abs(omegaR)

    def _glauert_inflow(lam):
        return mu*np.tan(alpha_TPP) + (CT/2.0) / np.sqrt(mu**2 + (lam+muxs)**2)

    try:
        lam = optimize.fixed_point(_glauert_inflow, lam0)
    except RuntimeError:
        print(f"Warning: Glauert inflow fixed-point iteration failed to converge -- "
            f"falling back to momentum-theory seed lam0.")
        lam = lam0

    # ------------------------------------------------------------------------------------------------------------------
    #  Regime dispatch -- per control point (NOT a single batch-wide choice: two control points
    #  in the same call can independently land in the hover and FF branches below).
    # ------------------------------------------------------------------------------------------------------------------
    mu_edge               = np.sqrt(muys**2 + muzs**2)
    mu_edgewise_threshold = wake_inputs.get('mu_edgewise_threshold', 1e-2)
    is_edgewise           = mu_edge > mu_edgewise_threshold   # (ctrl_pts,)
    is_edgewise_3         = is_edgewise[:, np.newaxis, np.newaxis]   # (ctrl_pts, 1, 1) -- broadcast helper

    # ==================================================================================================================
    #  HOVER-TAILORED MODELS (1, 2, 3) -- computed for every control point; only the points with
    #  is_edgewise == False actually use this result (selected in the combine step below).
    # ==================================================================================================================
    # lam (induced inflow ratio) is computed once above, shared with the FF regime.

    # --------------------------------------------------------------------------------------------------------------
    #  Wake contraction and axial convection -- MODEL DEPENDENT
    # --------------------------------------------------------------------------------------------------------------
    if wake_model_hov == 1:
        # Simple: Landgrebe k3/k4 contraction + single-rate axial convection
        Lambda = 0.145 + 27.0 * CT   # (ctrl_pts,)
        wcf    = 0.78 + (1.0 - 0.78) * np.exp(-Lambda[:, np.newaxis] * wakeage[np.newaxis, :])   # (ctrl_pts, N_wake+1)
    elif wake_model_hov in (2, 3):
        # Landgrebe (2) or Landgrebe - Kocurek & Tangler (3): empirical constants from total tip pitch
        theta_tip_deg = (np.degrees(rotor.twist_distribution[-1]))

        if wake_model_hov == 2:
            # Landgrebe - Source: DATTA's lecture notes
            k1 = 0.25 * (CT/sigma + 0.001 * theta_tip_deg)             # near-wake axial rate (ctrl_pts,)
            k2 = (1.41 + 0.0141 * theta_tip_deg) * np.sqrt(CT/2)       # far-wake axial rate (ctrl_pts,)
            k3 = 0.145 + 27.0 * CT                                     # contraction rate (ctrl_pts,)
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
                print(f"Warning: control point(s) {np.where(below_CT0)[0].tolist()} have "
                      f"CT={CT[below_CT0]} below the Kocurek-Tangler far-wake threshold "
                      f"CT0={CT0:.5f} (tip pitch {theta_tip_deg:.2f} deg) -- k2 is undefined "
                      f"below CT0. Clamping k2 to 0 for those points.")
            k2 = np.sqrt(np.maximum(CT - CT0, 0.0))   # (ctrl_pts,) -- clamped, avoids NaN below CT0
            k3 = 4.*(CT)**0.5                         # (ctrl_pts,)
            k4 = 0.78

        phi_break = 2.0 * np.pi / B

        # Piecewise axial convection + momentum theory base rate -- (ctrl_pts, N_wake+1)
        x_landgrebe = np.where(
            wakeage[np.newaxis, :] < phi_break,
            k1[:, np.newaxis] * wakeage[np.newaxis, :],
            k1[:, np.newaxis] * phi_break + k2[:, np.newaxis] * (wakeage[np.newaxis, :] - phi_break)
        ) + muxs[:, np.newaxis] * wakeage[np.newaxis, :] # (ctrl_pts, N_wake+1) non-dimensional x/R

        # Radial contraction
        wcf = k4 + (1.0 - k4) * np.exp(-k3[:, np.newaxis] * wakeage[np.newaxis, :])   # (ctrl_pts, N_wake+1)

    # --------------------------------------------------------------------------------------------------------------
    #  Shed point -- nearest node to r_R_shed*R (bound-node y/z already vary per control point
    #  via CW, see initialize_lifting_line.py)
    # --------------------------------------------------------------------------------------------------------------
    r_R_shed    = wake_inputs.get('r_R_shed', 1.0)
    R_shed_req  = r_R_shed * R
    nodes       = rotor.blades.bound.nodes_hub_14c   # (ctrl_pts, Nr, B, 3)
    i_shed      = np.argmin(np.abs(r_1d - R_shed_req))   # nearest node
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

    # --------------------------------------------------------------------------------------------------------------
    #  Build wake node positions in hub/thrust frame  (ctrl_pts, N_wake+1, B, 3)
    #
    #  Sign conventions:
    #  -- Negative sign on y_c: wake is shed opposite to CW rotation direction
    #     (the tip vortex trails behind the blade, unwinding counter to blade sweep).
    #  -- Rotation is in the y-z plane about the x/axial axis.
    #     Negative sign on y_rot enforces CW shedding direction.
    #  -- Axial convection in +x: lam > 0 means downwash in +x (thrust direction).
    #     muxs > 0 (climb) reduces net downwash: x_conv += R*(lam + muxs)*wa.
    #  -- In-plane convection in +z: forward flight velocity Wh in +z sweeps
    #     the wake in +z. Sign confirmed: positive Wh -> wake moves in +z.
    # --------------------------------------------------------------------------------------------------------------
    if wake_model_hov == 1:
        # Simple: rotate contracted tip position by wake age in y-z plane
        cwa = np.cos(wakeage)[np.newaxis, :, np.newaxis]   # (1, N+1, 1)
        swa = np.sin(wakeage)[np.newaxis, :, np.newaxis]

        y_c = -y_tip[:, np.newaxis, :] * wcf[:, :, np.newaxis]   # (ctrl_pts, N+1, B)
        z_c =  z_tip[:, np.newaxis, :] * wcf[:, :, np.newaxis]

        y_rot = np.where(CW_3, -(cwa*y_c - swa*z_c), -(cwa*y_c + swa*z_c))
        z_rot = np.where(CW_3,  (swa*y_c + cwa*z_c), -(swa*y_c - cwa*z_c))

        # axial convection -- broadcast (ctrl_pts,) with (N+1,) and (B,) -> (ctrl_pts, N+1, B)
        x_conv_hov = (x_tip[:, np.newaxis, :] +
                      R_shed * (lam[:, np.newaxis, np.newaxis] + muxs[:, np.newaxis, np.newaxis]) *
                      wakeage[np.newaxis, :, np.newaxis])
        y_conv_hov = y_rot
        z_conv_hov = z_rot

    elif wake_model_hov in (2, 3):
        # Landgrebe/KT: azimuthal position from psi_blade - wakeage
        psi_blade = rotor.blades.bound.psi[-1, :]   # (B,) tip azimuth

        psi_wake     = psi_blade[np.newaxis, :] - wakeage[:, np.newaxis]   # (N+1, B)
        r_contracted = R_shed * wcf                                          # (ctrl_pts, N+1)

        y_wake = np.where(
            CW_3,
            -r_contracted[:, :, np.newaxis] * np.sin(psi_wake)[np.newaxis, :, :],
             r_contracted[:, :, np.newaxis] * np.sin(psi_wake)[np.newaxis, :, :]
        )   # (ctrl_pts, N+1, B)
        z_wake = r_contracted[:, :, np.newaxis] * np.cos(psi_wake)[np.newaxis, :, :]   # CW-invariant

        x_conv_hov = (x_tip[:, np.newaxis, :] +
                      R_shed * x_landgrebe[:, :, np.newaxis])              # (ctrl_pts, N+1, B)
        y_conv_hov = y_wake
        z_conv_hov = z_wake

    # ==================================================================================================================
    #  FORWARD-FLIGHT MODELS (4, 5, 6) -- computed for every control point; only the points with
    #  is_edgewise == True actually use this result (selected in the combine step below).
    # ==================================================================================================================
    psi_blade = rotor.blades.bound.psi[-1, :]   # (B,) tip azimuth

    # --------------------------------------------------------------------------------------------------------------
    #  Wake distortion -- MODEL DEPENDENT
    # --------------------------------------------------------------------------------------------------------------
    # shapes: wakeage (N+1,), psi_blade (B,), muys/muzs/muxs/mu/lam (ctrl_pts,) -- broadcast
    # everything to (ctrl_pts, N_wake+1, B) up front so the case-selection below is vectorized.
    wa     = wakeage[np.newaxis, :, np.newaxis]        # (1, N+1, 1)
    psi    = psi_blade[np.newaxis, np.newaxis, :]      # (1, 1, B)
    muys_b = muys[:, np.newaxis, np.newaxis]           # (ctrl_pts, 1, 1)
    muzs_b = muzs[:, np.newaxis, np.newaxis]
    muxs_b = muxs[:, np.newaxis, np.newaxis]
    mu_b   = mu[:, np.newaxis, np.newaxis]
    lam_b  = lam[:, np.newaxis, np.newaxis]

    # Lat y / Long z -- same for models 4, 5, 6; only the axial (x) term differs.
    # This is the undistorted wake structure.
    #
    # y's rotational sign depends on CW/CCW, matching initialize_lifting_line.py's bound-node
    # convention (y=-r*sin(psi) for CW, y=+r*sin(psi) for CCW; z=r*cos(psi) is identical either
    # way, cosine being even). Without this branch y_conv only matches the blade layout for
    # CCW rotors (the sign that happened to be hardcoded here), and is mirrored relative to the
    # actual blade position for CW. The muys_b*wa drift term is a free-stream translation, not
    # a rotational effect, so it does NOT flip with CW -- only the sin(wa-psi) part does.
    x_conv_ff = R_shed * (lam_b + muxs_b) * wa
    y_conv_ff = np.where(CW_3,
                          R_shed * ( np.sin(wa - psi) + muys_b * wa),
                          R_shed * (-np.sin(wa - psi) + muys_b * wa))
    z_conv_ff = R_shed * (np.cos(wa - psi) + muzs_b * wa)

    if wake_model_FF == 5:
        # Beddoes (1985) distorted -- axial (x) position is picked from one of three
        # conditions (Case A/B/C) depending on where the vortex point sits relative to the
        # rotor disk, using a piecewise closed-form non-uniform (Glauert-type) inflow
        # distribution instead of the rigid axial rate used by model 4.
        x_conv_ff =  R_shed * (muxs_b) * wa
        Si = np.arctan2(mu_b,(lam_b + muxs_b))   # Wake Skew Angle
        E  = (Si)/2                              # / 2 as suggested by Leishman

        c       =  np.cos(psi - wa)          # = cos(psi_blade - wakeage), cosine is even
        y_star  =  y_conv_ff/ R_shed
        z_star  =  z_conv_ff/ R_shed
        abs_y3  =  np.abs(y_star**3)

        muzs_safe = np.where(np.abs(muzs_b) <= 1e-4, 1e-2, muzs_b)

        case_A = z_star < -c                # closest to the disk -- see below
        case_B = c > 0                      # given not case_A

        # Case A: "0.5*muzs_b*wa" (not mu_b) -- matches the muzs-based case boundary (z_star,
        # via muzs) so Case A is continuous with Case C at their shared boundary; using mu_b
        # here instead breaks that continuity (verified by boundary substitution).
        x_conv_A = x_conv_ff +   lam_b *                        (1 - (E)*abs_y3 + (E)*(c + 0.5*mu_b*wa)) * wa * R_shed # under the disk
        x_conv_B = x_conv_ff + 2*lam_b *                        (1 - (E)*abs_y3                        ) * wa * R_shed
        x_conv_C = x_conv_ff + 2*lam_b * (z_star / muzs_safe) * (1 - (E)*abs_y3                        )      * R_shed

        x_conv_ff = np.where(case_A, x_conv_A, np.where(case_B, x_conv_B, x_conv_C))

    elif wake_model_FF == 6:

        # Modified Beddoes by Van der Wall(2000)
        # adds this two terms: 8*(E)/15/np.pi - 2*muzs*y_star
        x_conv_ff =  R_shed * (muxs_b) * wa
        Si = np.arctan2(mu_b,(lam_b + muxs_b))   # Wake Skew Angle
        E  = (Si)/2                              # / 2 as suggested by Leishman

        c       =  np.cos(psi - wa)          # = cos(psi_blade - wakeage), cosine is even
        y_star  =  y_conv_ff/ R_shed
        z_star  =  z_conv_ff/ R_shed
        abs_y3  =  np.abs(y_star**3)

        muzs_safe = np.where(np.abs(muzs_b) <= 1e-4, 1e-2, muzs_b)

        case_A = z_star < -c                # closest to the disk -- see below
        case_B = c > 0                      # given not case_A

        # sign of the 2*muzs*y_star term flips with rotation sense -- muzs_b (broadcast to
        # (ctrl_pts,1,1)) used here, not raw muzs, so this is correctly shaped for ctrl_pts > 1
        sign_term = np.where(CW_3, 2*muzs_b*y_star, -2*muzs_b*y_star)

        x_conv_A = x_conv_ff +   lam_b *                        (1 + 8*(E)/15/np.pi + sign_term - (E)*abs_y3 + (E)*(c + 0.5*mu_b*wa)) * wa * R_shed # under the disk
        x_conv_B = x_conv_ff + 2*lam_b *                        (1 + 8*(E)/15/np.pi + sign_term - (E)*abs_y3                        ) * wa * R_shed
        x_conv_C = x_conv_ff + 2*lam_b * (z_star / muzs_safe) * (1 + 8*(E)/15/np.pi + sign_term - (E)*abs_y3                        )      * R_shed

        x_conv_ff = np.where(case_A, x_conv_A, np.where(case_B, x_conv_B, x_conv_C))

    # ==================================================================================================================
    #  Combine hover and FF results per control point
    # ==================================================================================================================
    x_conv = np.where(is_edgewise_3, x_conv_ff, x_conv_hov)
    y_conv = np.where(is_edgewise_3, y_conv_ff, y_conv_hov)
    z_conv = np.where(is_edgewise_3, z_conv_ff, z_conv_hov)

    # ------------------------------------------------------------------------------------------------------------------
    #  Store node positions (shared)
    # ------------------------------------------------------------------------------------------------------------------
    nodes_hub[:, :, :, 0] = x_conv
    nodes_hub[:, :, :, 1] = y_conv
    nodes_hub[:, :, :, 2] = z_conv

    # ------------------------------------------------------------------------------------------------------------------
    #  Transform to body frame
    # ------------------------------------------------------------------------------------------------------------------
    hub_origin = np.array(rotor.origin[0]).reshape(1, 1, 1, 3)
    nodes_body = np.einsum('cij,cwbj->cwbi', T_thrust2body, nodes_hub) + hub_origin

    # ------------------------------------------------------------------------------------------------------------------
    #  Lamb-Oseen core radius
    # ------------------------------------------------------------------------------------------------------------------
    wakeage_core = np.maximum((wakeage[:-1]) - (lamb_oseen_core_growth_delay), 0.0)

    rCvf = np.sqrt(
        (lamb_oseen_rc_0 * R)**2 +
        4 * lamb_oseen_alpha * lamb_oseen_delta * lamb_oseen_sigma *
        (nu[:, None] / np.abs(omega[:, 0, None])) * wakeage_core[None, :]
    )   # (ctrl_pts, N_wake)

    # ------------------------------------------------------------------------------------------------------------------
    #  Store
    # ------------------------------------------------------------------------------------------------------------------
    rotor.blades.wake = Data()

    rotor.blades.wake.nodes_hub   = nodes_hub   # (ctrl_pts, N_wake+1, B, 3)
    rotor.blades.wake.nodes_body  = nodes_body  # (ctrl_pts, N_wake+1, B, 3)
    rotor.blades.wake.wakeage     = wakeage     # (N_wake+1,)
    rotor.blades.wake.N_wake      = N_wake
    rotor.blades.wake.wcf         = wcf         # (ctrl_pts, N_wake+1) -- hover-model result; only meaningful where ~is_edgewise
    rotor.blades.wake.is_edgewise = is_edgewise # (ctrl_pts,) -- which regime each control point used
    rotor.blades.wake.muxs        = muxs        # (ctrl_pts,)
    rotor.blades.wake.muys        = muys        # (ctrl_pts,)
    rotor.blades.wake.muzs        = muzs        # (ctrl_pts,)
    rotor.blades.wake.lam         = lam         # (ctrl_pts,)
    rotor.blades.wake.rCvf        = rCvf        # (ctrl_pts, N_wake)
    rotor.blades.wake.gamma       = np.zeros((ctrl_pts, N_wake, B)) # filled after Gamma_b converges

    return