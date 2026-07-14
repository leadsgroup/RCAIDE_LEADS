# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/initialize_lifting_line.py
#
# Created:  Jun 2026, H. Hussien
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from RCAIDE.Framework.Core import Data, orientation_transpose

# ----------------------------------------------------------------------------------------------------------------------
#  initialize_lifting_line
# ----------------------------------------------------------------------------------------------------------------------
def initialize_lifting_line(rotor, conditions):
    """
    Initializes the lifting-line geometry arrays for a rotor.

    Must be called once before the iteration loop. Fills 1/4-chord and 3/4-chord
    node positions (hub frame and body frame) and allocates zero arrays for
    aerodynamic quantities updated each iteration. All quantities are expanded
    to (Nr, B) or (ctrl_pts, Nr, B) and stored directly on rotor, so every
    blade and station can be combined into a single stacked array.

    Parameters
    ----------
    rotor : Data
        Rotor object. Reads:
            - number_of_blades
            - radius_distribution
            - sweep_distribution
            - chord_distribution
            - twist_distribution
            - origin
            - orientation_euler_angles
            - psi_0  (optional, default 0.0)
    conditions : Data
        Flight conditions. Reads:
            - frames.body.transform_to_inertial
            - energy.converters[rotor.tag].commanded_thrust_vector_angle
            - energy.converters[rotor.tag].blade_pitch_command

    Returns
    -------
    None
        Populates rotor.blades (a single Data() container, not a list) with
        the following stacked attributes:

        rotor.blades.bound.psi               : (Nr, B)             azimuth angle [rad]
        rotor.blades.bound.chord             : (Nr, B)             chord distribution [m]
        rotor.blades.bound.radius            : (Nr, B)             radial stations [m]
        rotor.blades.bound.beta              : (ctrl_pts, Nr, B)    total blade pitch [rad]
        rotor.blades.bound.nodes_hub_14c     : (Nr, B, 3)           1/4c nodes, hub/thrust frame
        rotor.blades.bound.nodes_body_14c    : (ctrl_pts, Nr, B, 3) 1/4c nodes, body frame
        rotor.blades.bound.nodes_hub_34c     : (ctrl_pts, Nr, B, 3) 3/4c nodes, hub/thrust frame (varies with pitch_c)
        rotor.blades.bound.nodes_body_34c    : (ctrl_pts, Nr, B, 3) 3/4c nodes, body frame
        rotor.blades.bound.cl                : (ctrl_pts, Nr, B)    lift coefficient
        rotor.blades.bound.cd                : (ctrl_pts, Nr, B)    drag coefficient
        rotor.blades.bound.alpha             : (ctrl_pts, Nr, B)    angle of attack [rad]
        rotor.blades.bound.gamma             : (ctrl_pts, Nr, B)    bound circulation [m^2/s]
        rotor.blades.bound.U                 : (ctrl_pts, Nr, B)    total local velocity [m/s]
        rotor.blades.bound.Ua                : (ctrl_pts, Nr, B)    axial local velocity [m/s]
        rotor.blades.bound.Ut                : (ctrl_pts, Nr, B)    tangential local velocity [m/s]
        rotor.blades.wake                    : Data()
            .nodes                           : None              filled when wake is implemented
            .gamma                           : None              filled when wake is implemented
    """
    # ------------------------------------------------------------------------------------------------------------------
    #  Unpack
    # ------------------------------------------------------------------------------------------------------------------
    B        = rotor.number_of_blades
    R        = rotor.tip_radius
    r_1d     = rotor.radius_distribution                   # (Nr,)
    sweep    = rotor.sweep_distribution                    # (Nr,)
    c        = rotor.chord_distribution                    # (Nr,)
    theta_tw = rotor.twist_distribution                    # (Nr,)
    Nr       = len(r_1d)
    rc       = rotor.rc

    if np.isscalar(sweep):
        sweep = np.zeros(Nr) if sweep == 0 else np.full(Nr, sweep)

    commanded_TV = conditions.energy.converters[rotor.tag].commanded_thrust_vector_angle
    pitch_c      = conditions.energy.converters[rotor.tag].blade_pitch_command
    ctrl_pts     = len(conditions.frames.inertial.velocity_vector)

    # Total blade pitch  -- (ctrl_pts, Nr) then expanded to (ctrl_pts, Nr, B)
    # theta_tw is (Nr,), pitch_c is (ctrl_pts, 1) or scalar
    beta_1d = theta_tw[np.newaxis, :] + np.atleast_2d(pitch_c)        # (ctrl_pts, Nr)
    beta    = np.repeat(beta_1d[:, :, np.newaxis], B, axis=2)       # (ctrl_pts, Nr, B)

    # Thrust-to-body rotation matrices  -- (ctrl_pts, 3, 3)
    T_body2inertial = conditions.frames.body.transform_to_inertial
    body2thrust, _  = rotor.body_to_prop_vel(commanded_TV)
    T_body2thrust   = orientation_transpose(
                          np.ones_like(T_body2inertial[:]) * body2thrust)
    T_thrust2body   = orientation_transpose(T_body2thrust)   # (ctrl_pts, 3, 3)

    # Hub origin in body frame  -- (1, 1, 1, 3)
    hub_origin = np.array(rotor.origin[0]).reshape(1, 1, 1, 3)

    # ------------------------------------------------------------------------------------------------------------------
    #  Blade azimuth locations
    # ------------------------------------------------------------------------------------------------------------------
    pi    = np.pi
    dpsi  = 2.0 * pi / B                                   # azimuthal spacing between blades
    # psi +ve in the rotation direction, starting from +z
    psi_0 = rotor.psi_0 if hasattr(rotor, 'psi_0') else 0.0

    # Root azimuth of each blade  -- (B,)
    psi_root = psi_0 + np.arange(B) * dpsi


    CW = rotor.clockwise_rotation

    # ------------------------------------------------------------------------------------------------------------------
    #  Azimuth per blade per station  -- (Nr, B)
    # ------------------------------------------------------------------------------------------------------------------
    psi = psi_root[np.newaxis, :] - sweep[:, np.newaxis]      # (Nr, B)

    # ------------------------------------------------------------------------------------------------------------------
    #  1/4c and 3/4c node positions, hub/thrust frame  -- (Nr, B, 3)
    # ------------------------------------------------------------------------------------------------------------------
    nodes_hub_14c = np.zeros((Nr, B, 3))

    r_2d = r_1d[:, np.newaxis] * np.ones((Nr, B))    # (Nr, B)
    c_2d = c[:, np.newaxis]    * np.ones((Nr, B))    # (Nr, B)

    if CW:
        # x: along rotor axis (nodes lie in rotor plane)
        # y: -r * sin(psi)
        # z:  r * cos(psi)
        nodes_hub_14c[:, :, 0] = 0.0
        nodes_hub_14c[:, :, 1] = -r_2d * np.sin(psi)
        nodes_hub_14c[:, :, 2] =  r_2d * np.cos(psi)
    else: # CCW
        nodes_hub_14c[:, :, 0] = 0.0
        nodes_hub_14c[:, :, 1] =  r_2d * np.sin(psi)
        nodes_hub_14c[:, :, 2] =  r_2d * np.cos(psi)

    # 3/4c node positions  -- offset from 1/4c by c/2 along local chord direction.
    # beta varies with ctrl_pts, so nodes_hub_34c also varies with ctrl_pts here;
    # shape becomes (ctrl_pts, Nr, B, 3) directly, skipping a separate hub-only 3/4c array.
    c_3d    = c_2d[np.newaxis, :, :]                                  # (1, Nr, B)
    psi_3d  =  psi[np.newaxis, :, :]                                  # (1, Nr, B)
    nodes_hub_14c_3d = nodes_hub_14c[np.newaxis, :, :, :]             # (1, Nr, B, 3)

    nodes_hub_34c_at_nodes = np.zeros((ctrl_pts, Nr, B, 3))
    nodes_hub_34c_at_nodes[:, :, :, 0] = c_3d/2 * np.sin(beta)
    if CW:
        nodes_hub_34c_at_nodes[:, :, :, 1] = nodes_hub_14c_3d[:, :, :, 1] + (c_3d/2 * np.cos(beta)) * np.cos(psi_3d)
        nodes_hub_34c_at_nodes[:, :, :, 2] = nodes_hub_14c_3d[:, :, :, 2] + (c_3d/2 * np.cos(beta)) * np.sin(psi_3d)
    else: # CCW
        nodes_hub_34c_at_nodes[:, :, :, 1] = nodes_hub_14c_3d[:, :, :, 1] - (c_3d/2 * np.cos(beta)) * np.cos(psi_3d)
        nodes_hub_34c_at_nodes[:, :, :, 2] = nodes_hub_14c_3d[:, :, :, 2] + (c_3d/2 * np.cos(beta)) * np.sin(psi_3d)

    # collocation points are at segment midpoint -- (ctrl_pts, Nr-1, B, 3)
    nodes_hub_34c = 0.5 * (nodes_hub_34c_at_nodes[:, :-1, :, :] + nodes_hub_34c_at_nodes[:,  1:, :, :])

    # ------------------------------------------------------------------------------------------------------------------
    #  Transform to body frame  -- (ctrl_pts, Nr, B, 3)
    # ------------------------------------------------------------------------------------------------------------------
    nodes_body_14c = np.einsum('cij, rbj -> crbi', T_thrust2body, nodes_hub_14c) \
                   + hub_origin
    nodes_body_34c = np.einsum('cij, crbj -> crbi', T_thrust2body, nodes_hub_34c) \
                   + hub_origin

    # ------------------------------------------------------------------------------------------------------------------
    #  Core radius definition
    # ------------------------------------------------------------------------------------------------------------------
    rCb = rc * R * np.ones((ctrl_pts, Nr-1, B))   # (ctrl_pts, Nr-1, B)

    # ------------------------------------------------------------------------------------------------------------------
    #  Store on rotor.blades.bound
    # ------------------------------------------------------------------------------------------------------------------
    rotor.blades       = Data()
    rotor.blades.bound = Data()
    rotor.blades.wake  = Data()

    rotor.blades.bound.psi             = psi             # (Nr, B)
    rotor.blades.bound.chord           = c_2d            # (Nr, B)
    rotor.blades.bound.radius          = r_2d            # (Nr, B)
    rotor.blades.bound.beta            = beta            # (ctrl_pts, Nr, B)
    rotor.blades.bound.nodes_hub_14c   = nodes_hub_14c   # (Nr, B, 3)
    rotor.blades.bound.nodes_body_14c  = nodes_body_14c  # (ctrl_pts, Nr, B, 3)
    rotor.blades.bound.nodes_hub_34c   = nodes_hub_34c   # (ctrl_pts, Nr-1, B, 3)
    rotor.blades.bound.nodes_body_34c  = nodes_body_34c  # (ctrl_pts, Nr-1, B, 3)
    rotor.blades.bound.rCb             = rCb             # (ctrl_pts, Nr-1, B)

    rotor.blades.wake.nodes            = None            # filled when wake is implemented
    rotor.blades.wake.gamma            = None            # filled when wake is implemented

    return