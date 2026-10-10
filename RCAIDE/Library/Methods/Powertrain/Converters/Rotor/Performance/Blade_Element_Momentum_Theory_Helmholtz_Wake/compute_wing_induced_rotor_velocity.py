# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Blade_Element_Momentum_Theory_Helmholtz_Wake/compute_wing_induced_rotor_velocity.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Framework.Core                                                             import Data, orientation_transpose
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.compute_wing_induced_velocity import compute_wing_induced_velocity, CASE_INDEXED_VD_FIELDS

# package imports
import numpy as np
import hashlib

# vehicle frame (x aft, z up) and body frame (x out the nose, z down) differ by a pi rotation about y
BODY_TO_VEHICLE = np.array([-1., 1., -1.])

# ----------------------------------------------------------------------------------------------------------------------
#  compute_wing_induced_rotor_velocity
# ----------------------------------------------------------------------------------------------------------------------
def compute_wing_induced_rotor_velocity(rotor, conditions, T_body2thrust, r_dim_2d, psi_2d):
    """
    Computes the velocity induced by the wing vortex lattice at the rotor disc stations, in the rotor polar frame.

    Parameters
    ----------
    rotor : RCAIDE.Library.Components.Powertrain.Converters.Rotor
        Rotor whose disc is evaluated
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with aerodynamics.VD and aerodynamics.gamma from the VLM
    T_body2thrust : numpy.ndarray
        Body to rotor velocity frame transformation, shape (ctrl_pts, 3, 3)
    r_dim_2d : numpy.ndarray
        Radial station positions, shape (ctrl_pts, Nr, Na) [m]
    psi_2d : numpy.ndarray
        Azimuthal station positions in the direction of rotation, shape (ctrl_pts, Nr, Na) [rad]

    Returns
    -------
    ua : numpy.ndarray
        Axial induced velocity, shape (ctrl_pts, Nr, Na) [m/s]
    ut : numpy.ndarray
        Tangential induced velocity in the direction of rotation, shape (ctrl_pts, Nr, Na) [m/s]
    ur : numpy.ndarray
        Radial induced velocity, positive toward the tip, shape (ctrl_pts, Nr, Na) [m/s]

    Notes
    -----
    The unit-strength influence of every wing panel on the disc is cached on the rotor and
    rebuilt only when the rotor, wing geometry or Mach number changes.
    """
    VD    = conditions.aerodynamics.VD
    mach  = conditions.freestream.mach_number
    key   = wing_influence_key(rotor, VD, mach, T_body2thrust, r_dim_2d, psi_2d)
    cache = rotor.wing_influence_cache
    if cache is None or cache.key != key:
        cache     = compute_wing_influence(rotor, VD, mach, T_body2thrust, r_dim_2d, psi_2d)
        cache.key = key
        rotor.wing_influence_cache = cache

    # dimensional vortex strengths, shape (ctrl_pts, n_panels)
    Gamma = conditions.aerodynamics.gamma * conditions.freestream.velocity

    ua = np.einsum('crap,cp->cra', cache.u_axial     , Gamma)
    ut = np.einsum('crap,cp->cra', cache.u_tangential, Gamma)
    ur = np.einsum('crap,cp->cra', cache.u_radial    , Gamma)
    return ua, ut, ur

def compute_wing_influence(rotor, VD, mach, T_body2thrust, r_dim_2d, psi_2d):
    """ Unit-strength velocity induced by each wing panel at the rotor disc stations, in the rotor polar frame """
    ctrl_pts, Nr, Na = np.shape(r_dim_2d)

    # disc stations in the rotor velocity frame; BEMT tangential direction is (cos(psi), sin(psi)) in y-z,
    # and a counter-clockwise rotor is its mirror image in y
    rotation = 1 if rotor.clockwise_rotation else -1
    y_disc =  rotation*r_dim_2d*np.sin(psi_2d)
    z_disc = -r_dim_2d*np.cos(psi_2d)
    disc_thrust = np.stack((np.zeros_like(y_disc), y_disc, z_disc), axis=-1)

    # rotor velocity frame -> body frame -> vehicle frame, then translate to the hub
    T_thrust2body  = orientation_transpose(T_body2thrust)
    disc_body      = np.einsum('cij,craj->crai', T_thrust2body, disc_thrust)
    disc_vehicle   = disc_body*BODY_TO_VEHICLE + np.array(rotor.origin[0])
    disc_vehicle   = disc_vehicle.reshape(ctrl_pts, Nr*Na, 3)

    # wing panels with the disc stations as evaluation points; surrogate geometry has a single case
    n_cases = len(VD.XC)
    VD_disc = Data()
    for field in CASE_INDEXED_VD_FIELDS:
        VD_disc[field] = VD[field] if n_cases == ctrl_pts else np.repeat(VD[field][0:1], ctrl_pts, axis=0)
    VD_disc.XC = disc_vehicle[:,:,0]
    VD_disc.YC = disc_vehicle[:,:,1]
    VD_disc.ZC = disc_vehicle[:,:,2]
    C_mn, _, _, _ = compute_wing_induced_velocity(VD_disc, mach, compute_EW=False)

    # vehicle frame -> body frame -> rotor velocity frame
    C_mn_body   = C_mn*BODY_TO_VEHICLE
    C_mn_thrust = np.einsum('cij,cnpj->cnpi', T_body2thrust, C_mn_body)
    C_mn_thrust = C_mn_thrust.reshape(ctrl_pts, Nr, Na, -1, 3)

    # BEMT uses the rotor velocity relative to the air, the negative of the induced air velocity
    V_rel   = -C_mn_thrust
    sin_psi = np.sin(psi_2d)[:,:,:,None]
    cos_psi = np.cos(psi_2d)[:,:,:,None]
    Vy      = rotation*V_rel[...,1]
    Vz      = V_rel[...,2]

    # project onto the rotor polar frame, as for the freestream in BEMT_Helmholtz_performance
    influence              = Data()
    influence.u_axial      = V_rel[...,0]
    influence.u_tangential = -Vz*sin_psi - Vy*cos_psi
    influence.u_radial     =  Vz*cos_psi - Vy*sin_psi
    return influence

def wing_influence_key(rotor, VD, mach, T_body2thrust, r_dim_2d, psi_2d):
    """ Hash of the inputs that define the wing influence on the rotor disc """
    h = hashlib.md5()
    h.update(np.asarray(rotor.origin, dtype=float).tobytes())
    h.update(np.asarray(rotor.clockwise_rotation, dtype=float).tobytes())
    h.update(np.asarray(T_body2thrust, dtype=float).tobytes())
    h.update(np.asarray(r_dim_2d, dtype=float).tobytes())
    h.update(np.asarray(psi_2d, dtype=float).tobytes())
    h.update(np.round(np.asarray(mach, dtype=float), 2).tobytes())
    for field in ['XAH','YAH','ZAH','XBH','YBH','ZBH']:
        h.update(np.asarray(VD[field], dtype=float).tobytes())
    return h.hexdigest()
