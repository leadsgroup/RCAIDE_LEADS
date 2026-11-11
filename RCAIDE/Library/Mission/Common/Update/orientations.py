# RCAIDE/Library/Missions/Common/Update/orientations.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core  import Units,  angles_to_dcms, orientation_product, orientation_transpose

# package imports 
import numpy as np

from RCAIDE.Framework.Core import array_namespace
 
# ----------------------------------------------------------------------------------------------------------------------
#  Update Orientations
# ----------------------------------------------------------------------------------------------------------------------
def orientations(segment):
    
    """ Updates the orientation of the vehicle throughout the mission for each relevant axis
    
        Assumptions:
        This assumes the vehicle has 3 frames: inertial, body, and wind 
        
        Inputs:
        segment.state.conditions:
            frames.inertial.velocity_vector          [meters/second]
            frames.body.inertial_rotations           [Radians]
        segment.analyses.planet.mean_radius [meters]
        state.numerics.time.integrate                [float]
            
        Outputs:
            segment.state.conditions:           
                aerodynamics.angles.alpha         [Radians]
                aerodynamics.angles.beta          [Radians]
                aerodynamics.angles.roll          [Radians]
                frames.body.transform_to_inertial [Radians]
                frames.wind.body_rotations        [Radians]
                frames.wind.transform_to_inertial [Radians]
    

        Properties Used:
        N/A
    """

    # unpack
    conditions              = segment.state.conditions
    V_inertial              = conditions.frames.inertial.velocity_vector
    body_inertial_rotations = conditions.frames.body.inertial_rotations 
    roll_rate               = segment.state.conditions.static_stability.roll_rate      
    pitch_rate              = segment.state.conditions.static_stability.pitch_rate 
    yaw_rate                = segment.state.conditions.static_stability.yaw_rate    
    
    # ------------------------------------------------------------------
    #  Body Frame
    # ------------------------------------------------------------------

    # body frame rotations
    phi  = body_inertial_rotations[:,0,None] 
    beta = conditions.frames.wind.body_rotations[:,2]

    # body frame tranformation matrices
    T_inertial2body = angles_to_dcms(body_inertial_rotations,(2,1,0)) 
    T_body2inertial = orientation_transpose(T_inertial2body) 

    # transform inertial velocity to body frame
    V_body = orientation_product(T_inertial2body,V_inertial)

    # project inertial velocity into body x-z plane
    V_stability = V_body * 1.

    # calculate angle of attack
    xp    = array_namespace(V_inertial, body_inertial_rotations)
    alpha = xp.arctan2(V_stability[:,2],V_stability[:,0])[:,None] 

    # pack aerodynamics angles
    conditions.aerodynamics.angles.alpha = alpha
    conditions.aerodynamics.angles.beta  = beta[:,None] 
    conditions.aerodynamics.angles.phi   = phi 

    # pack transformation tensor
    conditions.frames.body.transform_to_inertial = T_body2inertial 

    # ------------------------------------------------------------------
    #  Wind Frame
    # ------------------------------------------------------------------

    # back calculate wind frame rotations
    # columns: roll (none in the wind frame), pitch (angle of attack), yaw (sideslip)
    wind_body_rotations = xp.stack([xp.zeros_like(beta), alpha[:,0], beta], axis=1)

    # wind frame tranformation matricies
    T_wind2body     = angles_to_dcms(wind_body_rotations,(2,1,0))       
    T_wind2inertial = orientation_product(T_wind2body,T_body2inertial) 

    # pack wind rotations
    conditions.frames.wind.body_rotations = wind_body_rotations

    # pack transformation tensor
    conditions.frames.wind.transform_to_inertial = T_wind2inertial
    conditions.frames.wind.transform_to_body     = T_wind2body
    
    # ------------------------------------------------------------------
    # Rotation rates 
    # ------------------------------------------------------------------ 
    stability_frame_rotations       =  xp.concatenate((roll_rate, pitch_rate, yaw_rate), axis=1)
    phi                             = body_inertial_rotations[:, 0]
    theta                           = body_inertial_rotations[:, 1]
    zero                            =  xp.zeros_like(phi)
    one                             =  xp.ones_like(phi)
    # body rates to Euler angle rates, one 3x3 matrix per control point, built row by row with stack rather than
    # by writing entries, so it works for JAX arrays too
    row_1                           =  xp.stack([one,  xp.sin(phi)*xp.tan(theta), xp.cos(phi)*xp.tan(theta)], axis=-1)
    row_2                           =  xp.stack([zero, xp.cos(phi),               -xp.sin(phi)             ], axis=-1)
    row_3                           =  xp.stack([zero, xp.sin(phi)/xp.cos(theta), xp.cos(phi)/xp.cos(theta)], axis=-1)
    reverse_transformation          =  xp.stack([row_1, row_2, row_3], axis=1)
    inertial_rotations              =  orientation_product(reverse_transformation,stability_frame_rotations)
    segment.state.conditions.frames.inertial.angular_velocity_vector = inertial_rotations 
    
    return
         