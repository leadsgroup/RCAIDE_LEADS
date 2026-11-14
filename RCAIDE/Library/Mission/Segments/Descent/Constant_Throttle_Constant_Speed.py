# RCAIDE/Library/Missions/Segments/Descent/Constant_Throttle_Constant_Speed.py
# 
# 
# Created:  Mar 2026, M. Clarke
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# Package imports  
from RCAIDE.Library.Mission.Common.Unpack_Unknowns.orientation import sideslip_angle
from RCAIDE.Framework.Core import array_namespace, set_at
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------
def unpack_pitch_angle(segment):
    """
    Unpacks and sets the proper value for body angle

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function handles the initialization of the body angle for a descent segment
    with constant throttle and constant speed.

    **Required Segment Components**

    segment:
        state:
            unknowns:
                pitch_angle : array
                    Aircraft body angle [rad]
            conditions:
                frames:
                    body:
                        transform_to_inertial : array
                            Rotation matrix from body to inertial frame

    Returns
    -------
    None
        Updates segment conditions directly:
        - conditions.frames.body.transform_to_inertial
    """          
    
    # set_at(A, np.s_[:,1], value) does A[:,1] = value and returns A. NumPy writes in place; JAX arrays cannot be
    # changed, so for JAX it returns a new array, which is why the result is always assigned back.
    ctrls     = segment.assigned_control_variables 
    rotations = segment.state.conditions.frames.body.inertial_rotations   # columns: roll, pitch, heading

    # Body Angle Control    
    if ctrls.pitch_angle.active: 
        rotations = set_at(rotations, np.s_[:,1], segment.state.unknowns.mission.pitch_angle[:,0])
    else:
        rotations = set_at(rotations, np.s_[:,1], segment.angle_of_attack)

    # Bank Angle 
    if ctrls.bank_angle.active: 
        rotations = set_at(rotations, np.s_[:,0], segment.state.unknowns.mission.bank_angle[:,0])
    else:
        rotations = set_at(rotations, np.s_[:,0], segment.bank_angle)

    # Sideslip Angle  
    wind = segment.state.conditions.frames.wind
    wind.body_rotations = set_at(wind.body_rotations, np.s_[:,2], sideslip_angle(segment))

    # Heading Angle  
    rotations = set_at(rotations, np.s_[:,2], segment.state.conditions.frames.planet.true_heading[:,0])
    segment.state.conditions.frames.body.inertial_rotations = rotations
         
# ----------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------

def initialize_conditions(segment):
    """Sets the specified conditions which are given for the segment type.
    
    Assumptions:
    Constant throttle estting, with a constant true airspeed

    Source:
    N/A

    Inputs:
    segment.air_speed                                   [meters/second] 
    segment.altitude_start                              [meters]
    segment.altitude_end                                [meters]
    segment.state.numerics.dimensionless.control_points [Unitless]
    conditions.freestream.density                       [kilograms/meter^3]

    Outputs:
    conditions.frames.inertial.velocity_vector          [meters/second]

    Properties Used:
    N/A
    """         
    
    # unpack 
    alt0       = segment.altitude_start 
    v_mag      = segment.air_speed 
    beta       = segment.sideslip_angle
    alpha      = segment.state.unknowns.mission.angle_of_attack[:,0][:,None]
    theta      = segment.state.unknowns.mission.pitch_angle[:,0][:,None]
    conditions = segment.state.conditions  

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
   
    # Flight path angle
    gamma = theta-alpha

    # process
    v_x =   np.cos(beta) *v_mag * np.cos(gamma)
    v_y =   np.sin(beta) *v_mag * np.cos(gamma)
    v_z = -v_mag * np.sin(gamma) # z points down

    # pack
    conditions.frames.inertial.velocity_vector[:,0] = v_x[:,0]
    conditions.frames.inertial.velocity_vector[:,1] = v_y[:,0]
    conditions.frames.inertial.velocity_vector[:,2] = v_z[:,0]
 

def update_differentials_altitude(segment):
    """On each iteration creates the differentials and integration funcitons from knowns about the problem. Sets the time at each point. Must return in dimensional time, with t[0] = 0
    
    Assumptions:
    Constant throttle setting, with a constant true airspeed.

    Source:
    N/A

    Inputs:
    segment.descent_angle                         [radians]
    state.conditions.frames.inertial.velocity_vector [meter/second]
    segment.altitude_start                      [meters]
    segment.altitude_end                        [meters]

    Outputs:
    state.conditions.frames.inertial.time       [seconds]
    conditions.frames.inertial.position_vector  [meters]
    conditions.freestream.altitude              [meters]

    Properties Used:
    N/A
    """   

    # unpack
    t = segment.state.numerics.dimensionless.control_points
    I = segment.state.numerics.dimensionless.integrate
    
    # Unpack segment initials
    alt0       = segment.altitude_start 
    altf       = segment.altitude_end    
    conditions = segment.state.conditions  
    v          = segment.state.conditions.frames.inertial.velocity_vector
    
    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]    
    
    # get overall time step
    vz = -v[:,2,None] # Inertial velocity is z down
    dz = altf - alt0 # negative in a descent, as is vz, so the segment time is positive
    xp = array_namespace(v)   # numpy, or jax.numpy when v is a JAX array
    dt = dz / xp.dot(I[-1,:],vz)[-1] # maintain column array
    
    # Integrate vz to get altitudes
    alt = alt0 + xp.dot(I*dt,vz)

    # rescale operators
    t = t * dt

    # pack
    t_initial = segment.state.conditions.frames.inertial.time[0,0]
    segment.state.conditions.frames.inertial.time      = t_initial + t
    conditions.frames.inertial.position_vector         = set_at(conditions.frames.inertial.position_vector, np.s_[:,2], -alt[:,0]) # z points down
    conditions.freestream.altitude                     =  alt # positive altitude in this context    

    return

# ----------------------------------------------------------------------
#  Update Velocity Vector from Wind Angle
# ----------------------------------------------------------------------

def update_velocity_vector_from_wind_angle(segment):
    
    # unpack
    conditions = segment.state.conditions 
    v_mag      = segment.air_speed 
    beta       = segment.sideslip_angle
    alpha      = segment.state.unknowns.mission.angle_of_attack[:,0][:,None]
    theta      = segment.state.unknowns.mission.pitch_angle[:,0][:,None]
    
    # Flight path angle
    gamma = theta-alpha

    # process
    xp  = array_namespace(alpha, theta, beta)   # numpy, or jax.numpy when the unknowns are JAX arrays
    v_x =   xp.cos(beta) *v_mag * xp.cos(gamma)
    v_y =   xp.sin(beta) *v_mag * xp.cos(gamma)
    v_z = -v_mag * xp.sin(gamma) # z points down

    # pack
    conditions.frames.inertial.velocity_vector = xp.concatenate([v_x, v_y, v_z], axis=1)

    return conditions
