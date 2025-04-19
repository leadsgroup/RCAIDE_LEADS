# RCAIDE/Library/Missions/Segments/Climb/Constant_Dynamic_Pressure_Constant_Angle.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE 
import RCAIDE 

# Package imports  
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------
def initialize_conditions_unpack_unknowns(segment):
    """Sets the specified conditions which are given for the segment type.

    Assumptions:
    Constrant dynamic pressure and constant rate of climb

    Source:
    N/A

    Inputs:
    segment.climb_angle                                 [radians]
    segment.dynamic_pressure                            [pascals]
    segment.altitude_start                              [meters]
    segment.altitude_end                                [meters]
    segment.state.numerics.dimensionless.control_points [unitless]
    conditions.freestream.density                       [kilograms/meter^3]  

    Outputs:
    conditions.frames.inertial.velocity_vector  [meters/second]
    conditions.frames.inertial.position_vector  [meters]
    conditions.energy.throttle              [unitless]
    conditions.frames.body.inertial_rotations   [radians]

    Properties Used:
    N/A
    """           
    
    # unpack
    climb_angle = segment.climb_angle
    q           = segment.dynamic_pressure
    alt0        = segment.altitude_start  
    conditions  = segment.state.conditions
    beta        = segment.sideslip_angle
    rho         = conditions.freestream.density[:,0]  
    
    # unpack unknowns  
    alts     = conditions.frames.inertial.position_vector[:,2]

    # Update freestream to get density
    RCAIDE.Library.Mission.Common.Update.atmosphere(segment)
    rho = conditions.freestream.density[:,0]   

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
    
    # pack conditions    
    conditions.freestream.altitude[:,0] =  -alts  # positive altitude in this context    
    

    # check for initial velocity
    if q is None: 
        if not segment.state.initials: raise AttributeError('dynamic pressure not set')
        v_mag = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
    else: 
        # Update freestream to get density
        RCAIDE.Library.Mission.Common.Update.atmosphere(segment)
        rho = conditions.freestream.density[:,0]       
    
        # process velocity vector
        v_mag = np.sqrt(2*q/rho)
        
    v_x   = np.cos(beta)*v_mag * np.cos(climb_angle)
    v_y   = np.sin(beta)*v_mag * np.cos(climb_angle)
    v_z   = -v_mag * np.sin(climb_angle)
    
    # pack conditions    
    conditions.frames.inertial.velocity_vector[:,0] = v_x
    conditions.frames.inertial.velocity_vector[:,1] = v_y
    conditions.frames.inertial.velocity_vector[:,2] = v_z   
    
def residual_altitude(segment):
    """Computes the altitude residual

    Assumptions:
    No higher order terms.

    Source:
    N/A

    Inputs:
    segment.state.conditions.frames.inertial.total_force_vector   [Newtons]
    segment.state.conditions.frames.inertial.acceleration_vector  [meter/second^2]
    segment.state.conditions.weights.total_mass                   [kilogram]
    segment.state.conditions.freestream.altitude                  [meter]

    Outputs:
    segment.state.residuals.altitude                              [meters] 

    Properties Used:
    N/A
    """     
    
    # Unpack results 
    alt_in  = segment.state.unknowns.altitude[:,0] 
    alt_out = segment.state.conditions.freestream.altitude[:,0]  
    segment.state.residuals.altitude[:,0] = (alt_in - alt_out)/alt_out[-1]

    return


# ----------------------------------------------------------------------------------------------------------------------  
# Update Differentials
# ----------------------------------------------------------------------------------------------------------------------     
def update_differentials(segment):
    """ On each iteration creates the differentials and integration functions from knowns about the problem. 
      Sets the time at each point. Must return in dimensional time, with t[0] = 0.
      This is different from the common method as it also includes the scaling of operators.

        Assumptions:
        Works with a segment discretized in vertical position, altitude

        Inputs:
        state.numerics.dimensionless.control_points      [Unitless]
        state.numerics.dimensionless.differentiate       [Unitless]
        state.numerics.dimensionless.integrate           [Unitless]
        state.conditions.frames.inertial.position_vector [meter]
        state.conditions.frames.inertial.velocity_vector [meter/second]
        

        Outputs:
        state.conditions.frames.inertial.time            [second]

    """    

    # unpack
    numerics   = segment.state.numerics
    conditions = segment.state.conditions
    x          = numerics.dimensionless.control_points
    D          = numerics.dimensionless.differentiate
    I          = numerics.dimensionless.integrate 
    r          = segment.state.conditions.frames.inertial.position_vector
    v          = segment.state.conditions.frames.inertial.velocity_vector
    alt0       = segment.altitude_start
    altf       = segment.altitude_end    

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
        
    dz = altf - alt0
    vz = -v[:,2,None] # maintain column array

    # get overall time step
    dt = (dz/np.dot(I,vz))[-1]

    # rescale operators
    x = x * dt
    D = D / dt
    I = I * dt
    
    # Calculate the altitudes
    alt = np.dot(I,vz) + alt0
    
    # pack
    t_initial                                       = segment.state.conditions.frames.inertial.time[0,0]
    numerics.time.control_points                    = x
    numerics.time.differentiate                     = D
    numerics.time.integrate                         = I
    conditions.frames.inertial.time[1:,0]            = t_initial + x[1:,0]
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0]  
    conditions.freestream.altitude[:,0]             =  alt[:,0]  

    return