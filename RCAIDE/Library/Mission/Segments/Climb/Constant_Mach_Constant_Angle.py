# RCAIDE/Library/Missions/Segments/Climb/Constant_Mach_Constant_Angle.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------  
#  IMPORT 
# ----------------------------------------------------------------------------------------------------------------------  
# import RCAIDE 
from RCAIDE.Library.Mission.Common.Update.atmosphere import atmosphere

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------  
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------  
def initialize_conditions(segment):
    """Sets the specified conditions which are given for the segment type.
    
    Assumptions:
    Constant Mach number, with a constant angle of climb

    Source:
    N/A

    Inputs:
    segment.climb_angle                                 [radians]
    segment.mach_number                                 [Unitless]
    segment.altitude_start                              [meters]
    segment.altitude_end                                [meters]
    segment.state.numerics.dimensionless.control_points [Unitless]
    conditions.freestream.density                       [kilograms/meter^3]

    Outputs:
    conditions.frames.inertial.velocity_vector  [meters/second]
    conditions.frames.inertial.position_vector  [meters]
    conditions.freestream.altitude              [meters]

    Properties Used:
    N/A
    """       
    # unpack User Inputs
    climb_angle = segment.climb_angle
    mach_number = segment.mach_number
    alt0        = segment.altitude_start 
    beta        = segment.sideslip_angle 
    conditions  = segment.state.conditions  
        
    # unpack unknowns  
    alts     = conditions.frames.inertial.position_vector[:,2]
    
    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
    
    # pack conditions   
    conditions.freestream.altitude[:,0]   = -alts 

    # check for initial velocity
    if mach_number is None: 
        if not segment.state.initials: raise AttributeError('mach not set')
        v_mag  = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])*segment.state.ones_row(1)   
    else: 
        # Update freestream to get speed of sound
        atmosphere(segment)
        a = conditions.freestream.speed_of_sound    
        
        # process velocity vector
        v_mag = mach_number * a
    v_xy  = v_mag * np.cos(climb_angle)
    v_z   = -v_mag * np.sin(climb_angle)
    v_x   = np.cos(beta)*v_xy
    v_y   = np.sin(beta)*v_xy
    
    # pack conditions    
    conditions.frames.inertial.velocity_vector[:,0]              = v_x[:,0]
    conditions.frames.inertial.velocity_vector[:,1]              = v_y[:,0]
    conditions.frames.inertial.velocity_vector[:,2]              = v_z[:,0]   
    
# ----------------------------------------------------------------------------------------------------------------------  
#  Residual Total Forces
# ----------------------------------------------------------------------------------------------------------------------  
def residual_total_forces(segment):
    
    # Unpack results
    FT      = segment.state.conditions.frames.inertial.total_force_vector
    a       = segment.state.conditions.frames.inertial.acceleration_vector
    m       = segment.state.conditions.weights.total_mass    
    alt_in  = segment.state.unknowns.altitude[:,0] 
    alt_out = segment.state.conditions.freestream.altitude[:,0] 
    
    # Residual in X and Z, as well as a residual on the guess altitude
    if segment.flight_dynamics.force_x: 
        segment.state.residuals.force_x[:,0] = FT[:,0]/m[:,0] - a[:,0]
    if segment.flight_dynamics.force_y: 
        segment.state.residuals.force_y[:,0] = FT[:,1]/m[:,0] - a[:,1]       
    if segment.flight_dynamics.force_z: 
        segment.state.residuals.force_z[:,0] = FT[:,2]/m[:,0] - a[:,2]    
          
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
    conditions.frames.inertial.time[1:,0]           = t_initial + x[1:,0]
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0]  
    conditions.freestream.altitude[:,0]             =  alt[:,0]  

    return