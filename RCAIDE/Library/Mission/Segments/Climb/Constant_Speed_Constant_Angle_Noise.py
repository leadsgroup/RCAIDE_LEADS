# RCAIDE/Library/Missions/Segments/Climb/Constant_Speed_Constant_Angle_Noise.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# Package imports  
import numpy as np

# ----------------------------------------------------------------------
#  Initialize Conditions
# ---------------------------------------------------------------------- 
def initialize_conditions(segment):
    """
    Expands state array for noise certification analysis

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function determines the minimum number of points needed for noise
    certification analysis and expands the state arrays accordingly.

    **Required Segment Components**

    segment:
        state:
            numerics:
                number_of_control_points : int
                    Number of discretization points

    **Major Assumptions**
    * Half-second time intervals required for certification
    * Fixed microphone position at 6500m
    * Continuous noise characteristics

    Returns
    -------
    None
        Updates segment state arrays directly
    """     
    
    dt=0.5  #time step in seconds for noise calculation
    
    # unpack
    climb_angle = segment.climb_angle
    air_speed   = segment.air_speed     
    beta        = segment.sideslip_angle
    t_nondim    = segment.state.numerics.dimensionless.control_points
    conditions  = segment.state.conditions  

    # check for initial velocity
    if air_speed is None: 
        if not segment.state.initials: raise AttributeError('airspeed not set')
        air_speed = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
        
    # process velocity vector
    v_mag = air_speed
    v_x   = np.cos(beta)*v_mag * np.cos(climb_angle)
    v_y   = np.sin(beta)*v_mag * np.cos(climb_angle)
    v_z   = -v_mag * np.sin(climb_angle)    

    #initial altitude
    alt0 = 10.668   #(35ft)
    altf = alt0 + (-v_z)*dt*len(t_nondim)

    # discretize on altitude
    alt = t_nondim * (altf-alt0) + alt0    
    
    # pack conditions    
    conditions.frames.inertial.velocity_vector[:,0] = v_x
    conditions.frames.inertial.velocity_vector[:,1] = v_y
    conditions.frames.inertial.velocity_vector[:,2] = v_z
    conditions.frames.inertial.position_vector[:,2] = -alt[:,0] # z points down
    conditions.freestream.altitude[:,0]             =  alt[:,0] # positive altitude in this context

def expand_state(segment):
    
    """Makes all vectors in the state the same size. Determines the minimum amount of points needed to get data for noise certification.

    Assumptions:
    Half second intervals for certification requirements. Fixed microphone position

    Source:
    N/A

    Inputs:
    state.numerics.number_of_control_points  [Unitless]

    Outputs:
    N/A

    Properties Used:
    Position of the flyover microphone is 6500 meters
    """          
    
    # unpack
    climb_angle  = segment.climb_angle
    air_speed    = segment.air_speed   
    conditions   = segment.state.conditions
    
    #Necessary input for determination of noise trajectory    
    dt = 0.5  #time step in seconds for noise calculation - Certification requirement    
    x0 = 6500 #Position of the Flyover microphone relatively to the break-release point
    
    # process velocity vector
    v_x=air_speed*np.cos(climb_angle)
    
    #number of time steps (space discretization)
    total_time=(x0+500)/v_x    
    n_points   = np.int(np.ceil(total_time/dt +1))       
    
    segment.state.numerics.number_of_control_points = n_points
    
    segment.state.expand_rows(n_points,override=True)      
    
    return
