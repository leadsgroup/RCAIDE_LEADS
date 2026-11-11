# RCAIDE/Library/Missions/Segments/Common/Initialize/inertial_position.py
# 
# 
# Created:  Jul 2023, M. Clarke 
# Modified: Nov 2026, M. Clarke

import numpy as np

from RCAIDE.Framework.Core import set_at

# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Inertial Position
# ----------------------------------------------------------------------------------------------------------------------
def inertial_position(segment): 
    """ Initializes intertial positon of vehicle
    
        Assumptions:  
            Only used if there is an initial condition
            
        Inputs: 
            state.conditions:           
                numerics.dimensionless.integrate        [i]
                numerics.dimensionless.control_points   [-]
                frames.inertial.aircraft_range          [m]
            
        Outputs: 
            state.conditions.frames.inertial.position_vector  [m]
            state.conditions.frames.inertial.aircraft_range   [m]
           
        Properties Used:
        N/A 
    """      
    if segment.state.initials:
        r_initial = segment.state.initials.conditions.frames.inertial.position_vector
        r_current = segment.state.conditions.frames.inertial.position_vector
        R_initial = segment.state.initials.conditions.frames.inertial.aircraft_range
        R_current = segment.state.conditions.frames.inertial.aircraft_range
        
        # start from the last point of the previous segment, at this segment's altitude; r_start is a new array,
        # so the previous segment's results are not changed
        r_start = r_initial[-1,None,:] * 1.
        if 'altitude' in segment.keys() and segment.altitude is not None:
            r_start = set_at(r_start, np.s_[0,2], -segment.altitude)
        elif 'altitude_start' in segment.keys() and segment.altitude_start is not None:
            r_start = set_at(r_start, np.s_[0,2], -segment.altitude_start)
        else:
            assert('Altitude not set')
            
        segment.state.conditions.frames.inertial.position_vector = r_current + (r_start - r_current[0,None,:])
        segment.state.conditions.frames.inertial.aircraft_range  = R_current + (R_initial[-1,None,:] - R_current[0,None,:])
        
    return 