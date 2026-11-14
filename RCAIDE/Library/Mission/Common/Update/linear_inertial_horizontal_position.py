# RCAIDE/Library/Missions/Common/Update/linear_inertial_horizontal_position.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import array_namespace, set_at, gradient

# Package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Integrate Position
# ----------------------------------------------------------------------------------------------------------------------
def linear_inertial_horizontal_position(segment):
    """ Determines how far the airplane has traveled. 
    
        Assumptions:
        Assumes a flat earth, this is planar motion.
        
        Inputs:
            segment.state.conditions:
                frames.inertial.position_vector [meters]
                frames.inertial.velocity_vector [meters/second]
            segment.state.numerics.time.integrate       [float]
            
        Outputs:
            segment.state.conditions:           
                frames.inertial.position_vector [meters]

        Properties Used:
        N/A
                                
    """        

    conditions = segment.state.conditions
    inertial   = conditions.frames.inertial
    psi        = segment.true_course       # sign convetion is clockwise positive
    I          = segment.state.numerics.time.integrate
    xp         = array_namespace(I, inertial.velocity_vector, inertial.position_vector, inertial.aircraft_range, inertial.time, psi)
    x0         = inertial.position_vector[0,None,0:1+1]
    R0         = inertial.aircraft_range[0,0]
    vx         = inertial.velocity_vector[:,0:1+1]
    trajectory = xp.stack([xp.cos(psi), xp.sin(psi)])[None,:]

    # integrate to compute position credit: the distance along the course
    distance   = xp.dot(I,vx)[:,0]
    inertial.position_vector = set_at(inertial.position_vector, (slice(None), slice(0,2)), x0 + distance[:,None]*trajectory)

    # do not apply apply range credit for loiter  
    if type(segment) ==  RCAIDE.Framework.Mission.Segments.Cruise.Constant_Dynamic_Pressure_Constant_Altitude_Loiter or  \
       type(segment) ==  RCAIDE.Framework.Mission.Segments.Cruise.Constant_Mach_Constant_Altitude_Loiter or \
       type(segment) ==  RCAIDE.Framework.Mission.Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter:
           
        inertial.aircraft_range = set_at(inertial.aircraft_range, (slice(None), 0), R0 + 0.*distance)
    else: 
        inertial.aircraft_range = set_at(inertial.aircraft_range, (slice(None), 0), R0 + distance)
    
    # compute climb rate 
    inertial.climb_rate = set_at(inertial.climb_rate, (slice(None), 0), gradient(-inertial.position_vector[:,2], inertial.time[:,0]))
    
    return