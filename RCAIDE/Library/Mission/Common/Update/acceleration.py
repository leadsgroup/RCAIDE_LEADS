# RCAIDE/Library/Missions/Common/Update/acceleration.py
# 
# 
# Created:  Jul 2023, M. Clarke 
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
import numpy as np

from RCAIDE.Framework.Core import array_namespace

# ----------------------------------------------------------------------------------------------------------------------
# Update Acceleration
# ----------------------------------------------------------------------------------------------------------------------   
def acceleration(segment):
    """ Differentiates the velocity vector to get accelerations
    
        Assumptions:
        Assumes a flat earth, this is planar motion.
        
        Inputs:
            segment.state.conditions:
                frames.inertial.velocity_vector     [meters/second]
            segment.state.numerics.time.differentiate       [float]
            
        Outputs:
            segment.state.conditions:           
                frames.inertial.acceleration_vector [meters]

        Properties Used:
        N/A
                                
    """            
    
    # unpack conditions
    v = segment.state.conditions.frames.inertial.velocity_vector
    D = segment.state.numerics.time.differentiate
    
    # accelerations
    xp  = array_namespace(v)   # numpy, or jax.numpy when v is a JAX array
    acc = xp.dot(D,v)
    
    # pack conditions
    segment.state.conditions.frames.inertial.acceleration_vector = acc   