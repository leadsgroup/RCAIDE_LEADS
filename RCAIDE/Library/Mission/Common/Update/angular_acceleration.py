# RCAIDE/Library/Missions/Common/Update/angular_acceleration.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
import numpy as np
from RCAIDE.Framework.Core   import orientation_product, orientation_transpose 

# ----------------------------------------------------------------------------------------------------------------------
# Update Acceleration
# ----------------------------------------------------------------------------------------------------------------------   
def angular_acceleration(segment):
    """ Differentiates the angular velocity vector to get angular accelerations
    
        Assumptions:
        Assumes a flat earth, this is planar motion.
        
        Inputs:
            segment.state.conditions:
                frames.inertial.angular_velocity_vector     [rad/second]
            segment.state.numerics.time.differentiate       [float]
            
        Outputs:
            segment.state.conditions:           
                frames.inertial.angular_acceleration_vector [rad/s^2]

        Properties Used:
        N/A
                                
    """            
    
    # unpack conditions
    omega           = segment.state.conditions.frames.inertial.angular_velocity_vector
    D               = segment.state.numerics.time.differentiate 
    T_wind2inertial = segment.state.conditions.frames.wind.transform_to_inertial   
    T_inertia2wind  = orientation_transpose(T_wind2inertial)    
    
    # accelerations
    ang_acc_i = np.dot(D,omega)
    ang_acc_w = orientation_product(T_inertia2wind,ang_acc_i )

    # pack conditions
    segment.state.conditions.frames.inertial.angular_acceleration_vector[:,:] = ang_acc_i[:,:] 
    segment.state.conditions.frames.wind.angular_acceleration_vector[:,:]     = ang_acc_w[:,:]