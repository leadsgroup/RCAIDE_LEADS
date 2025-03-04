# RCAIDE/Library/Missions/Common/Update/forces.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import  RCAIDE
from RCAIDE.Framework.Core  import  orientation_product
import  numpy as  np 

# ----------------------------------------------------------------------------------------------------------------------
#  Update Forces
# ----------------------------------------------------------------------------------------------------------------------
def forces(segment): 
    """ Updates the total resultant force on the vehicle 
        
        Assumptions:
        N/A
        
        Inputs:
            segment.state.conditions.:
                frames.wind.force_vector               [N]
                frames.body.thrust_force_vector        [N]
                frames.inertial.gravity_force_vector   [N]
        Outputs:
            segment.conditions
                frames.inertial.total_force_vector     [N]

      
        Properties Used:
        N/A
                    
    """    
 
    # unpack 
    conditions                    = segment.state.conditions 
    wind_force_vector             = conditions.frames.wind.force_vector
    body_thrust_force_vector      = conditions.frames.body.thrust_force_vector
    inertial_gravity_force_vector = conditions.frames.inertial.gravity_force_vector

    # unpack transformation matrices
    T_body2inertial = conditions.frames.body.transform_to_inertial
    T_wind2inertial = conditions.frames.wind.transform_to_inertial

    # to inertial frame
    F = orientation_product(T_wind2inertial,wind_force_vector) 
    T = orientation_product(T_body2inertial,body_thrust_force_vector)
    if type(segment) ==  RCAIDE.Framework.Mission.Segments.Vertical_Flight.Climb:
        F =  np.zeros_like(T)
    elif type(segment) ==  RCAIDE.Framework.Mission.Segments.Vertical_Flight.Hover:
        F =  np.zeros_like(T)
    elif type(segment) ==  RCAIDE.Framework.Mission.Segments.Vertical_Flight.Descent:
        F =  np.zeros_like(T) 
    W = inertial_gravity_force_vector

    # sum of the forces
    F_tot = F +  T  + W 

    # pack
    conditions.frames.inertial.total_force_vector[:,:] = F_tot[:,:]

    return
 