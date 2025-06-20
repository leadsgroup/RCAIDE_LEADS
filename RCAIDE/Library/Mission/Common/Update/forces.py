# RCAIDE/Library/Missions/Common/Update/forces.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import  RCAIDE 
from RCAIDE.Framework.Core   import orientation_product, orientation_transpose  
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
    conditions      = segment.state.conditions 
    F_aero_w        = conditions.frames.wind.force_vector
    F_thrust_body   = conditions.frames.body.thrust_force_vector
    F_weight_i      = conditions.frames.inertial.gravity_force_vector

    # unpack transformation matrices
    T_body2inertial = conditions.frames.body.transform_to_inertial
    T_wind2inertial = conditions.frames.wind.transform_to_inertial   
    T_wind2body     = conditions.frames.wind.transform_to_body     
    T_inertia2wind  = orientation_transpose(T_wind2inertial)
    T_inertial2body = orientation_transpose(T_body2inertial)
    T_body2wind     = orientation_transpose(T_wind2body) 

    # transform matrices 
    F_aero_i      = orientation_product(T_wind2inertial,F_aero_w) 
    F_thrust_i    = orientation_product(T_body2inertial,F_thrust_body)  
    F_thrust_wind = orientation_product(T_body2wind,F_thrust_body)
    F_weight_body = orientation_product(T_inertial2body, F_weight_i)
    F_weight_wind = orientation_product(T_body2wind,F_weight_body)
    
    if type(segment) ==  RCAIDE.Framework.Mission.Segments.Vertical_Flight.Climb:
        F_aero_i =  np.zeros_like(F_thrust_i)
    elif type(segment) ==  RCAIDE.Framework.Mission.Segments.Vertical_Flight.Hover:
        F_aero_i =  np.zeros_like(F_thrust_i)
    elif type(segment) ==  RCAIDE.Framework.Mission.Segments.Vertical_Flight.Descent:
        F_aero_i =  np.zeros_like(F_thrust_i)  
    
    F_weight_i[:,1] *= -1        
    F_weight_wind[:,1] *= -1
        
    # sum of the forces
    F_tot_i = F_aero_i +  F_thrust_i  + F_weight_i
    F_tot_w = F_aero_w +  F_thrust_wind  + F_weight_wind

    # pack
    conditions.frames.inertial.total_force_vector[:,:] = F_tot_i[:,:]
    conditions.frames.wind.total_force_vector[:,:]     = F_tot_w[:,:]

    return
 