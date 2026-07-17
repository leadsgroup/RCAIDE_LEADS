# RCAIDE/Library/Missions/Common/Update/moments.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
from RCAIDE.Framework.Core   import orientation_product, orientation_transpose  

# ----------------------------------------------------------------------------------------------------------------------
#  Update Moments
# ----------------------------------------------------------------------------------------------------------------------
def moments(segment): 
    """ Updates the total resultant moment on the vehicle 
        
        Assumptions:
        N/A
        
        Inputs:
            segment.state.conditions.:
                frames.wind.force_vector          [N] 
                frames.body.thrust_force_vector        [N]
                frames.inertial.gravity_force_vector   [N]
        Outputs:
            segment.conditions
                frames.inertial.total_force_vector     [N]
 
      
        Properties Used:
        N/A
                    
    """    

    # unpack
    conditions    = segment.state.conditions  
    M_aero_w      = conditions.frames.wind.moment_vector
    M_thrust_b    = conditions.frames.body.thrust_moment_vector
    
    # unpack transformation matrices
    T_wind2inertial = conditions.frames.wind.transform_to_inertial 
    T_body2inertial = conditions.frames.body.transform_to_inertial 
    T_inertial2body = orientation_transpose(T_body2inertial)
    T_inertia2wind  = orientation_transpose(T_wind2inertial) 

    # transform matrices 
    M_thrust_i = orientation_product(T_body2inertial,M_thrust_b) 
    M_aero_i   = orientation_product(T_wind2inertial, M_aero_w) 
    M_thrust_w = orientation_product(T_inertia2wind, M_thrust_i)
    
    M_tot_i = M_thrust_i + M_aero_i
    M_tot_w = M_thrust_w + M_aero_w
    
    # pack
    conditions.frames.inertial.total_moment_vector[:,:] = M_tot_i[:,:]
    conditions.frames.wind.total_moment_vector[:,:]     = M_tot_w[:,:]

    return
