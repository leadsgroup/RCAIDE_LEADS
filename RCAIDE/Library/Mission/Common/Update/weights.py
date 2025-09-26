# RCAIDE/Library/Missions/Common/Update/weights.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE

# package imports 
import numpy as np  
    
# ----------------------------------------------------------------------------------------------------------------------
# Update Weights
# ----------------------------------------------------------------------------------------------------------------------  
def weights(segment): 
    """ Updates the weight of the vehicle 
        
        Assumptions:
        N/A
        
        Inputs:
             segment.state.
                 numerics.time.integrate               [-]
                 conditions.weights.total_mass         [kg]
                 conditions.weights.vehicle_mass_rate  [kg/s]
                 conditions.freestream.gravity         [m/s^2]

                 
        Outputs: 
            segment.state.conditions.
                 weights.total_mass
                 frames.inertial.gravity_force_vector
      
        Properties Used:
        N/A
                    
    """ 
    
    # unpack
    conditions     = segment.state.conditions
    I              = segment.state.numerics.time.integrate  
    m_0_vehicle    = conditions.weights.total_mass[0,0]
    m_dot_vehicle  = conditions.weights.vehicle_mass_rate
    g              = conditions.freestream.gravity    

    # --------------------------------------------------------------------------       
    # update mass 
    # --------------------------------------------------------------------------       
    if (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude) or\
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_AVL_Trimmed) or \
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_No_Propulsion) or \
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Throttle): 
        
        W = m_0_vehicle*g 
        conditions.frames.inertial.gravity_force_vector[:,2] = W[:,0]
        
    else:
        
        m = m_0_vehicle + np.dot(I, -m_dot_vehicle)
    
        # weight
        W = m*g
        
        # pack
        conditions.weights.total_mass[1:,0]                  = m[1:,0]  
        conditions.frames.inertial.gravity_force_vector[:,2] = W[:,0]     
    return
 