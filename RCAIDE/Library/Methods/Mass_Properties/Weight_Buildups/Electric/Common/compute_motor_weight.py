# RCAIDE/Library/Methods/Mass_Properties/Weight_Buildups/Electric/Common/compute_motor_weight.py
# 
# 
# Created:  Sep 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE  
 
# ----------------------------------------------------------------------------------------------------------------------
#  Motor Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_motor_weight(motor):
    """ Calculate the weight of motor using NASA correlations 
             
    
    Inputs:
            torque- maximum torque the motor can deliver safely      [N-m]
            kwt2
            xwt
            
    Outputs:
            mass- mass of the motor                                [kilograms]
        
    Properties Used:
            N/A
    """
    
    torque =  motor.design_torque
    mass  = -2E-7 * (torque ** 2) +  0.0117 * torque +  34.124
     
    return mass 