# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor/design_electric_rotor.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE 
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor                                   import design_propeller 
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor                                   import design_lift_rotor 
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor                                   import design_prop_rotor 
from RCAIDE.Library.Methods.Powertrain.Converters.Motor                                   import design_optimal_motor 
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric.Common               import compute_motor_weight

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Design Turbofan
# ---------------------------------------------------------------------------------------------------------------------- 
def design_electric_rotor(electric_rotor):
    """Compute perfomance properties of a propeller-driven internal combustion engine model
    Turbofan is created by manually linking the different components
    
    
    Assumtions:
       None 
    
    Source:
    
    Args:
        electric_rotor (dict):  electric rotor [-]
    
    Returns:
        None 
    
    """
    
    rotor = electric_rotor.rotor
    motor = electric_rotor.motor
    
    if type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Propeller: 
        design_propeller(rotor)
        motor.design_torque            = rotor.cruise.design_torque 
        motor.design_angular_velocity  = rotor.cruise.design_angular_velocity 
    if type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor:
        design_prop_rotor(rotor)
        motor.design_torque            = rotor.hover.design_torque 
        motor.design_angular_velocity  = rotor.hover.design_angular_velocity 
    else: 
        design_lift_rotor(rotor)  
        motor.design_torque            = rotor.hover.design_torque 
        motor.design_angular_velocity  = rotor.hover.design_angular_velocity
    
    # design motor 
    design_optimal_motor(motor)
    
    # compute weight of motor 
    compute_motor_weight(motor) 
     
    # Step 2 Static Sea Level Thrust  
    #atmo_data_sea_level   = atmosphere.compute_values(0.0,0.0)   
    #V                     = atmo_data_sea_level.speed_of_sound[0][0]*0.01 
    #operating_state       = setup_operating_conditions(turbofan, altitude = 0,velocity_range=np.array([V]))  
    #operating_state.conditions.energy.propulsors[turbofan.tag].throttle[:,0] = 1.0  
    #sls_T,_,sls_P,_,_,_                          = turbofan.compute_performance(operating_state) 
    #turbofan.sealevel_static_thrust              = sls_T[0][0]
    #turbofan.sealevel_static_power               = sls_P[0][0]
     
    return 