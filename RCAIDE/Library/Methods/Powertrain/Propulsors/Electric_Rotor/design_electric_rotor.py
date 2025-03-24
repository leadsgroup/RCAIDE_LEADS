# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor/design_electric_rotor.py
# 
# Created:  Jul 2024, RCAIDE Team

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE 
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor                     import design_propeller 
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor                     import design_lift_rotor 
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor                     import design_prop_rotor 
from RCAIDE.Library.Methods.Powertrain.Converters.Motor                     import design_optimal_motor 
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric.Common import compute_motor_weight
from RCAIDE.Library.Methods.Powertrain                                      import setup_operating_conditions 

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Design Turbofan
# ---------------------------------------------------------------------------------------------------------------------- 
def design_electric_rotor(electric_rotor):
    """Compute perfomance properties of an electrically powered rotor, which is driven by an electric machine
    
    Assumtions:
       None 
    
    Source:
    
    Args:
        electric_rotor (dict):  electric rotor [-]
    
    Returns:
        None 
    
    """
    if electric_rotor.rotor == None:
        raise AssertionError("Rotor not specified on propulsor")
    rotor = electric_rotor.rotor

    if electric_rotor.motor == None:
        raise AssertionError("Motor not specified on propulsor")    
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
     
    # Static Sea Level Thrust   
    atmosphere            = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data_sea_level   = atmosphere.compute_values(0.0,0.0)   
    V                     = atmo_data_sea_level.speed_of_sound[0][0]*0.01 
    operating_state       = setup_operating_conditions(electric_rotor, altitude = 0,velocity_range=np.array([V]))  
    operating_state.conditions.energy.propulsors[electric_rotor.tag].throttle[:,0] = 1.0  
    sls_T,_,sls_P,_,_,_                          = electric_rotor.compute_performance(operating_state) 
    electric_rotor.sealevel_static_thrust        = sls_T[0][0]
    electric_rotor.sealevel_static_power         = sls_P[0][0]
     
    return 