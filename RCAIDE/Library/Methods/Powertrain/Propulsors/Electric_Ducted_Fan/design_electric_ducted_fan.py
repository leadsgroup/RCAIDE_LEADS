# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Ducted_Fan/design_electric_rotor.py
# 
# Created:  Mar 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE 
from RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan                import design_ducted_fan  
from RCAIDE.Library.Methods.Powertrain.Converters.Motor                     import design_optimal_motor 
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric.Common import compute_motor_weight
from RCAIDE.Library.Methods.Powertrain                                      import setup_operating_conditions 

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Design Electric Ducted Fan 
# ---------------------------------------------------------------------------------------------------------------------- 
def design_electric_ducted_fan(EDF, new_regression_results = False, keep_files = True):
    """Compute perfomance properties of an electrically powered ducted fan, which is driven by an electric machine
    
    Assumtions:
       None 
    
    Source:
    
    Args:
        electric_rotor (dict):  electric ducted_fan [-]
    
    Returns:
        None 
    
    """
    if EDF.electronic_speed_controller == None: 
        raise AssertionError("electric speed controller not defined on propulsor")
    
    if EDF.electronic_speed_controller.bus_voltage == None: 
        raise AssertionError("ESC bus voltage not specified on propulsor")
    
    if EDF.ducted_fan == None:
        raise AssertionError("ducted fan not defined on propulsor")
    ducted_fan = EDF.ducted_fan

    if EDF.motor == None:
        raise AssertionError("Motor not specified on propulsor")    
    motor = EDF.motor
    
    design_ducted_fan(ducted_fan,new_regression_results, keep_files = keep_files)
    motor.design_torque            = ducted_fan.cruise.design_torque 
    motor.design_angular_velocity  = ducted_fan.cruise.design_angular_velocity
        
    # design motor 
    design_optimal_motor(motor)
    
    # compute weight of motor 
    compute_motor_weight(motor) 
     
    # Static Sea Level Thrust   
    atmosphere            = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data_sea_level   = atmosphere.compute_values(0.0,0.0)   
    V                     = atmo_data_sea_level.speed_of_sound[0][0]*0.01 
    operating_state       = setup_operating_conditions(EDF, altitude = 0,velocity_range=np.array([V]))  
    operating_state.conditions.energy.propulsors[EDF.tag].throttle[:,0] = 1.0  
    sls_T,_,sls_P,_,_,_               = EDF.compute_performance(operating_state) 
    EDF.sealevel_static_thrust        = sls_T[0][0]
    EDF.sealevel_static_power         = sls_P[0][0]
    return 