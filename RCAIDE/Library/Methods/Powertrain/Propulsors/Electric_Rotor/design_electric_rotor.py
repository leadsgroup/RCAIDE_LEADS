# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor/design_electric_rotor.py
# 
# Created:  Mar 2025, M. Clarke

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
#  Design Electric Rotor 
# ---------------------------------------------------------------------------------------------------------------------- 
def design_electric_rotor(electric_rotor,number_of_stations = 20,solver_name= 'SLSQP',iterations = 200,
                      solver_sense_step = 1E-5,solver_tolerance = 1E-4,print_iterations = False):
    """Compute perfomance properties of an electrically powered rotor, which is driven by an electric machine
    
    Assumtions:
       None 
    
    Source:
    
    Args:
        electric_rotor (dict):  electric rotor [-]
    
    Returns:
        None 
    
    """

    if electric_rotor.electronic_speed_controller == None: 
        raise AssertionError("Electric Speed Controller not defined on propulsor")
    
    if electric_rotor.electronic_speed_controller.bus_voltage == None: 
        raise AssertionError("Electric Speed Controller  bus voltage not specified on propulsor") 
    
    if electric_rotor.rotor == None:
        raise AssertionError("Rotor not defined on propulsor")
    rotor = electric_rotor.rotor

    if electric_rotor.motor == None:
        raise AssertionError("Motor not defined on propulsor")
    
    motor = electric_rotor.motor
    
    if type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Propeller: 
        design_propeller(rotor,number_of_stations = number_of_stations)
        motor.design_torque            = rotor.cruise.design_torque 
        motor.design_angular_velocity  = rotor.cruise.design_angular_velocity 
    elif type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor:
        design_prop_rotor(rotor,number_of_stations ,solver_name,iterations,solver_sense_step,solver_tolerance,print_iterations)
        motor.design_torque            = rotor.hover.design_torque 
        motor.design_angular_velocity  = rotor.hover.design_angular_velocity 
    elif type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor: 
        design_lift_rotor(rotor,number_of_stations ,solver_name,iterations,solver_sense_step,solver_tolerance,print_iterations)
        motor.design_torque            = rotor.hover.design_torque 
        motor.design_angular_velocity  = rotor.hover.design_angular_velocity
    
    # design motor if design torque is specified 
    if motor.design_torque != None: 
        design_optimal_motor(motor)
    
        # compute weight of motor 
        compute_motor_weight(motor) 
     
    # Static Sea Level Thrust   
    atmosphere            = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976() 
    atmo_data_sea_level   = atmosphere.compute_values(0.0,0.0)   
    V                     = atmo_data_sea_level.speed_of_sound[0][0]*0.01 
    operating_state       = setup_operating_conditions(electric_rotor, altitude = 0,velocity_range=np.array([V]))  
    operating_state.conditions.energy.propulsors[electric_rotor.tag].throttle[:,0] = 1.0
    operating_state.conditions.energy.converters[motor.tag].inputs.current[:,0] =  motor.design_current
    sls_T,_,sls_P,_,_,_                          = electric_rotor.compute_performance(operating_state) 
    electric_rotor.sealevel_static_thrust        = sls_T[0][0]
    electric_rotor.sealevel_static_power         = sls_P[0][0]
     
    return 