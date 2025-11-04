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
def design_electric_rotor(electric_rotor, network):
    """
    Computes performance properties of an electrically powered rotor.
    
    Parameters
    ----------
    electric_rotor : RCAIDE.Library.Components.Propulsors.Electric_Rotor
        Electric rotor propulsor component with the following attributes:
            - tag : str
                Identifier for the propulsor
            - electronic_speed_controller : Data
                ESC component
                    - bus_voltage : float
                        Bus voltage [V]
            - rotor : Data
                Rotor component (Propeller, Lift_Rotor, or Prop_Rotor)
            - motor : Data
                Electric motor component
                    - design_torque : float
                        Design torque [N·m]
                    - design_angular_velocity : float
                        Design angular velocity [rad/s]
                    - design_current : float
                        Design current [A]
    number_of_stations : int, optional
        Number of radial stations for rotor blade discretization
        Default: 20
    solver_name : str, optional
        Name of the numerical solver to use for rotor design
        Default: 'SLSQP'
    iterations : int, optional
        Maximum number of iterations for the solver
        Default: 200
    solver_sense_step : float, optional
        Step size for finite difference approximations in the solver
        Default: 1E-6
    solver_tolerance : float, optional
        Convergence tolerance for the solver
        Default: 1E-5
    print_iterations : bool, optional
        Flag to print solver iterations
        Default: False
    
    Returns
    -------
    None
        Results are stored in the electric_rotor object:
            - sealevel_static_thrust : float
                Sea level static thrust [N]
            - sealevel_static_power : float
                Sea level static power [W]
    
    Notes
    -----
    This function performs several tasks:
        1. Designs the rotor based on its type (propeller, lift rotor, or prop rotor)
        2. Sets the motor design parameters based on the rotor requirements
        3. Designs the motor for optimal performance
        4. Computes the motor weight
        5. Calculates the sea level static performance (thrust and power)
    
    The function handles different types of rotors:
        * For propellers, it uses the design_propeller function and sets the motor design
          parameters based on cruise conditions
        * For prop rotors, it uses the design_prop_rotor function and sets the motor design
          parameters based on hover conditions
        * For lift rotors, it uses the design_lift_rotor function and sets the motor design
          parameters based on hover conditions
    
    **Major Assumptions**
        * US Standard Atmosphere 1976 is used for atmospheric properties
        * Sea level static conditions are approximated with a very low velocity (1% of speed of sound)
        * Full throttle (throttle = 1.0) is used for sea level static performance
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.design_propeller
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.design_lift_rotor
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.design_prop_rotor
    RCAIDE.Library.Methods.Powertrain.Converters.Motor.design_optimal_motor
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric.Common.compute_motor_weight
    RCAIDE.Library.Methods.Powertrain.setup_operating_conditions
    """
    for modulator_tag in electric_rotor.assigned_modulators:
        if modulator_tag == None:
            raise AssertionError("Electric Speed Controller not defined on propulsor")
        if isinstance(network.modulators[modulator_tag[0][0]], RCAIDE.Library.Components.Powertrain.Modulators.Electronic_Speed_Controller):
            esc = network.modulators[modulator_tag[0][0]]
    
    for converter_tag in electric_rotor.assigned_converters:
        if converter_tag == None:
            raise AssertionError("Electric Motor or Rotor not defined on propulsor")
        if isinstance(network.converters[converter_tag[0][0]], RCAIDE.Library.Components.Powertrain.Converters.Rotor):
            rotor = network.converters[converter_tag[0][0]]
        elif isinstance(network.converters[converter_tag[0][0]], RCAIDE.Library.Components.Powertrain.Converters.Motor):
            motor = network.converters[converter_tag[0][0]]
    
    if type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Propeller: 
        design_propeller(rotor)
        motor.design_torque            = rotor.cruise.design_torque 
        motor.design_angular_velocity  = rotor.cruise.design_angular_velocity 
    elif type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor:
        design_prop_rotor(rotor)
        motor.design_torque            = rotor.hover.design_torque 
        motor.design_angular_velocity  = rotor.hover.design_angular_velocity 
    elif type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor: 
        design_lift_rotor(rotor)
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
    operating_state       = setup_operating_conditions(electric_rotor, network, velocity_range=np.array([V]), altitude = 0, angle_of_attack=0, temperature_deviation=0)  
    operating_state.conditions.energy.propulsors[electric_rotor.tag].throttle[:,0] = 1.0
    operating_state.conditions.energy.converters[motor.tag].inputs.current[:,0] =  motor.design_current

    _,sls_outputs,_ ,_                           = electric_rotor.compute_performance(network, operating_state) 
    electric_rotor.sealevel_static_thrust       = sls_outputs.thrust[0][0]
    electric_rotor.sealevel_static_power        = sls_outputs.power.propulsive[0][0]
     
    return 