# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Ducted_Fan/design_electric_ducted_fan.py
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
def design_electric_ducted_fan(EDF, new_regression_results=False, keep_files=True):
    """
    Compute performance properties of an electrically powered ducted fan, which is driven by an electric machine.
    
    Parameters
    ----------
    EDF : RCAIDE.Library.Components.Propulsors.Electric_Ducted_Fan
        Electric ducted fan propulsor component with the following attributes:
            - tag : str
                Identifier for the propulsor
            - electronic_speed_controller : Data
                ESC component
                    - bus_voltage : float
                        Bus voltage [V]
            - ducted_fan : Data
                Ducted fan component
                    - cruise : Data
                        Cruise conditions
                            - design_torque : float
                                Design torque [N·m]
                            - design_angular_velocity : float
                                Design angular velocity [rad/s]
            - motor : Data
                Electric motor component
                    - design_torque : float
                        Design torque [N·m]
                    - design_angular_velocity : float
                        Design angular velocity [rad/s]
    new_regression_results : bool, optional
        Flag to generate new regression results for the ducted fan
        Default: False
    keep_files : bool, optional
        Flag to keep temporary files generated during ducted fan design
        Default: True
    
    Returns
    -------
    None
        Results are stored in the EDF object:
            - sealevel_static_thrust : float
                Sea level static thrust [N]
            - sealevel_static_power : float
                Sea level static power [W]
    
    Notes
    -----
    This function performs several tasks:
        1. Validates that all required components are defined
        2. Designs the ducted fan using the design_ducted_fan function
        3. Sets the motor design parameters based on the ducted fan requirements
        4. Designs the motor for optimal performance
        5. Computes the motor weight
        6. Calculates the sea level static performance (thrust and power)
    
    The sea level static performance is calculated by:
        - Setting up atmospheric conditions at sea level
        - Creating a low-speed operating state (1% of sea level speed of sound)
        - Setting the throttle to maximum (1.0)
        - Computing the performance at these conditions
    
    **Major Assumptions**
        * US Standard Atmosphere 1976 is used for atmospheric properties
        * Sea level static conditions are approximated with a very low velocity (1% of speed of sound)
        * Full throttle (throttle = 1.0) is used for sea level static performance
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.design_ducted_fan
    RCAIDE.Library.Methods.Powertrain.Converters.Motor.design_optimal_motor
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric.Common.compute_motor_weight
    RCAIDE.Library.Methods.Powertrain.setup_operating_conditions
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