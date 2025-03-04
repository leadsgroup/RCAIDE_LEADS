# RCAIDE/Library/Methods/Powertrain/Converters/Ducted_Fan/append_ducted_fan_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_motor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_ducted_fan_conditions(ducted_fan,segment,energy_conditions): 

    """
    Initializes ducted fan operating conditions for a mission segment.
    
    Parameters
    ----------
    ducted_fan : Converter
        Ducted fan component for which conditions are being initialized
    segment : Segment
        Mission segment containing the state conditions
    energy_conditions : Conditions
        Container for energy system operating conditions
    
    Returns
    -------
    None
        Modifies energy_conditions in-place by adding ducted-fan-specific conditions
    
    Notes
    -----
    This function initializes arrays for key ducted fan operating parameters during
    a mission segment. The conditions are stored in a nested structure under the 
    ducted fan's tag within energy_conditions.
    
    The following conditions are initialized:
        - orientation : array(3)
            Fan orientation angles [rad]
        - commanded_thrust_vector_angle : array(1) 
            Commanded thrust vectoring angle [rad]
        - torque : array(1)
            Shaft torque [N-m]
        - throttle : array(1)
            Throttle setting [-]
        - thrust : array(1)
            Net thrust [N]
        - rpm : array(1)
            Rotor speed [RPM]
        - omega : array(1)
            Angular velocity [rad/s]
        - disc_loading : array(1)
            Thrust per unit disc area [N/m^2]
        - power_loading : array(1)
            Power per unit disc area [W/m^2]
        - tip_mach : array(1)
            Blade tip Mach number [-]
        - efficiency : array(1)
            Overall efficiency [-]
        - figure_of_merit : array(1)
            Hovering figure of merit [-]
        - power_coefficient : array(1)
            Non-dimensional power coefficient [-]
    
    **Major Assumptions**
        * All conditions except throttle are initialized as zero arrays
        * Throttle is initialized as ones array
        * Array length matches the segment state vector length
        * Conditions are stored using the ducted fan's tag as the dictionary key
    
    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Ducted_Fan
    RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.compute_ducted_fan_performance
    """

    ones_row    = segment.state.ones_row 
    energy_conditions[ducted_fan.tag]                               = Conditions()   
    energy_conditions[ducted_fan.tag].orientation                   = 0. * ones_row(3) 
    energy_conditions[ducted_fan.tag].commanded_thrust_vector_angle = 0. * ones_row(1) 
    energy_conditions[ducted_fan.tag].torque                        = 0. * ones_row(1)
    energy_conditions[ducted_fan.tag].throttle                      = ones_row(1)
    energy_conditions[ducted_fan.tag].thrust                        = 0. * ones_row(1)
    energy_conditions[ducted_fan.tag].rpm                           = 0. * ones_row(1)
    energy_conditions[ducted_fan.tag].omega                         = 0. * ones_row(1)
    energy_conditions[ducted_fan.tag].disc_loading                  = 0. * ones_row(1)                 
    energy_conditions[ducted_fan.tag].power_loading                 = 0. * ones_row(1)
    energy_conditions[ducted_fan.tag].tip_mach                      = 0. * ones_row(1)
    energy_conditions[ducted_fan.tag].efficiency                    = 0. * ones_row(1)
    energy_conditions[ducted_fan.tag].figure_of_merit               = 0. * ones_row(1)
    energy_conditions[ducted_fan.tag].power_coefficient             = 0. * ones_row(1)  
    return 


