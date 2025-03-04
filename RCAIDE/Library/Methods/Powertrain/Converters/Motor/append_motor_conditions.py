# RCAIDE/Library/Methods/Powertrain/Converters/Motor/append_motor_conditions.py
# 
# Created:  Jan 2025, M. Clarke, M. Guidotti

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_motor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_motor_conditions(motor,segment,propulsor_conditions): 

    """
    Initializes motor operating conditions for a mission segment.

    Parameters
    ----------
    motor : Converter
        Motor component (DC_Motor or PMSM_Motor) for which conditions are being initialized
    segment : Segment
        Mission segment containing the state conditions
    propulsor_conditions : Conditions
        Container for propulsor operating conditions

    Returns
    -------
    None
        Modifies propulsor_conditions in-place by adding motor-specific conditions

    Notes
    -----
    This function initializes arrays of zeros for key motor operating parameters during
    a mission segment. The conditions are stored in a nested structure under the motor's
    tag within propulsor_conditions.

    The following conditions are initialized:
        - torque: Motor output torque [N-m]
        - efficiency: Motor operating efficiency [-] 
        - current: Motor current draw [A]
        - voltage: Motor voltage [V]
        - rotor_power_coefficient: Non-dimensional power coefficient [-]

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.DC_Motor
    RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor
    """


    ones_row    = segment.state.ones_row 
    propulsor_conditions[motor.tag]                         = Conditions()
    propulsor_conditions[motor.tag].inputs                  = Conditions()
    propulsor_conditions[motor.tag].outputs                 = Conditions()
    propulsor_conditions[motor.tag].torque                  = 0. * ones_row(1) 
    propulsor_conditions[motor.tag].efficiency              = 0. * ones_row(1) 
    propulsor_conditions[motor.tag].current                 = 0. * ones_row(1) 
    propulsor_conditions[motor.tag].voltage                 = 0. * ones_row(1)
    propulsor_conditions[motor.tag].rotor_power_coefficient = 0. * ones_row(1)
    return 

