# RCAIDE/Library/Methods/Powertrain/Converters/Motor/append_motor_conditions.py
# 
# Created:  Jan 2025, M. Clarke, M. Guidotti

from RCAIDE.Framework.Mission.Common     import   Conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Common.append_converter_power_conditions import append_converter_power_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_motor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_motor_conditions(motor,segment): 

    """
    Initializes motor operating conditions for a mission segment.

    Parameters
    ----------
    motor : Converter
        Motor component (DC_Motor or PMSM_Motor) for which conditions are being initialized
    segment : Segment
        Mission segment containing the state conditions

    Returns
    -------
    None
        Modifies segment.state.conditions.energy in-place by adding motor-specific conditions

    Notes
    -----
    This function initializes arrays of zeros for key motor operating parameters during
    a mission segment. The conditions are stored in a nested structure under the motor's
    tag within segment.state.conditions.energy.

    The following conditions are initialized:
        - torque: Motor output torque [N-m]
        - efficiency: Motor operating efficiency [-] 
        - current: Motor current draw [A]
        - voltage: Motor voltage [V]

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.DC_Motor
    RCAIDE.Library.Components.Powertrain.Converters.PMSM_Motor
    """


    ones_row         = segment.state.ones_row
    motor_conditions = append_converter_power_conditions(motor, segment)
    motor_conditions.efficiency        = 0. * ones_row(1)
    motor_conditions.inputs.voltage    = 0. * ones_row(1)
    motor_conditions.inputs.current    = 0. * ones_row(1)
    motor_conditions.outputs.work_done = 0. * ones_row(1)
    motor_conditions.outputs.torque    = 0. * ones_row(1)
    motor_conditions.outputs.omega     = 0. * ones_row(1)
    return

