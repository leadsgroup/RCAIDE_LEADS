# RCAIDE/Library/Methods/Powertrain/Converters/Motor/append_motor_conditions.py
# 
# Created:  Jan 2025, M. Clarke, M. Guidotti

from RCAIDE.Framework.Mission.Common     import   Conditions

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


    ones_row    = segment.state.ones_row 
    segment.state.conditions.energy.converters[motor.tag]                         = Conditions()
    segment.state.conditions.energy.converters[motor.tag].inputs                  = Conditions()
    segment.state.conditions.energy.converters[motor.tag].outputs                 = Conditions()
    segment.state.conditions.energy.converters[motor.tag].efficiency              = 0. * ones_row(1)  
    segment.state.conditions.energy.converters[motor.tag].inputs.voltage          = 0. * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].inputs.current          = 0. * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].outputs.work_done       = 0. * ones_row(1) 
    segment.state.conditions.energy.converters[motor.tag].outputs.torque          = 0. * ones_row(1)  
    segment.state.conditions.energy.converters[motor.tag].outputs.omega           = 0. * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].inputs.power            = Conditions() 
    segment.state.conditions.energy.converters[motor.tag].inputs.power.mechanical = 0 * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].inputs.power.electrical = 0 * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].inputs.power.chemical   = 0 * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].inputs.power.pneumatic  = 0 * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].inputs.power.hydraulic  = 0 * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].inputs.power.thermal    = 0 * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].outputs.power            = Conditions() 
    segment.state.conditions.energy.converters[motor.tag].outputs.power.mechanical = 0 * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].outputs.power.electrical = 0 * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].outputs.power.chemical   = 0 * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].outputs.power.pneumatic  = 0 * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].outputs.power.hydraulic  = 0 * ones_row(1)
    segment.state.conditions.energy.converters[motor.tag].outputs.power.thermal    = 0 * ones_row(1)
    return 

