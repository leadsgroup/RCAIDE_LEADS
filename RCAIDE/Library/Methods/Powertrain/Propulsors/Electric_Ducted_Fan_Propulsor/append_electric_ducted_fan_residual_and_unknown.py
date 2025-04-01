# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/append_electric_ducted_fan_residual_and_unknowns.py 
# 
# Created:  Jun 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_electric_ducted_fan_residual_and_unknown
# ----------------------------------------------------------------------------------------------------------------------  
def append_electric_ducted_fan_residual_and_unknown(propulsor,segment):
    """
    Appends the power coefficient as an unknown and motor torque as a residual for an electric ducted fan propulsor system.

    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors
        The electric ducted fan propulsor system
            - ducted_fan : Component
                The ducted fan component of the propulsor
            - tag : str
                Identifier for the propulsor system
    segment : RCAIDE.Framework.Mission.Segments.Segment
        The mission segment being analyzed
            - state : State
                Contains the flight condition state variables

    Returns
    -------
    None

    Notes
    -----
    This function initializes the power coefficient unknown using the design power 
    coefficient from cruise conditions and sets up the motor torque residual that 
    will be used in the solver.
    """ 
    ones_row    = segment.state.ones_row 
    ducted_fan   = propulsor.ducted_fan
    cp_init      = ducted_fan.cruise.design_power_coefficient
    segment.state.unknowns[ propulsor.tag  + '_ducted_fan_cp']               = cp_init * ones_row(1)  
    segment.state.residuals.network[ propulsor.tag  + '_ducted_fan_motor_torque'] = 0. * ones_row(1)    
    
    return 