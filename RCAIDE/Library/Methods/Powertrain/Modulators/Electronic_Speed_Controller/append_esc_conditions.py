# RCAIDE/Library/Methods/Powertrain/Modulators/Electronic_Speed_Controller/append_motor_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_esc_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_esc_conditions(esc,segment,energy_conditions): 
    """
    Initializes the Electronic Speed Controller (ESC) condition containers for tracking 
    electrical state variables. Sets up basic input/output conditions and throttle settings 
    for ESC performance analysis.

    Parameters
    ----------
    esc : RCAIDE.Library.Components.Energy.Modulators.Electronic_Speed_Controller
        The electronic speed controller component
            - tag : str
                Identifier for the ESC
    segment : RCAIDE.Framework.Mission.Segments.Segment
        The mission segment being analyzed
            - state : State
                Contains the flight condition state variables
                    - ones_row : function
                        Returns array of ones with specified size
    propulsor_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Container for propulsor-specific conditions

    Returns
    -------
    None

    Notes
    -----
    Creates and initializes the following state variables:
        - inputs : Conditions
            Container for input electrical parameters
        - outputs : Conditions
            Container for output electrical parameters
        - throttle : float
            Power modulation setting from 0 to 1
    """
    
    ones_row    = segment.state.ones_row 
    energy_conditions.modulators[esc.tag]                  = Conditions()
    energy_conditions.modulators[esc.tag].inputs           = Conditions()
    energy_conditions.modulators[esc.tag].outputs          = Conditions()
    energy_conditions.modulators[esc.tag].throttle         = 0. * ones_row(1)  
    energy_conditions.modulators[esc.tag].outputs.voltage  = 0. * ones_row(1)  
    energy_conditions.modulators[esc.tag].inputs.voltage   = esc.bus_voltage * ones_row(1)   
    return 
