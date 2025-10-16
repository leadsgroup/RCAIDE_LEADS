# RCAIDE/Library/Methods/Powertrain/Modulators/Transformer_Rectifier_Unit/append_tru_conditions.py
# 
#
# Created:  Sep 2025, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_tru_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_tru_conditions(tru,segment): 
    """
    Initializes the Electronic Speed Controller (tru) condition containers for tracking 
    electrical state variables. Sets up basic input/output conditions and throttle settings 
    for tru performance analysis.

    Parameters
    ----------
    tru : RCAIDE.Library.Components.Energy.Modulators.Electronic_Speed_Controller
        The electronic speed controller component
            - tag : str
                Identifier for the tru
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
    
    ones_row                                                                      = segment.state.ones_row 
    segment.state.conditions.energy.modulators[tru.tag]                           = Conditions()
    segment.state.conditions.energy.modulators[tru.tag].inputs                    = Conditions()
    segment.state.conditions.energy.modulators[tru.tag].outputs                   = Conditions()
    segment.state.conditions.energy.modulators[tru.tag].inputs.voltage            = 0 * ones_row(1)  
    segment.state.conditions.energy.modulators[tru.tag].inputs.Vll_rms_primary    = 1 * ones_row(1)   # [V_rms] line-line AC primary voltage
    segment.state.conditions.energy.modulators[tru.tag].inputs.Idc_set            = 0 * ones_row(1)   # [A] desired DC current (use either R_load or Idc_set) 
    
    return 