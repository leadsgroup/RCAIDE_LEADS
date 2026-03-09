# RCAIDE/Library/Methods/Powertrain/Modulators/Transformer_Rectifier_Unit/append_TRU_conditions.py
# 
#
# Created:  Sep 2025, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_TRU_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_TRU_conditions(TRU,segment): 
    """
    Initializes the Electronic Speed Controller (TRU) condition containers for tracking 
    electrical state variables. Sets up basic input/output conditions and throttle settings 
    for TRU performance analysis.

    Parameters
    ----------
    TRU : RCAIDE.Library.Components.Energy.Modulators.Electronic_Speed_Controller
        The electronic speed controller component
            - tag : str
                Identifier for the TRU
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
    segment.state.conditions.energy.modulators[TRU.tag]                           = Conditions()
    segment.state.conditions.energy.modulators[TRU.tag].inputs                    = Conditions()
    segment.state.conditions.energy.modulators[TRU.tag].inputs.power              = Conditions()  
    segment.state.conditions.energy.modulators[TRU.tag].inputs.voltage            = 0 * ones_row(1)  
    segment.state.conditions.energy.modulators[TRU.tag].inputs.Vll_rms_primary    = 1 * ones_row(1)   # [V_rms] line-line AC primary voltage
    segment.state.conditions.energy.modulators[TRU.tag].inputs.Idc_set            = 0 * ones_row(1)   # [A] desired DC current (use either R_load or Idc_set) 
    segment.state.conditions.energy.modulators[TRU.tag].inputs.power.propulsive   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[TRU.tag].inputs.power.mechanical   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[TRU.tag].inputs.power.electrical   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[TRU.tag].inputs.power.chemical     = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[TRU.tag].inputs.power.pneumatic    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[TRU.tag].inputs.power.hydraulic    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[TRU.tag].inputs.power.thermal      = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[TRU.tag].outputs                   = Conditions()
    segment.state.conditions.energy.modulators[TRU.tag].outputs.power             = Conditions()  
    segment.state.conditions.energy.modulators[TRU.tag].outputs.power.propulsive  = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[TRU.tag].outputs.power.mechanical  = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[TRU.tag].outputs.power.electrical  = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[TRU.tag].outputs.power.chemical    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[TRU.tag].outputs.power.pneumatic   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[TRU.tag].outputs.power.hydraulic   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[TRU.tag].outputs.power.thermal     = 0 * ones_row(1)
    
    return

def append_tru_segment_conditions(TRU, segment): 
    energy_conditions  = segment.state.conditions.energy     
    energy_conditions.modulators[TRU.tag].inputs.power.propulsive[:,0]    = 0.0
    energy_conditions.modulators[TRU.tag].inputs.power.mechanical[:,0]    = 0.0 
    energy_conditions.modulators[TRU.tag].inputs.power.electrical[:,0]    = 0.0 
    energy_conditions.modulators[TRU.tag].inputs.power.chemical[:,0]      = 0.0
    energy_conditions.modulators[TRU.tag].inputs.power.pneumatic[:,0]     = 0.0 
    energy_conditions.modulators[TRU.tag].inputs.power.hydraulic[:,0]     = 0.0 
    energy_conditions.modulators[TRU.tag].inputs.power.thermal[:,0]       = 0.0 
    energy_conditions.modulators[TRU.tag].outputs.power.propulsive[:,0]   = 0.0
    energy_conditions.modulators[TRU.tag].outputs.power.mechanical[:,0]   = 0.0   
    energy_conditions.modulators[TRU.tag].outputs.power.electrical[:,0]   = 0.0 
    energy_conditions.modulators[TRU.tag].outputs.power.chemical[:,0]     = 0.0
    energy_conditions.modulators[TRU.tag].outputs.power.pneumatic[:,0]    = 0.0   
    energy_conditions.modulators[TRU.tag].outputs.power.hydraulic[:,0]    = 0.0   
    energy_conditions.modulators[TRU.tag].outputs.power.thermal[:,0]      = 0.0       