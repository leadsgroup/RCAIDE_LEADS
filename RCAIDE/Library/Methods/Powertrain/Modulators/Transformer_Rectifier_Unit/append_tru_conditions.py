# RCAIDE/Library/Methods/Powertrain/Modulators/Transformer_Rectifier_Unit/append_tru_conditions.py
# 
#
# Created:  Sep 2025, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_tru_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_tru_conditions(tru,segment,energy_conditions): 
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
    
    ones_row                                                        = segment.state.ones_row 
    energy_conditions.modulators[tru.tag]                           = Conditions()
    energy_conditions.modulators[tru.tag].inputs                    = Conditions()
    energy_conditions.modulators[tru.tag].outputs                   = Conditions()
    energy_conditions.modulators[tru.tag].inputs.voltage            = 0 * ones_row(1)  
    energy_conditions.modulators[tru.tag].inputs.Vll_rms_primary    = 0 * ones_row(1)   # [V_rms] line-line AC primary voltage
    energy_conditions.modulators[tru.tag].inputs.turns_ratio        = 0 * ones_row(1)   # [-] transformer turns ratio (N_primary / N_secondary)
    energy_conditions.modulators[tru.tag].inputs.diode_drop         = 0 * ones_row(1)   # [V] average forward drop per diode
    energy_conditions.modulators[tru.tag].inputs.eta_tru            = 0 * ones_row(1)   # [-] transformer-rectifier efficiency (0<eta<=1)
    energy_conditions.modulators[tru.tag].inputs.pf_assumed         = 0 * ones_row(1)   # [-] assumed power factor at AC input (for sizing)
    energy_conditions.modulators[tru.tag].inputs.R_load             = 0 * ones_row(1)   # [ohm] DC load (use either R_load or Idc_set)
    energy_conditions.modulators[tru.tag].inputs.Idc_set            = 0 * ones_row(1)   # [A] desired DC current (use either R_load or Idc_set) 
    energy_conditions.modulators[tru.tag].outputs.Vll_rms_secondary = 0 * ones_row(1)   # [V_rms] AC secondary line-line
    energy_conditions.modulators[tru.tag].outputs.Vdc_ideal         = 0 * ones_row(1)   # [V] ideal no-drop DC
    energy_conditions.modulators[tru.tag].outputs.Vdc_no_load       = 0 * ones_row(1)   # [V] minus diode drops
    energy_conditions.modulators[tru.tag].outputs.Vdc               = 0 * ones_row(1)   # [V] averaged DC at load
    energy_conditions.modulators[tru.tag].outputs.Idc               = 0 * ones_row(1)   # [A] DC current
    energy_conditions.modulators[tru.tag].outputs.P_out_W           = 0 * ones_row(1)   # [W] DC real power delivered
    energy_conditions.modulators[tru.tag].outputs.P_in_W            = 0 * ones_row(1)   # [W] AC real power drawn
    energy_conditions.modulators[tru.tag].outputs.S_in_VA           = 0 * ones_row(1)   # [VA] input apparent power
    energy_conditions.modulators[tru.tag].outputs.I_line_rms_A      = 0 * ones_row(1)   # [A_rms] input line current per phase

    return 