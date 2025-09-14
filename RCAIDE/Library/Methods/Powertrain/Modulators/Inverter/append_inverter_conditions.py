# RCAIDE/Library/Methods/Powertrain/Modulators/Inverter/append_inverter_conditions.py
# 
#
# Created:  Sep 2025, M. Guidotti

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_inverter_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_inverter_conditions(inverter,segment,energy_conditions): 
    """
   
    """
    
    ones_row                                                         = segment.state.ones_row 
    energy_conditions.modulators[inverter.tag]                       = Conditions()
    energy_conditions.modulators[inverter.tag].inputs                = Conditions()
    energy_conditions.modulators[inverter.tag].outputs               = Conditions()
    energy_conditions.modulators[inverter.tag].inputs.dc_voltage     = 0 * ones_row(1)    # [V]
    energy_conditions.modulators[inverter.tag].inputs.efficiency     = 0 * ones_row(1)  # [-] 0<eta<=1
    energy_conditions.modulators[inverter.tag].inputs.frequency      = 0 * ones_row(1)  # [Hz]
    energy_conditions.modulators[inverter.tag].inputs.target_vph_rms = 0 * ones_row(1)  # [V_rms] per-phase setpoint
    energy_conditions.modulators[inverter.tag].inputs.z_phase        = 0 * ones_row(1)  # [ohm] per-phase load impedance
    energy_conditions.modulators[inverter.tag].outputs.Vph_rms            = 0 * ones_row(1)  # 
    energy_conditions.modulators[inverter.tag].outputs.Vll_rms            = 0 * ones_row(1)
    energy_conditions.modulators[inverter.tag].outputs.Iph_rms            = 0 * ones_row(1)
    energy_conditions.modulators[inverter.tag].outputs.P_out              = 0 * ones_row(1)
    energy_conditions.modulators[inverter.tag].outputs.Q_out              = 0 * ones_row(1)
    energy_conditions.modulators[inverter.tag].outputs.S_out              = 0 * ones_row(1)
    energy_conditions.modulators[inverter.tag].outputs.pf                 = 0 * ones_row(1)
    energy_conditions.modulators[inverter.tag].outputs.P_in               = 0 * ones_row(1)
    energy_conditions.modulators[inverter.tag].outputs.Idc                = 0 * ones_row(1)
    energy_conditions.modulators[inverter.tag].outputs.m                  = 0 * ones_row(1)
    energy_conditions.modulators[inverter.tag].outputs.m_target           = 0 * ones_row(1)
    energy_conditions.modulators[inverter.tag].outputs.modulation_limited = 0 * ones_row(1)
    energy_conditions.modulators[inverter.tag].outputs.f_out              = 0 * ones_row(1)

    return 
