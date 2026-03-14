# RCAIDE/Library/Methods/Powertrain/Modulators/Inverter/append_inverter_conditions.py
# 
#
# Created:  Sep 2025, M. Guidotti

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_inverter_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_inverter_conditions(inverter,segment): 
    """
   
    """
    
    ones_row                                                                            = segment.state.ones_row 
    segment.state.conditions.energy.modulators[inverter.tag]                            = Conditions()
    segment.state.conditions.energy.modulators[inverter.tag].inputs                     = Conditions()
    segment.state.conditions.energy.modulators[inverter.tag].inputs.dc_voltage          = 0 * ones_row(1)    # [V]
    segment.state.conditions.energy.modulators[inverter.tag].inputs.efficiency          = 0 * ones_row(1)  # [-] 0<eta<=1
    segment.state.conditions.energy.modulators[inverter.tag].inputs.frequency           = 0 * ones_row(1)  # [Hz]
    segment.state.conditions.energy.modulators[inverter.tag].inputs.target_vph_rms      = 0 * ones_row(1)  # [V_rms] per-phase setpoint
    segment.state.conditions.energy.modulators[inverter.tag].inputs.z_phase             = 0 * ones_row(1)  # [ohm] per-phase load impedance
    segment.state.conditions.energy.modulators[inverter.tag].inputs.power               = Conditions()
    segment.state.conditions.energy.modulators[inverter.tag].inputs.power.propulsive    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].inputs.power.mechanical    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].inputs.power.electrical    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].inputs.power.chemical      = 0 * ones_row(1) 
    segment.state.conditions.energy.modulators[inverter.tag].inputs.power.hydraulic     = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].inputs.power.thermal       = 0 * ones_row(1) 
    segment.state.conditions.energy.modulators[inverter.tag].outputs                    = Conditions()
    segment.state.conditions.energy.modulators[inverter.tag].outputs.Vph_rms            = 0 * ones_row(1)   
    segment.state.conditions.energy.modulators[inverter.tag].outputs.Vll_rms            = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.Iph_rms            = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.P_out              = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.Q_out              = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.S_out              = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.pf                 = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.P_in               = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.Idc                = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.m                  = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.m_target           = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.modulation_limited = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.f_out              = 0 * ones_row(1) 
    segment.state.conditions.energy.modulators[inverter.tag].outputs.power              = Conditions()
    segment.state.conditions.energy.modulators[inverter.tag].outputs.power.propulsive   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.power.mechanical   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.power.electrical   = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.power.chemical     = 0 * ones_row(1) 
    segment.state.conditions.energy.modulators[inverter.tag].outputs.power.hydraulic    = 0 * ones_row(1)
    segment.state.conditions.energy.modulators[inverter.tag].outputs.power.thermal      = 0 * ones_row(1)
     

    return 

def append_inverter_segment_conditions(inverter, segment): 
    energy_conditions  = segment.state.conditions.energy     
    energy_conditions.modulators[inverter.tag].inputs.power.propulsive[:,0]    = 0.0
    energy_conditions.modulators[inverter.tag].inputs.power.mechanical[:,0]    = 0.0 
    energy_conditions.modulators[inverter.tag].inputs.power.electrical[:,0]    = 0.0 
    energy_conditions.modulators[inverter.tag].inputs.power.chemical[:,0]      = 0.0
    energy_conditions.modulators[inverter.tag].inputs.power.pneumatic[:,0]     = 0.0 
    energy_conditions.modulators[inverter.tag].inputs.power.hydraulic[:,0]     = 0.0 
    energy_conditions.modulators[inverter.tag].inputs.power.thermal[:,0]       = 0.0 
    energy_conditions.modulators[inverter.tag].outputs.power.propulsive[:,0]   = 0.0
    energy_conditions.modulators[inverter.tag].outputs.power.mechanical[:,0]   = 0.0   
    energy_conditions.modulators[inverter.tag].outputs.power.electrical[:,0]   = 0.0 
    energy_conditions.modulators[inverter.tag].outputs.power.chemical[:,0]     = 0.0
    energy_conditions.modulators[inverter.tag].outputs.power.pneumatic[:,0]    = 0.0   
    energy_conditions.modulators[inverter.tag].outputs.power.hydraulic[:,0]    = 0.0   
    energy_conditions.modulators[inverter.tag].outputs.power.thermal[:,0]      = 0.0   