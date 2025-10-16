# RCAIDE/Library/Methods/Powertrain/Modulators/tru/compute_tru_performance.py
# 
# 
# Created:  Sep 2025, M. Guidotti

import numpy as np

def compute_tru_performance(tru, state):

    """
   
    """
    tru_conditions = state.conditions.energy.modulators[tru.tag]

    # Inputs
    Vll_rms_primary = tru_conditions.inputs.Vll_rms_primary  # [V_rms] line-line AC primary voltage
    Idc_set         = tru_conditions.inputs.Idc_set          # [A] desired DC current (use either R_load or Idc_set)

    turns_ratio     = tru.turns_ratio      # [-] transformer turns ratio (N_primary / N_secondary)
    diode_drop      = tru.diode_drop       # [V] average forward drop per diode
    eta_tru         = tru.efficiency       # [-] transformer-rectifier efficiency (0<eta<=1)
    R_load          = tru.R_load           # [ohm] DC load (use either R_load or Idc_set)

    # Secondary AC and simple DC estimate
    Vll_rms_sec = np.asarray(Vll_rms_primary) / np.maximum(np.asarray(turns_ratio), 1e-9)
    Vdc_no_load = np.maximum(1.35 * Vll_rms_sec - 2.0 * np.asarray(diode_drop), 0.0)

    # DC current
    if Idc_set is not None:
        Idc = np.asarray(Idc_set) * np.ones_like(Vdc_no_load)
    elif R_load is not None:
        Idc = Vdc_no_load / np.maximum(np.asarray(R_load), 1e-9)
    else:
        Idc = 10.0 * np.ones_like(Vdc_no_load)  # A, tiny default load
        
    # DC voltage under load 
    Vdc = Vdc_no_load

    # Output DC power 
    P_out = Vdc * Idc                    # [W]

    P_in  = P_out / eta_tru              # [W]

    # Populate a few handy outputs/inputs (lightweight)
    tru_conditions.outputs.ac_voltage_secondary   = Vll_rms_sec
    tru_conditions.outputs.dc_voltage_average     = Vdc
    tru_conditions.outputs.dc_current             = Idc
    tru_conditions.outputs.dc_real_power          = P_out
    tru_conditions.inputs.ac_real_power           = P_in

    # Report to network evaluator:
    P_mech = 0.0
    P_elec = P_out          
    stored_results_flag = True
    stored_modulator_tag = tru.tag

    return P_mech,P_elec,stored_results_flag,stored_modulator_tag