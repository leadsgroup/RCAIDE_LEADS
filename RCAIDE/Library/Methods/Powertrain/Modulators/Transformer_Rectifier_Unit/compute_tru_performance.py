# RCAIDE/Library/Methods/Powertrain/Modulators/tru/compute_tru_performance.py
# 
# 
# Created:  Sep 2025, M. Guidotti

import numpy as np

def compute_tru_performance(tru, network, state):

    """
   
    """
    tru_conditions = state.conditions.energy.modulators[tru.tag]

    for distributor_tag in tru.assigned_distributors[0]:
        if network.distributors[distributor_tag].bus_type == 'AC':
            Vll = network.distributors[distributor_tag].voltage_phase_to_phase
        elif network.distributors[distributor_tag].bus_type == 'DC':
            Vdc = network.distributors[distributor_tag].voltage

    # -------------------------
    # Component parameters 
    # -------------------------
    eta        = tru.electrical_efficiency
    turns      = tru.turns_ratio
    diode_drop = tru.diode_drop
    R_load     = tru.R_load
    Idc_set    = tru.Idc_set

    # -------------------------
    # Estimate DC voltage if DC bus voltage isn't provided
    # Vdc_open ≈ 1.35 * Vll_sec - 2*diode_drop  (6-pulse)
    # -------------------------
    Vll_sec  = Vll / turns
    Vdc_open = max(1.35 * Vll_sec - 2.0 * diode_drop, 0.0)
    if Vdc <= 0.0:
        Vdc_use = Vdc_open
    else:
        Vdc_use = Vdc

    # -------------------------
    # Choose DC current (Idc)
    # -------------------------
    if Idc_set is not None:
        Idc = float(Idc_set)
    elif R_load is not None and float(R_load) > 0.0:
        Idc = Vdc_use / float(R_load)
    else:
        Idc = 0.0 

    # -------------------------
    # Powers
    # -------------------------
    P_out = Vdc_use * Idc            # DC side power delivered
    P_in  = P_out / eta              # AC side real power drawn

    # -------------------------
    # Write a few useful I/O
    # -------------------------
    tru_conditions.inputs.Vll_rms_primary          = Vll
    tru_conditions.outputs.ac_voltage_secondary    = Vll_sec
    tru_conditions.outputs.dc_voltage_average      = Vdc_use
    tru_conditions.outputs.dc_current              = Idc
    tru_conditions.outputs.dc_real_power           = P_out * state.ones_row(1)
    tru_conditions.inputs.ac_real_power            = P_in

    # -------------------------
    # Return to network
    # -------------------------
    P_mech = 0.0 * state.ones_row(1)
    P_elec = - P_out * state.ones_row(1)
    P_hydr = 0.0 * state.ones_row(1)
    P_therm= 0.0 * state.ones_row(1)
    stored_results_flag  = True
    stored_modulator_tag = tru.tag

    return P_mech, P_elec, P_hydr, P_therm, stored_results_flag, stored_modulator_tag