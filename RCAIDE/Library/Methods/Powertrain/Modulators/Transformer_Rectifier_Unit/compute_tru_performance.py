# RCAIDE/Library/Methods/Powertrain/Modulators/tru/compute_tru_performance.py
# 
# 
# Created:  Sep 2025, M. Guidotti

import numpy as np

def compute_tru_performance(tru, network, state):

    """
   
    """
    conditions     = state.conditions
    tru_conditions = conditions.energy.modulators[tru.tag]

    # -------------------------------------------------
    # Read buses (AC power in, AC LL voltage, DC bus V)
    # -------------------------------------------------
    P_in = 0.0 * state.ones_row(1)   # [W] AC-side real power drawn
    Vll  = 0.0                       # [V_rms] AC line-line
    Vdc  = 0.0                       # [V] DC bus

    for distributor_tag in tru.assigned_distributors[0]:
        dist = network.distributors[distributor_tag]
        if dist.bus_type == 'AC':
            P_in += conditions.energy.distributors[distributor_tag].net_electrical_power
            Vll   = dist.voltage_phase_to_phase
        elif dist.bus_type == 'DC':
            Vdc   = dist.voltage

    # -------------------------------------------------
    # TRU parameters (keep it simple)
    # -------------------------------------------------
    eta        = tru.electrical_efficiency
    turns      = tru.turns_ratio
    diode_drop = tru.diode_drop
    R_load     = tru.R_load
    Idc_set    = tru.Idc_set

    # -------------------------------------------------
    # Voltages (estimate DC if no DC bus voltage given)
    # Vdc_open ≈ 1.35 * (Vll/turns) - 2*diode_drop
    # -------------------------------------------------
    Vll_sec  = Vll / turns
    Vdc_open = max(1.35 * Vll_sec - 2.0 * diode_drop, 0.0)
    if Vdc > 0.0:
        Vdc_use = Vdc
    else:
        Vdc_use = Vdc_open

    # -------------------------------------------------
    # Output power from input power
    # -------------------------------------------------
    P_out = eta * P_in                      # [W] DC-side power delivered

    # -------------------------------------------------
    # DC current choice
    # -------------------------------------------------
    if Idc_set is not None:
        Idc = Idc_set * state.ones_row(1)
    elif (R_load is not None) and (R_load > 0.0):
        Idc = Vdc_use / R_load
    else:
        # infer from power balance if no load/current specified
        Idc = P_out / Vdc_use if Vdc_use != 0.0 else 0.0 * state.ones_row(1)

    # -------------------------------------------------
    # Record I/O
    # -------------------------------------------------
    tru_conditions.inputs.Vll_rms_primary       = Vll
    tru_conditions.outputs.ac_voltage_secondary = Vll_sec
    tru_conditions.outputs.dc_voltage_average   = Vdc_use
    tru_conditions.outputs.dc_current           = Idc
    tru_conditions.outputs.dc_real_power        = P_out
    tru_conditions.inputs.ac_real_power         = P_in

    # -------------------------------------------------
    # Return powers to the network evaluator
    # (source on DC side → negative sign for P_elec)
    # -------------------------------------------------
    P_mech = 0.0 * state.ones_row(1)
    P_elec = - P_out
    P_hydr = 0.0 * state.ones_row(1)
    P_therm= 0.0 * state.ones_row(1)
    stored_results_flag  = True
    stored_modulator_tag = tru.tag

    return P_mech, P_elec, P_hydr, P_therm, stored_results_flag, stored_modulator_tag