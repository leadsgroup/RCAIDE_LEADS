# RCAIDE/Library/Methods/Powertrain/Modulators/tru/compute_tru_performance.py
# 
# 
# Created:  Sep 2025, M. Guidotti

import numpy as np

def compute_tru_performance(tru):

    """
   
    """

    # Inputs
    Vll_rms_primary = tru.inputs.Vll_rms_primary  # [V_rms] line-line AC primary voltage
    turns_ratio     = tru.inputs.turns_ratio      # [-] transformer turns ratio (N_primary / N_secondary)
    diode_drop      = tru.inputs.diode_drop       # [V] average forward drop per diode
    eta_tru         = tru.inputs.eta_tru          # [-] transformer-rectifier efficiency (0<eta<=1)
    pf_assumed      = tru.inputs.pf_assumed       # [-] assumed power factor at AC input (for sizing)
    R_load          = tru.inputs.R_load           # [ohm] DC load (use either R_load or Idc_set)
    Idc_set         = tru.inputs.Idc_set          # [A] desired DC current (use either R_load or Idc_set)

    # Secondary AC (after transformer)
    Vll_rms_sec = Vll_rms_primary / turns_ratio  # [V_rms] line-line at secondary

    # Ideal 6-pulse diode bridge average DC (no drops)
    Vdc_ideal = 1.35 * Vll_rms_sec                      # [V] 6-pulse rectifier formula

    # Account for diode drops (two diodes conduct in series)
    V_drop_bridge = 2.0 * diode_drop                # [V] average series drop
    Vdc_no_load = max(Vdc_ideal - V_drop_bridge, 0.0)   # [V] open-circuit DC

    # Load condition
    if R_load is not None and Idc_set is not None:
        raise ValueError("Specify either R_load or Idc_set, not both")
    elif R_load is not None:
        if R_load <= 0:
            raise ValueError("R_load must be positive")
        Idc = Vdc_no_load / R_load                  # [A] DC current for resistive load
    elif Idc_set is not None:
        if Idc_set < 0:
            raise ValueError("Idc_set must be non-negative")
        Idc = Idc_set                               # [A] DC current setpoint
    else:
        # Default: assume load drawing ~10 A at no-load voltage
        R_load = max(Vdc_no_load / 10.0, 1e-3)
        Idc = Vdc_no_load / R_load                      # [A]

    # DC voltage under load (simplified; assumes minimal regulation effects)
    Vdc = max(Vdc_no_load - 0.01 * Idc, 0.0)  # [V] small drop proportional to current (approximation)

    # Output power
    P_out = Vdc * Idc                           # [W] DC real power to load

    # Input power and current
    P_in = P_out / eta_tru                  # [W] AC real power drawn
    S_in = P_in / pf_assumed                # [VA] input apparent power
    I_line_rms = S_in / (np.sqrt(3.0) * Vll_rms_primary)  # [A_rms] per-line RMS current

    tru.outputs.Vll_rms_secondary = Vll_rms_sec       # [V_rms] AC secondary line-line
    tru.outputs.Vdc_ideal = Vdc_ideal                 # [V] ideal no-drop DC
    tru.outputs.Vdc_no_load = Vdc_no_load             # [V] minus diode drops
    tru.outputs.Vdc = Vdc                             # [V] averaged DC at load
    tru.outputs.Idc = Idc                             # [A] DC current
    tru.outputs.P_out_W = P_out                       # [W] DC real power delivered
    tru.outputs.P_in_W = P_in                         # [W] AC real power drawn
    tru.outputs.S_in_VA = S_in                        # [VA] input apparent power
    tru.outputs.I_line_rms_A = I_line_rms             # [A_rms] input line current per phase

    return