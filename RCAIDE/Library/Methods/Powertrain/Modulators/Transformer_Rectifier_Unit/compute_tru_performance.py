# RCAIDE/Library/Methods/Powertrain/Modulators/Transformer_Rectifier_Unit/compute_tru_performance.py
# 
# 
# Created:  Sep 2025, M. Guidotti

import numpy as np

def compute_tru_performance(Transformer_Rectifier_Unit):

    """
   
    """

    # Secondary AC (after transformer)
    Vll_rms_sec = Transformer_Rectifier_Unit.Vll_rms_primary / Transformer_Rectifier_Unit.turns_ratio  # [V_rms] line-line at secondary

    # Ideal 6-pulse diode bridge average DC (no drops)
    Vdc_ideal = 1.35 * Vll_rms_sec                      # [V] 6-pulse rectifier formula

    # Account for diode drops (two diodes conduct in series)
    V_drop_bridge = 2.0 * Transformer_Rectifier_Unit.diode_drop                # [V] average series drop
    Vdc_no_load = max(Vdc_ideal - V_drop_bridge, 0.0)   # [V] open-circuit DC

    # Load condition
    if Transformer_Rectifier_Unit.R_load is not None and Transformer_Rectifier_Unit.Idc_set is not None:
        raise ValueError("Specify either R_load or Idc_set, not both")
    elif Transformer_Rectifier_Unit.R_load is not None:
        if Transformer_Rectifier_Unit.R_load <= 0:
            raise ValueError("R_load must be positive")
        Idc = Vdc_no_load / Transformer_Rectifier_Unit.R_load                  # [A] DC current for resistive load
    elif Transformer_Rectifier_Unit.Idc_set is not None:
        if Transformer_Rectifier_Unit.Idc_set < 0:
            raise ValueError("Idc_set must be non-negative")
        Idc = Transformer_Rectifier_Unit.Idc_set                               # [A] DC current setpoint
    else:
        # Default: assume load drawing ~10 A at no-load voltage
        R_load = max(Vdc_no_load / 10.0, 1e-3)
        Idc = Vdc_no_load / R_load                      # [A]

    # DC voltage under load (simplified; assumes minimal regulation effects)
    Vdc = max(Vdc_no_load - 0.01 * Idc, 0.0)  # [V] small drop proportional to current (approximation)

    # Output power
    P_out = Vdc * Idc                           # [W] DC real power to load

    # Input power and current
    P_in = P_out / Transformer_Rectifier_Unit.eta_Transformer_Rectifier_Unit                  # [W] AC real power drawn
    S_in = P_in / Transformer_Rectifier_Unit.pf_assumed                # [VA] input apparent power
    I_line_rms = S_in / (np.sqrt(3.0) * Transformer_Rectifier_Unit.Vll_rms_primary)  # [A_rms] per-line RMS current

    # Compile results
    result = {
        "Vll_rms_secondary": Vll_rms_sec,       # [V_rms] AC secondary line-line
        "Vdc_ideal": Vdc_ideal,                 # [V] ideal no-drop DC
        "Vdc_no_load": Vdc_no_load,             # [V] minus diode drops
        "Vdc": Vdc,                             # [V] averaged DC at load
        "Idc": Idc,                             # [A] DC current
        "P_out_W": P_out,                       # [W] DC real power delivered
        "P_in_W": P_in,                         # [W] AC real power drawn
        "S_in_VA": S_in,                        # [VA] input apparent power
        "I_line_rms_A": I_line_rms,             # [A_rms] input line current per phase
    }

    return result