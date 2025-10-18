# RCAIDE/Library/Methods/Powertrain/Modulators/dc_to_dc_converter/compute_dcdc_performance.py
# 
# 
# Created:  Sep 2025, M. Guidotti

import numpy as np

def compute_dcdc_performance(dc_to_dc_converter):

    """
   
    """

    dc_to_dc_converter.inputs.voltage,            # [V] input DC voltage
    dc_to_dc_converter.inputs.efficiency,         # [-] converter efficiency (0<eta<=1)
    dc_to_dc_converter.inputs.voltage_target,     # [V] desired output voltage (use sign for polarity; neg for inverting)
    dc_to_dc_converter.inputs.resistance,         # [ohm] DC load (use either Rload or Iout_set)
    dc_to_dc_converter.inputs.current,            # [A] desired output current

    Vout_sp = float(dc_to_dc_converter.inputs.voltage_target)
    D       = Vout_sp / dc_to_dc_converter.inputs.voltage # Duty cycle (ideal, continuous conduction mode)

    # --- Achieved (ideal averaged) Vout from D (useful to report) ---
    Vout = D * dc_to_dc_converter.inputs.voltage
    R_equiv = float(dc_to_dc_converter.inputs.resistance)
    Iout = Vout / R_equiv

    P_out = Vout * Iout                              # [W] (sign carries polarity; magnitude is power)
    P_in  = P_out / dc_to_dc_converter.inputs.efficiency    # [W] DC input power
    Iin   = P_in / dc_to_dc_converter.inputs.voltage # [A] input current
 
    dc_to_dc_converter.outputs.voltage    = Vout 
    dc_to_dc_converter.outputs.current    = Iout
    dc_to_dc_converter.inputs.power       = P_in  
    dc_to_dc_converter.outputs.power      = P_out
    dc_to_dc_converter.inputs.current     = Iin
    dc_to_dc_converter.outputs.resistance = R_equiv

    P_mech = 0.0 * np.ones_like(P_out)
    P_elec = P_out
    P_hydr = 0.0 * np.ones_like(P_out)
    P_therm= 0.0 * np.ones_like(P_out)
    stored_results_flag = True
    stored_modulator_tag = dc_to_dc_converter.tag

    return P_mech,P_elec, P_hydr, P_therm, stored_results_flag,stored_modulator_tag