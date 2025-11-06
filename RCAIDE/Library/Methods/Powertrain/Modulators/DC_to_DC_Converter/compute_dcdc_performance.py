# RCAIDE/Library/Methods/Powertrain/Modulators/dc_to_dc_converter/compute_dcdc_performance.py
# 
# 
# Created:  Sep 2025, M. Guidotti

import numpy as np

def compute_dcdc_performance(dc_to_dc_converter, state):

    """
   
    """

    dc_to_dc_converter_conditions = state.conditions.energy.modulators[dc_to_dc_converter.tag]

    dc_to_dc_converter_conditions.inputs.voltage,            # [V] input DC voltage
    dc_to_dc_converter_conditions.inputs.efficiency,         # [-] converter efficiency (0<eta<=1)
    dc_to_dc_converter_conditions.inputs.voltage_target,     # [V] desired output voltage (use sign for polarity; neg for inverting)
    dc_to_dc_converter_conditions.inputs.resistance,         # [ohm] DC load (use either Rload or Iout_set)
    dc_to_dc_converter_conditions.inputs.current,            # [A] desired output current

    Vout_sp = float(dc_to_dc_converter_conditions.inputs.voltage_target)
    D       = Vout_sp / dc_to_dc_converter_conditions.inputs.voltage # Duty cycle (ideal, continuous conduction mode)

    # --- Achieved (ideal averaged) Vout from D (useful to report) ---
    Vout = D * dc_to_dc_converter_conditions.inputs.voltage
    R_equiv = float(dc_to_dc_converter_conditions.inputs.resistance)
    Iout = Vout / R_equiv

    P_out = Vout * Iout                              # [W] (sign carries polarity; magnitude is power)
    P_in  = P_out / dc_to_dc_converter_conditions.inputs.efficiency    # [W] DC input power
    Iin   = P_in / dc_to_dc_converter_conditions.inputs.voltage # [A] input current
 
    dc_to_dc_converter_conditions.outputs.voltage    = Vout 
    dc_to_dc_converter_conditions.outputs.current    = Iout
    dc_to_dc_converter_conditions.inputs.power.electrical       = P_in  
    dc_to_dc_converter_conditions.outputs.power.electrical      = P_out
    dc_to_dc_converter_conditions.inputs.current     = Iin
    dc_to_dc_converter_conditions.outputs.resistance = R_equiv

    stored_results_flag            = True
    stored_modulator_tag           = dc_to_dc_converter.tag  

    return  dc_to_dc_converter_conditions.inputs, dc_to_dc_converter_conditions.outputs, stored_results_flag, stored_modulator_tag