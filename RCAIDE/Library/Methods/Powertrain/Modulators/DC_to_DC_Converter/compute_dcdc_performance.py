# RCAIDE/Library/Methods/Powertrain/Modulators/dc_to_dc_converter/compute_dcdc_performance.py
# 
# 
# Created:  Sep 2025, M. Guidotti

import numpy as np

def compute_dcdc_performance(dc_to_dc_converter):

    """
   
    """

    dc_to_dc_converter.inputs.Vin,                 # [V] input DC voltage
    dc_to_dc_converter.inputs.eta,                 # [-] converter efficiency (0<eta<=1)
    dc_to_dc_converter.inputs.Vout_target,         # [V] desired output voltage (use sign for polarity; neg for inverting)
    dc_to_dc_converter.inputs.Rload,               # [ohm] DC load (use either Rload or Iout_set)
    dc_to_dc_converter.inputs.Iout_set,            # [A] desired output current

    """
    Simple averaged CCM model:
      - Buck:        Vout = D * Vin

    Returns a dict with duty, Vout, Iout, Pin/Pout, Iin.
    """

    Vout_sp = float(dc_to_dc_converter.Vout_target)

    # Vout = D * Vin  -> D = Vout/Vin
    D = Vout_sp / dc_to_dc_converter.Vin

    # --- Achieved (ideal averaged) Vout from D (useful to report) ---
    Vout = D * dc_to_dc_converter.Vin
    M = D

    # --- Output load & power ---
    if dc_to_dc_converter.Iout_set is not None:
        Iout = float(dc_to_dc_converter.Iout_set)
        R_equiv = np.inf if Iout == 0 else abs(Vout) / abs(Iout)
    else:
        if dc_to_dc_converter.Rload is None or dc_to_dc_converter.Rload <= 0:
            raise ValueError("Rload must be >0 if Iout_set not provided")
        R_equiv = float(dc_to_dc_converter.Rload)
        Iout = Vout / R_equiv

    P_out = Vout * Iout                    # [W] (sign carries polarity; magnitude is power)
    P_in  = P_out / dc_to_dc_converter.eta # [W] DC input power
    Iin   = P_in / dc_to_dc_converter.Vin  # [A] input current
 
    dc_to_dc_converter.outputs.Vout = Vout 
    dc_to_dc_converter.outputs.duty = D
    dc_to_dc_converter.outputs.Iout = Iout
    dc_to_dc_converter.outputs.Pin  = P_in  
    dc_to_dc_converter.outputs.Pout = P_out
    dc_to_dc_converter.outputs.Iin  = Iin
    dc_to_dc_converter.outputs.R_equiv = R_equiv

    return 