# RCAIDE/Library/Methods/Powertrain/Modulators/DC_to_DC_Converter/compute_dcdc_performance.py
# 
# 
# Created:  Sep 2025, M. Guidotti

import numpy as np

def compute_dcdc_performance(DC_to_DC_Converter):

    """
   
    """

    DC_to_DC_Converter.Vin,                 # [V] input DC voltage
    DC_to_DC_Converter.eta,                 # [-] converter efficiency (0<eta<=1)
    DC_to_DC_Converter.Vout_target,         # [V] desired output voltage (use sign for polarity; neg for inverting)
    DC_to_DC_Converter.Rload,               # [ohm] DC load (use either Rload or Iout_set)
    DC_to_DC_Converter.Iout_set,            # [A] desired output current

    """
    Simple averaged CCM model:
      - Buck:        Vout = D * Vin

    Returns a dict with duty, Vout, Iout, Pin/Pout, Iin.
    """

    Vout_sp = float(DC_to_DC_Converter.Vout_target)

    # Vout = D * Vin  -> D = Vout/Vin
    D = Vout_sp / DC_to_DC_Converter.Vin

    # --- Achieved (ideal averaged) Vout from D (useful to report) ---
    Vout = D * DC_to_DC_Converter.Vin
    M = D

    # --- Output load & power ---
    if DC_to_DC_Converter.Iout_set is not None:
        Iout = float(DC_to_DC_Converter.Iout_set)
        R_equiv = np.inf if Iout == 0 else abs(Vout) / abs(Iout)
    else:
        if DC_to_DC_Converter.Rload is None or DC_to_DC_Converter.Rload <= 0:
            raise ValueError("Rload must be >0 if Iout_set not provided")
        R_equiv = float(DC_to_DC_Converter.Rload)
        Iout = Vout / R_equiv

    P_out = Vout * Iout                    # [W] (sign carries polarity; magnitude is power)
    P_in  = P_out / DC_to_DC_Converter.eta # [W] DC input power
    Iin   = P_in / DC_to_DC_Converter.Vin  # [A] input current

    return {
        "Vout_V": Vout,
        "duty": D,
        "gain_M": M,                # Vout/Vin (sign shows polarity)
        "Iout_A": Iout,
        "Pin_W": P_in,
        "Pout_W": P_out,
        "Iin_A": Iin,
        "R_equiv_ohm": R_equiv,
    }