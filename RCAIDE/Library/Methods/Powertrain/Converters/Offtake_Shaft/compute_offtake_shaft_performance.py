# RCAIDE/Library/Methods/Powertrain/Converters/Shaft_Power_Offtake/compute_shaft_power_offtaker.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    

# package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
# compute_offtake_shaft_performance
# ----------------------------------------------------------------------------------------------------------------------     
def compute_offtake_shaft_performance(offtake_shaft,offtake_shaft_conditions, conditions):
    """ This computes the work done from the power draw. The following properties are computed: 
    offtake_shaft.outputs.
      power        (numpy.ndarray): power                              [W]
      work_done    (numpy.ndarray): work done normalized by mass flow  [J/(kg/s)] 

    Assumptions:
        None

    Source:
        https://web.stanford.edu/~cantwell/AA283_Course_Material/AA283_Course_Notes/

    Args:
        offtake_shaft
          .inputs.mdhc                  (numpy.ndarray): Compressor nondimensional mass flow   [unitless] 
          .inputs.reference_temperature (numpy.ndarray): Reference temperature                 [K]
          .inputs.reference_pressure    (numpy.ndarray): Reference pressure                    [Pa]
          .power_draw                           (float): power draw                            [W]

    Returns:
        None 
    """  
    if offtake_shaft.power_draw == 0.0:
        offtake_shaft.outputs.work_done = np.array([0.0]) 
    else: 
        # unpack 
        total_temperature_reference = offtake_shaft_conditions.inputs.total_temperature_reference
        total_pressure_reference    = offtake_shaft_conditions.inputs.total_pressure_reference
        mdhc                        = offtake_shaft_conditions.inputs.mdhc
        Tref                        = offtake_shaft.reference_temperature
        Pref                        = offtake_shaft.reference_pressure
        
        # compute core mass flow rate 
        mdot_core = mdhc * np.sqrt(Tref / total_temperature_reference) * (total_pressure_reference / Pref)

        offtake_shaft_conditions.outputs.power     = offtake_shaft.power_draw
        offtake_shaft_conditions.outputs.work_done = offtake_shaft_conditions.outputs.power / mdot_core  # normalize 
        offtake_shaft_conditions.outputs.work_done[mdot_core == 0] = 0
        
    return 