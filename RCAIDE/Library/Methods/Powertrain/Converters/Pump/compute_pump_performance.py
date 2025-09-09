# RCAIDE/Library/Methods/Powertrain/Converters/Pump/compute_pump_performance.py
#
# 
# Created:  Sep. 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import  RCAIDE
from RCAIDE.Framework.Core import  Units

def compute_pump_performance(pump, line, conditions):

    p_in  = line.pressure
    p_out = line.pressure + pump.delta_p
    power = pump.mass_flow_rate * pump.delta_p / (line.density * pump.efficiency)

    return