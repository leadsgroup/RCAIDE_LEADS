# RCAIDE/Library/Methods/Powertrain/Converters/Pump/compute_pump_performance.py
#
# 
# Created:  Sep. 2025, M. Guidotti

def compute_pump_performance(pump, line, conditions):

    pump_conditions = conditions.converters[pump.tag]
    pump_conditions.inputs.p_in = line.pressure
    pump_conditions.outputs.p_out = line.pressure + pump.delta_p
    pump_conditions.outputs.power = pump.mass_flow_rate * pump.delta_p / (line.density * pump.efficiency)

    return