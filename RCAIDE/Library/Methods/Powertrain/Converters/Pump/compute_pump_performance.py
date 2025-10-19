# RCAIDE/Library/Methods/Powertrain/Converters/Pump/compute_pump_performance.py
#
# 
# Created:  Sep. 2025, M. Guidotti

def compute_pump_performance(pump, line, state):

    pump_conditions = state.conditions.converters[pump.tag]
    pump_conditions.inputs.pressure = line.pressure
    pump_conditions.outputs.pressure = line.pressure + pump.delta_pressure
    pump_conditions.outputs.power = pump.mass_flow_rate * pump.delta_pressure / (line.density * pump.efficiency)

    stored_results_flag            = True
    stored_converter_tag           = pump.tag  

    pump_conditions.power.propulsive               = 0.0 * state.ones_row(1)
    pump_conditions.power.mechanical               = 0.0 * state.ones_row(1)
    pump_conditions.power.electrical               = pump_conditions.inputs.power 
    pump_conditions.power.chemical                 = 0.0 * state.ones_row(1)
    pump_conditions.power.pneumatic                = 0.0 * state.ones_row(1)
    pump_conditions.power.hydraulic                = pump_conditions.outputs.power 
    pump_conditions.power.thermal                  = 0.0 * state.ones_row(1)

    return  pump_conditions.power, stored_results_flag, stored_converter_tag