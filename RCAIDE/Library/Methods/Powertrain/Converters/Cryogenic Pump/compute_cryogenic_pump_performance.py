# RCAIDE/Library/Methods/Powertrain/Converters/Pump/compute_pump_performance.py
#
# 
# Created:  Sep. 2025, M. Guidotti

def compute_cryogenic_pump_performance(pump, line, state):

    pump_conditions = state.conditions.converters[pump.tag]

    # === Pump Fundamental Equations (from guiding theory) ===
    # Eq (3.2): Volumetric flow rate from mass flow
    Q = pump.mass_flow_rate / line.density
    pump_conditions.outputs.volumetric_flow_rate = Q

    # Eq (3.3a): Hydraulic output power
    W_out = pump.delta_pressure * Q
    pump_conditions.outputs.hydraulic_power_out = W_out

    # Eq (3.3b): Input mechanical power (if torque & omega available)
    if hasattr(pump, 'torque') and hasattr(pump, 'omega'):
        W_in = pump.torque * pump.omega
        pump_conditions.outputs.mechanical_power_in = W_in
    else:
        W_in = None

    # Eq (3.4): Ideal displacement pump flow (if displacement & omega exist)
    if hasattr(pump, 'displacement') and hasattr(pump, 'omega'):
        Q_disp = pump.displacement * pump.omega
        pump_conditions.outputs.ideal_displacement_flow = Q_disp

    # Eq (3.5): Torque for ideal displacement pump (if displacement exists)
    if hasattr(pump, 'displacement'):
        T_required = pump.displacement * pump.delta_pressure
        pump_conditions.outputs.ideal_required_torque = T_required

    # Eq (3.6): Theoretical piston pump displacement (if n pistons & piston volume available)
    if hasattr(pump, 'num_pistons') and hasattr(pump, 'piston_volume') and hasattr(pump, 'omega'):
        Q_th = pump.num_pistons * pump.omega * pump.piston_volume
        pump_conditions.outputs.theoretical_piston_flow = Q_th

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