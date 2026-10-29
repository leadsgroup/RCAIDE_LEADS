# RCAIDE/Library/Methods/Powertrain/Converters/Pump/compute_pump_performance.py
#
#
# Created:  Sep. 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE

# ----------------------------------------------------------------------------------------------------------------------
#  compute_pump_performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_pump_performance(pump, state, network):
    """
    Computes the performance of an electrically-driven boost pump.

    The hydraulic power is the fuel line's flow-driven demand plus the pressure
    rise. The line demand comes from its own distribution-loss calculation
    (RCAIDE.Library.Methods.Powertrain.Distributors.Fuel_Line.
    compute_fuel_line_distribution_losses), which has already accumulated the
    pipe-friction power and the fuel mass flow of every propulsor assigned to
    the line (propulsors are evaluated before converters, so both are
    populated by the time the pump runs). The pressure rise from
    design_inlet_pressure to design_outlet_pressure costs the line's volume
    flow times that rise. The pump takes its distributor_split share of the
    total, converts it to shaft power via pump.efficiency, then to electrical
    power via pump.turbine_efficiency, and draws that from its assigned bus.
    """
    pump_conditions = state.conditions.energy.converters[pump.tag]

    # Resolve the fuel line (chemical flow this pump moves) and the electrical
    # bus (its actual power source) from among this pump's assigned distributors
    fuel_line = None
    for d_tag in pump.assigned_distributors[0]:
        distributor = network.distributors[d_tag]
        if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
            fuel_line = distributor
    fuel_line_conditions = state.conditions.energy.distributors[fuel_line.tag]

    # Pressure rise across the pump: volume flow of the line times (outlet - inlet) pressure
    volumetric_flow_rate  = fuel_line_conditions.mass_flow_rate / fuel_line.working_fluid.density
    pressure_rise_power   = volumetric_flow_rate * (pump.design_outlet_pressure - pump.design_inlet_pressure)

    # This pump's share of the line's flow-driven power demand plus the pressure rise
    hydraulic_power = pump.distributor_split * (fuel_line_conditions.inputs.power.hydraulic + pressure_rise_power)

    # hydraulic -> shaft, via pump efficiency
    shaft_power      = hydraulic_power / pump.efficiency
    # shaft -> electrical, via motor/drive efficiency
    electrical_power = shaft_power / pump.turbine_efficiency

    pump_conditions.inputs.p_in              = pump.design_inlet_pressure  * state.ones_row(1)
    pump_conditions.outputs.p_out            = pump.design_outlet_pressure * state.ones_row(1)
    pump_conditions.outputs.power.hydraulic  = hydraulic_power
    pump_conditions.outputs.power.mechanical = shaft_power
    pump_conditions.inputs.power.electrical  = electrical_power

    stored_results_flag   = True
    stored_converter_tag  = pump.tag

    return pump_conditions.inputs, pump_conditions.outputs, stored_results_flag, stored_converter_tag
