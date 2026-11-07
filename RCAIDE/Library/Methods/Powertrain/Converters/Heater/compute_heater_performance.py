# RCAIDE/Library/Methods/Powertrain/Converters/Heater/compute_heater_performance.py
#
# Created:  Aug 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  compute_heater_performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_heater_performance(heater, state, network):
    """
    Computes the electrical power drawn by a cryogenic tank's pressure-builder heater.

    The heater's thermal power is read from its tank's ``heater_power`` condition, which
    compute_cryogenic_tank_performance computes. Network.evaluate runs converters before sources, so the
    heater triggers the tank solve first and reads this evaluation's heater power; the later call in the
    sources loop hits the tank's cache.

    Parameters
    ----------
    heater : Heater
        Heater with ``assigned_tank`` set to the tag of its Cryogenic_Tank.
    state : RCAIDE.Framework.Mission.Common.State
        Mission segment state.
    network : RCAIDE.Framework.Networks.Network
        Network holding the heater and its tank.

    Returns
    -------
    inputs, outputs, stored_results_flag, stored_converter_tag
        Standard converter return signature.
    """
    heater_conditions = state.conditions.energy.converters[heater.tag]

    # A heater whose tank was dropped during sizing has no tank conditions and draws no power
    if heater.assigned_tank not in state.conditions.energy.sources:
        thermal_power    = 0.0 * heater_conditions.outputs.power.thermal
        electrical_power = 0.0 * heater_conditions.inputs.power.electrical
    else:
        if network is not None and heater.assigned_tank in network.sources:
            network.sources[heater.assigned_tank].compute_performance(state, network)
        tank_conditions  = state.conditions.energy.sources[heater.assigned_tank]
        # Copied, not aliased: append_segment_conditions zeroes converter outputs in place
        thermal_power    = tank_conditions.heater_power.copy()
        electrical_power = thermal_power / heater.efficiency

    heater_conditions.outputs.power.thermal   = thermal_power
    heater_conditions.inputs.power.electrical = electrical_power

    stored_results_flag  = True
    stored_converter_tag = heater.tag

    return heater_conditions.inputs, heater_conditions.outputs, stored_results_flag, stored_converter_tag
