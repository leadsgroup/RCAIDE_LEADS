# RCAIDE/Library/Methods/Powertrain/Converters/Heater/compute_heater_performance.py
#
# Created:  Aug 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  compute_heater_performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_heater_performance(heater, state, network):
    """
    Computes the electrical power draw of a cryogenic tank's pressure-builder heater.

    The heater's thermal power requirement is not recomputed here -- it is read
    directly from its assigned tank's own already-computed boil-off physics
    (``RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.
    compute_cryogenic_tank_performance``), which stores the heater's required
    thermal duty in that tank's own ``heater_power`` condition at every mission
    point. This mirrors ``compute_pump_performance``, which likewise reads an
    already-computed demand (there, the fuel line's hydraulic power) rather than
    deriving it independently.

    Ordering note
    -------------
    ``RCAIDE.Framework.Networks.Network.evaluate`` runs converters before
    sources, so the assigned tank's solve is triggered here first (it is cached
    on its inputs, so the later call in the sources loop costs nothing). The
    heater therefore reads this call's ``heater_power``, not the previous
    call's, which keeps the electrical-power residual consistent for the
    mission solver.

    Parameters
    ----------
    heater : Heater
        Heater component being analyzed, with ``assigned_tank`` set to the tag
        of the ``Cryogenic_Tank`` source whose pressure-builder duty it supplies.
    state : RCAIDE.Framework.Mission.Common.State
        Mission segment state.
    network : RCAIDE.Framework.Networks.Network
        The network this heater belongs to (holds its assigned tank).

    Returns
    -------
    inputs, outputs, stored_results_flag, stored_converter_tag
        Standard converter return signature (see ``Pump.compute_performance``).
    """
    heater_conditions = state.conditions.energy.converters[heater.tag]

    # Defensive: the tank a heater is wired to in a vehicle-setup file is fixed
    # by tag, but tank geometry sizing (or per-config resizing/deepcopy) can
    # drop an infeasible tank from network.sources entirely, in which case its
    # condition container is never created. Rather than let a stale tag
    # reference crash the whole mission solve, a heater with no matching tank
    # in this particular state simply draws no power -- there is nothing left
    # for it to condition.
    if heater.assigned_tank not in state.conditions.energy.sources:
        thermal_power    = 0.0 * heater_conditions.outputs.power.thermal
        electrical_power = 0.0 * heater_conditions.inputs.power.electrical
    else:
        if network is not None and heater.assigned_tank in network.sources:
            network.sources[heater.assigned_tank].compute_performance(state, network)
        tank_conditions  = state.conditions.energy.sources[heater.assigned_tank]
        # Copied, not aliased: append_segment_conditions zeroes converter outputs in place, which would
        # otherwise also zero the tank's heater_power (and a cached tank solve never rewrites it).
        thermal_power    = tank_conditions.heater_power.copy()
        electrical_power = thermal_power / heater.efficiency

    heater_conditions.outputs.power.thermal   = thermal_power
    heater_conditions.inputs.power.electrical = electrical_power

    stored_results_flag  = True
    stored_converter_tag = heater.tag

    return heater_conditions.inputs, heater_conditions.outputs, stored_results_flag, stored_converter_tag
