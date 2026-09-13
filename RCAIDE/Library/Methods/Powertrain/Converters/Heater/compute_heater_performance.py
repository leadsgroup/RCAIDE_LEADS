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

    Ordering / staleness note
    --------------------------
    ``RCAIDE.Framework.Networks.Network.evaluate`` runs converters *before*
    sources within a single call (propulsors -> converters -> sources ->
    distributors), so the ``heater_power`` read here is whatever the tank's
    source computed on the *previous* call to ``evaluate`` (mutated in place on
    the same persistent ``state``, not the value about to be computed this call.
    Pump has an analogous ordering dependency in the other direction (it needs
    propulsors, which run first, to have already populated the fuel line's
    hydraulic power -- satisfied within the same call). This heater's dependency
    is not satisfiable within a single call given the current evaluation order,
    so it is instead a one-call-lagged (Gauss-Seidel-style) coupling: as the
    outer network solver (``RCAIDE.Library.Mission.Common.Update.network``,
    fsolve/least_squares) converges, successive ``evaluate()`` calls stop
    changing state, and the lag vanishes at convergence. It is not exact within
    a single call and should not be relied on for anything requiring intra-call
    consistency. On the very first call of a segment, ``heater_power`` reads at
    its initialized value of 0 (from ``append_fuel_tank_conditions``/
    ``append_cryogenic_tank_conditions``), so the electrical bus sees no heater
    load until the tank source has evaluated at least once.

    Parameters
    ----------
    heater : Heater
        Heater component being analyzed, with ``assigned_tank`` set to the tag
        of the ``Cryogenic_Tank`` source whose pressure-builder duty it supplies.
    state : RCAIDE.Framework.Mission.Common.State
        Mission segment state.
    network : RCAIDE.Framework.Networks.Network
        The network this heater belongs to (unused directly here, kept for
        signature parity with other converters' ``compute_performance``).

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
        tank_conditions  = state.conditions.energy.sources[heater.assigned_tank]
        thermal_power    = tank_conditions.heater_power
        electrical_power = thermal_power / heater.efficiency

    heater_conditions.outputs.power.thermal   = thermal_power
    heater_conditions.inputs.power.electrical = electrical_power

    stored_results_flag  = True
    stored_converter_tag = heater.tag

    return heater_conditions.inputs, heater_conditions.outputs, stored_results_flag, stored_converter_tag
