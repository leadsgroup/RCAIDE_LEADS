# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/distribute_fuel_across_tanks.py
#
# Created: Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------
def distribute_fuel_across_tanks(tanks, total_fuel_mass):
    """
    Splits a requested total fuel mass across several tanks, proportional to
    each tank's own capacity, and writes the result into each tank's
    ``fuel.mass_properties.mass``.

    This is the multi-tank counterpart to setting a single tank's
    ``fuel.mass_properties.mass`` directly for a partial load: for a cryogenic
    tank, that field is read as the initial fuel charge by
    ``append_cryogenic_tank_conditions`` (whenever it's non-zero) instead of
    the design-basis full-tank default -- see that function's docstring for
    the ``!= 0`` sentinel convention this relies on. Call this before
    ``evaluate()`` is invoked, same as setting the field directly.

    Parameters
    ----------
    tanks : list of Fuel_Tank
        Tanks to distribute the fuel across (e.g. every tank on one fuel
        line/network). Each tank's capacity is
        ``tank.volume_properties.net_volume * tank.fuel.density``.
    total_fuel_mass : float
        Total fuel mass [kg] to split across the tanks.

    Raises
    ------
    ValueError
        If ``total_fuel_mass`` exceeds the tanks' combined capacity.
    """
    capacities     = [tank.volume_properties.net_volume * tank.fuel.density for tank in tanks]
    total_capacity = sum(capacities)

    if total_fuel_mass > total_capacity * (1 + 1e-6):
        raise ValueError(
            f"Requested total fuel mass ({total_fuel_mass:.1f} kg) exceeds the "
            f"combined capacity of the given tanks ({total_capacity:.1f} kg)."
        )

    for tank, capacity in zip(tanks, capacities):
        tank.fuel.mass_properties.mass = total_fuel_mass * (capacity / total_capacity)

    return
