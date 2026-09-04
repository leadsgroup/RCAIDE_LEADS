# RCAIDE/Library/Methods/Geometry/Planform/compute_fuel_volume.py
# 
# 
# Created:  Jul 2024, M. Clarke 
# Modified: Aug 2025, S. Shekar 
# ---------------------------------------------------------------------------------------
# Imports
# ---------------------------------------------------------------------------------------
import RCAIDE
import warnings

# ----------------------------------------------------------------------------------------------------------------------
# _tanks_in_pack_order
# ----------------------------------------------------------------------------------------------------------------------
def _tanks_in_pack_order(fuel_tanks):
    """Reorders fuel tanks so a packs_after_tank dependency is always computed
    before its dependent, regardless of container insertion order (see
    compute_wing_non_integral_tank_volume, which reads the dependency's
    already-computed tank_percent_span_location)."""
    by_tag   = {t.tag: t for t in fuel_tanks}
    ordered  = []
    placed   = set()
    visiting = set()

    def place(tank):
        if tank.tag in placed:
            return
        if tank.tag in visiting:
            warnings.warn(f"Fuel tank '{tank.tag}' has a circular packs_after_tank "
                           f"chain; processing in container order instead.", stacklevel=2)
            return
        visiting.add(tank.tag)
        dep_tag = tank.packs_after_tank
        if dep_tag is not None and dep_tag in by_tag:
            place(by_tag[dep_tag])
        visiting.discard(tank.tag)
        if tank.tag not in placed:
            placed.add(tank.tag)
            ordered.append(tank)

    for tank in fuel_tanks:
        place(tank)
    return ordered

# ----------------------------------------------------------------------------------------------------------------------
# compute_fuel_volume
# ----------------------------------------------------------------------------------------------------------------------
def compute_fuel_volume(vehicle, compute_fuel_volume = True, update_max_fuel = False):
    """
    Computes the total fuel volume and mass for all fuel tanks in a vehicle.

    This function iterates through all networks, fuel lines, and fuel tanks in the vehicle
    to calculate the total fuel volume and mass. It updates each fuel tank's internal volume
    and sets the vehicle's total fuel volume attribute.

    Parameters
    ----------
    vehicle : Vehicle
        The vehicle object containing networks, wings, and fuselages
            - networks : list
                Collection of propulsion networks containing fuel lines
            - wings : list
                Collection of wing objects for volume calculations
            - fuselages : list
                Collection of fuselage objects for volume calculations

    Returns
    -------
    None
        Function modifies the vehicle object in-place by setting total_fuel_volume

    Notes
    -----
    This function performs the following operations:
        1. Initializes total fuel volume and mass counters
        2. Iterates through all propulsion networks in the vehicle
        3. For each network, iterates through all fuel lines
        4. For each fuel line, iterates through all fuel tanks
        5. Resets each fuel tank's internal volume to zero (This is updated by the compute fuel volume function)
        6. Calls the fuel tank's compute_volume method with wings and fuselages
        7. Accumulates the volume and mass contributions
        8. Sets the vehicle's total_fuel_volume attribute

    **Major Assumptions**
        * All fuel tanks have a compute_volume method that accepts wings and fuselages
        * Fuel tanks have mass_properties.fuel attribute for fuel mass
        * Vehicle object has networks, wings, and fuselages attributes

    **Definitions**

    'Fuel Line'
        A collection of fuel tanks that are connected in series within a propulsion network

    'Fuel Tank'
        A container that stores fuel and has methods to compute its volume based on vehicle geometry

    'Internal Volume'
        The calculated volume of a fuel tank based on its geometry and position within the vehicle

    See Also
    --------
    Vehicle : RCAIDE.Vehicle
    """
    wings             = vehicle.wings
    fuselages         = vehicle.fuselages
    total_fuel_volume = 0
    for network in vehicle.networks:
         distributor_tanks = {}  # distributor tag -> list of fuel tanks assigned to it, this network only
         fuel_tanks_in_network = [source for source in network.sources
                                   if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank)]
         for fuel_tank in _tanks_in_pack_order(fuel_tanks_in_network):
                fuel_tank.fuel.tag = fuel_tank.tag + '_' + fuel_tank.fuel.tag
                if compute_fuel_volume:
                    fuel_tank.compute_volume(wings, fuselages,  network.sources)
                total_fuel_volume += fuel_tank.volume_properties.net_volume
                if fuel_tank.assigned_distributors:
                    distributor_tag = fuel_tank.assigned_distributors[0][0]
                    distributor_tanks.setdefault(distributor_tag, []).append(fuel_tank)

         # Multiple tanks can be assigned to the same distributor (fuel line), each
         # independently drawing down at power_split_ratio * that distributor's
         # chemical power demand (see compute_fuel_tank_performance). Left at the
         # Source class default of 1.0 for every tank, this makes every tank on a
         # shared line behave as if it alone supplies the full demand -- smaller
         # tanks on the same line run dry mid-mission while larger ones barely
         # deplete. Set each tank's share proportional to its sized net volume so
         # tanks sharing a line draw down proportionally to their capacity instead.
         #
         # Deliberately NOT gated on the compute_fuel_volume flag: this function is
         # called repeatedly through the vehicle's weight/geometry convergence loop,
         # and later calls pass compute_fuel_volume=False to reuse already-sized
         # volumes without re-running the (expensive, iterative) geometry solve.
         # Net volumes can still be unequal placeholders on an early pass (e.g.
         # before wing-derived sizing constraints are applied) and only settle to
         # their final values on a later pass; gating this block the same way left
         # power_split_ratio permanently locked at whatever an early, not-yet-final
         # snapshot produced (confirmed: every tank ended up at exactly 1/N,
         # consistent with them still reading as equal at that early pass).
         # Recomputing from whatever net_volume currently holds on every call keeps
         # it consistent with the final sizing without needing to know which call is
         # "the last one."
         for tanks in distributor_tanks.values():
             total_volume = sum(t.volume_properties.net_volume for t in tanks)
             if total_volume > 0:
                 for t in tanks:
                     t.power_split_ratio = t.volume_properties.net_volume / total_volume

    if compute_fuel_volume:
        vehicle.volume_properties.max_fuel = total_fuel_volume

    if update_max_fuel:
        total_fuel_mass = sum(
            source.volume_properties.net_volume * source.fuel.density
            for network in vehicle.networks
            for source in network.sources
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank)
        )
        vehicle.mass_properties.max_fuel = total_fuel_mass

    return