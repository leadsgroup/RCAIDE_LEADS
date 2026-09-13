# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/compute_fuel_mass.py
#
#
# Created:  Jun 2026, M. Clarke

import RCAIDE

# ----------------------------------------------------------------------------------------------------------------------
# compute_fuel_mass
# ----------------------------------------------------------------------------------------------------------------------
def compute_fuel_mass(vehicle, update_fuel_mass = True, update_max_fuel_mass=True):

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
    
    max_fuel_tank_mass = 0
    fuel_mass          = 0
    for network in vehicle.networks:
        for source in network.sources:
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
                max_fuel_tank_mass += source.volume_properties.net_volume * source.fuel.density
                # Skip if already explicitly set (partial load) -- same "!= 0" convention as
                # append_cryogenic_tank_conditions.py.
                if update_fuel_mass and source.fuel.mass_properties.mass == 0:
                    source.fuel.mass_properties.mass = source.volume_properties.net_volume * source.fuel.density
                fuel_mass += source.fuel.mass_properties.mass
                
    # Assign Total Fuel Volume and to Vehicle 
    if update_max_fuel_mass:
        vehicle.mass_properties.max_fuel   = max_fuel_tank_mass 
    
    if update_fuel_mass:
        vehicle.mass_properties.fuel = fuel_mass

    return 
 