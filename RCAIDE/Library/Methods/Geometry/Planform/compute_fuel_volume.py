# RCAIDE/Library/Methods/Geometry/Planform/compute_fuel_volume.py
# 
# 
# Created:  Jul 2024, M. Clarke 
# Modified: Aug 2025, S. Shekar 
import RCAIDE
# ----------------------------------------------------------------------------------------------------------------------
# compute_fuel_volume 
# ----------------------------------------------------------------------------------------------------------------------
def compute_fuel_volume(vehicle, overwrite_fuel_volume = False):
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
    total_fuel_mass   = 0
    for network in vehicle.networks: 
        for fuel_tank in network.sources:
            if isinstance(fuel_tank, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
                try:
                    compute_fuel_tank_volume = fuel_tank.compute_volume
                except Exception as e:
                    total_fuel_volume += getattr(fuel_tank.fuel.volume_properties, "net_volume", None)
                    total_fuel_mass   += getattr(fuel_tank.fuel.mass_properties, "mass", None)
                else:
                    # if no error getting the method, run it normally
                    if overwrite_fuel_volume:
                        compute_fuel_tank_volume(wings, fuselages) 
                        fuel_tank.fuel.volume_properties.net_volume = fuel_tank.fuel.mass_properties.mass / fuel_tank.fuel.density
                    total_fuel_volume += fuel_tank.fuel.volume_properties.net_volume 
                    total_fuel_mass   += fuel_tank.fuel.mass_properties.mass 
                    
    # Assign Total Fuel Volume and Mass to Vehicle 
    vehicle.volume_properties.fuel = total_fuel_volume
    vehicle.mass_properties.fuel   = total_fuel_mass
    return 