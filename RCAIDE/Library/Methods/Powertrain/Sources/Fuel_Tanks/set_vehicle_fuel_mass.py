# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/set_vehicle_fuel_mass.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from .distribute_fuel_across_tanks import distribute_fuel_across_tanks

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------
def set_vehicle_fuel_mass(vehicle, fuel_mass):
    """
    Sets the vehicle fuel load and splits it across all of its fuel tanks by capacity,
    so ``vehicle.mass_properties.fuel`` and each ``tank.fuel.mass_properties.mass`` agree.

    Parameters
    ----------
    vehicle : Vehicle
        Vehicle whose fuel load is set.
    fuel_mass : float
        Total fuel mass [kg].

    Raises
    ------
    ValueError
        If ``fuel_mass`` exceeds the combined capacity of the vehicle's fuel tanks.
    """
    tanks = [source for network in vehicle.networks for source in network.sources
             if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank)
             and source.volume_properties.net_volume * source.fuel.density > 0]
    if len(tanks) > 0:
        distribute_fuel_across_tanks(tanks, fuel_mass)
    vehicle.mass_properties.fuel = fuel_mass
    return
