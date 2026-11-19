# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/compute_fuel_tank_capacity.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------
def compute_fuel_tank_capacity(tank):
    """
    Returns the full fuel mass of a tank: net volume times liquid density at design_inlet_temperature for a
    cryogenic tank, and net volume times fuel.density otherwise.

    Parameters
    ----------
    tank : Fuel_Tank
        Fuel tank with volume_properties.net_volume set.

    Returns
    -------
    capacity : float
        Full fuel mass [kg].
    """
    if isinstance(tank, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank):
        density = tank.fuel.cryogen_properties(tank.design_inlet_temperature, "Density (kg/m3)", phase='liquid')
    else:
        density = tank.fuel.density
    return tank.volume_properties.net_volume * float(density)
