# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank/compute_cryogenic_tank_heater_design_power.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

from RCAIDE.Framework.Core import Units

# ----------------------------------------------------------------------------------------------------------------------
#  Cryogenic Tank Heater Design Power
# ----------------------------------------------------------------------------------------------------------------------
def compute_cryogenic_tank_heater_design_power(tank, network):
    """
    Rated power of a cryogenic tank's pressure-builder heater, sized to hold the
    ullage at the minimum operating pressure while the tank feeds the engines at
    their takeoff rating: the liquid volume drawn per second is backfilled with
    saturated vapor at that pressure, which takes the latent heat to produce.

        P_heater = margin * (m_dot_takeoff / rho_l) * rho_g * (h_g - h_l)

    m_dot_takeoff is the tank's ``power_split_ratio`` share of the fuel chemical
    power that the propulsors on its fuel line draw at the sea-level static
    takeoff rating (``Propulsor.sealevel_static_chemical_power``), divided by the
    fuel's lower heating value. Saturated properties are evaluated at the
    saturation temperature of the tank's minimum operating pressure.

    Parameters
    ----------
    tank : Cryogenic_Tank
        Sized tank (``minimum_operating_pressure`` set) with ``assigned_distributors`` and
        ``power_split_ratio``.
    network : Network
        The network the tank belongs to.

    Returns
    -------
    float or None
        Heater design power [W], or None when no propulsor on the tank's fuel
        line reports a takeoff chemical power.
    """
    fuel_line = tank.assigned_distributors[0][0]
    P_chem    = sum(getattr(propulsor, 'sealevel_static_chemical_power', 0.0) for propulsor in network.propulsors
                    if propulsor.assigned_distributors is not None and fuel_line in propulsor.assigned_distributors[0])
    if P_chem <= 0.0:
        return None

    fuel          = tank.fuel
    m_dot_takeoff = tank.power_split_ratio * P_chem / fuel.lower_heating_value
    (_, _), (P_lo, P_hi) = fuel.property_table_range(phase='liquid')
    T_sat         = fuel.saturation_temperature(np.clip(np.atleast_1d(tank.minimum_operating_pressure / Units.MPa), P_lo, P_hi))
    rho_l, h_l    = fuel.cryogen_property_set(T_sat, ("Density (kg/m3)", "Enthalpy (kJ/kg)"), phase='liquid')
    rho_g, h_g    = fuel.cryogen_property_set(T_sat, ("Density (kg/m3)", "Enthalpy (kJ/kg)"), phase='vapor')
    return float(np.ravel(tank.heater_power_margin * (m_dot_takeoff / rho_l) * rho_g * (h_g - h_l) * 1000.0)[0])
