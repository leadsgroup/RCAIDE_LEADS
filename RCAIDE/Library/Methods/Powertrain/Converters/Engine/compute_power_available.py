# RCAIDE/Library/Methods/Powertrain/Converters/Engine/compute_power_available.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE

# package imports
import numpy as np
from RCAIDE.Framework.Core import array_namespace

# ----------------------------------------------------------------------------------------------------------------------
#  compute_power_available
# ----------------------------------------------------------------------------------------------------------------------
def compute_power_available(engine, altitude, delta_isa):
    """
    Computes the full-throttle power of a flat-rated piston engine at altitude.

    Parameters
    ----------
    engine : RCAIDE.Library.Components.Powertrain.Converters.Engine
        Engine with sealevel_static_power [W] and flat_rate_altitude [m]
    altitude : numpy.ndarray
        Altitude [m]
    delta_isa : numpy.ndarray
        Temperature offset from the standard atmosphere [K]

    Returns
    -------
    power_available : numpy.ndarray
        Full-throttle power [W]

    Notes
    -----
    Below the flat-rate altitude the engine delivers its sea-level static power. Above it, power lapses with the
    density ratio at the altitude shifted down by the flat-rate altitude:

    .. math::
        P_{available} = P_{SL} \\frac{\\sigma - 0.117}{0.883}

    References
    ----------
    [1] Gudmundsson, S. (2014). General Aviation Aircraft Design: Applied Methods and Procedures. Butterworth-Heinemann.
    [2] Gagg, R., and Farrar, E. (1934). Altitude Performance of Aircraft Engines Equipped with Gear-Driven
        Superchargers. SAE Technical Paper 340096. https://doi.org/10.4271/340096
    """
    atmosphere       = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    rho0             = atmosphere.compute_values(0, 0).density[0, 0]
    xp               = array_namespace(altitude, delta_isa)
    altitude_virtual = xp.maximum(altitude - engine.flat_rate_altitude, 0.)
    sigma            = atmosphere.compute_values(altitude_virtual, delta_isa).density / rho0
    PSLS             = engine.sealevel_static_power
    return xp.where(engine.flat_rate_altitude > altitude, PSLS, PSLS * (sigma - 0.117) / 0.883)
