# RCAIDE/Library/Methods/Powertrain/Converters/Engine/compute_throttle_from_power.py
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
 # RCAIDE imports
from RCAIDE.Library.Methods.Powertrain.Converters.Engine.compute_power_available import compute_power_available

# package imports
import numpy as np
from RCAIDE.Framework.Core import array_namespace

# ---------------------------------------------------------------------------------------------------------------------- 
#  calculate_throttle_from_power
# ----------------------------------------------------------------------------------------------------------------------    
def compute_throttle_from_power(engine,conditions):
    """
    Computes engine throttle setting and fuel consumption based on required power output.

    Parameters
    ----------
    engine : RCAIDE.Library.Components.Propulsors
        Engine instance with the following attributes:
            - sealevel_static_power : float
                Maximum power output at sea level [W]
            - flat_rate_altitude : float
                Altitude below which power remains constant [m]
            - power_specific_fuel_consumption : float
                Power specific fuel consumption [kg/(W·s)]
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - freestream.altitude : numpy.ndarray
                Current altitude [m]
            - freestream.delta_ISA : numpy.ndarray
                Temperature offset from standard atmosphere [K]
            - energy.converters[engine.tag] : RCAIDE.Framework.Mission.Common.Conditions
                Engine operating conditions, with:
                    - power.propulsive : numpy.ndarray
                        Required power output [W]

    Returns
    -------
    None

    Notes
    -----
    This function is the inverse of compute_power_from_throttle, calculating the
    required throttle setting to achieve a desired power output considering
    atmospheric conditions.

    **Major Assumptions**
        * Power varies linearly with density ratio above flat-rate altitude
        * Power remains constant below flat-rate altitude
        * Standard atmosphere conditions apply except for ISA temperature offset
        * Minimum power output is zero (negative values are clipped)

    **Theory**
    
    The power available is computed using:

    .. math::
        P_{available} = P_{SL} \\frac{\\sigma - 0.117}{0.883}

    Then throttle is determined by:

    .. math::
        \\text{throttle} = \\frac{P_{required}}{P_{available}}

    where:
        - :math:`P_{SL}` is sea-level power
        - :math:`\\sigma` is the density ratio
        - :math:`P_{required}` is the requested power output

    References
    ----------
    [1] Gudmundsson, S. (2014). General Aviation Aircraft Design: Applied Methods and Procedures. Butterworth-Heinemann.
    [2] Gagg, R., and Farrar, E. (1934). Altitude Performance of Aircraft Engines Equipped with Gear-Driven
        Superchargers. SAE Technical Paper 340096. https://doi.org/10.4271/340096

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Engine.compute_power_from_throttle
    RCAIDE.Library.Attributes.Atmospheres.Earth.US_Standard_1976
    """

    # Unpack atmospheric conditions 
    delta_isa         = conditions.freestream.delta_ISA
    altitude          = conditions.freestream.altitude
    
    # Unpack engine operating conditions 
    engine_conditions = conditions.energy.converters[engine.tag] 
    P                 = engine_conditions.power.propulsive
    PSFC              = engine.power_specific_fuel_consumption
    Pavailable        = compute_power_available(engine, altitude, delta_isa)
 
    # Compute throttle 
    throttle = P/Pavailable 
    xp       = array_namespace(P, Pavailable)
    P        = xp.maximum(P, 0.) 

    # Compute fuel flow rate
    m_dot_fuel      = xp.fmax(P*PSFC, 0.0)
    
    # Store outputs 
    engine_conditions.power_specific_fuel_consumption = PSFC
    engine_conditions.fuel_mass_flow_rate             = m_dot_fuel
    engine_conditions.throttle                        = throttle

    return
