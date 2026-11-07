# RCAIDE/Library/Methods/Powertrain/Converters/Engine/compute_power_from_throttle.py
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
 # RCAIDE imports 
from RCAIDE.Library.Methods.Powertrain.Converters.Engine.compute_power_available import compute_power_available

# package imports
import numpy as np 

# ---------------------------------------------------------------------------------------------------------------------- 
# compute_power_from_throttle
# ----------------------------------------------------------------------------------------------------------------------    
def compute_power_from_throttle(engine,conditions):
    """
    Computes engine power output and performance metrics based on throttle setting and atmospheric conditions.

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
                    - throttle : numpy.ndarray
                        Throttle setting [dimensionless]
                    - omega : numpy.ndarray
                        Engine angular velocity [rad/s]

    Returns
    -------
    None

    Notes
    -----
    Power available is computed using the Gagg and Ferrar model, which accounts for
    atmospheric density effects on engine performance.

    **Major Assumptions**
        * Power varies linearly with density ratio above flat-rate altitude
        * Power remains constant below flat-rate altitude
        * Standard atmosphere conditions apply except for ISA temperature offset
        * Minimum power output is zero (negative values are clipped)
        * Power is directly proportional to throttle setting

    **Theory**
    
    The power available is computed using:

    .. math::
        P_{available} = P_{SL} \\frac{\\sigma - 0.117}{0.883}

    where:
        - :math:`P_{SL}` is sea-level power
        - :math:`\\sigma` is the density ratio

    References
    ----------
    [1] Gudmundsson, S. (2014). General Aviation Aircraft Design: Applied Methods and Procedures. Butterworth-Heinemann.
    [2] Gagg, R., and Farrar, E. (1934). Altitude Performance of Aircraft Engines Equipped with Gear-Driven
        Superchargers. SAE Technical Paper 340096. https://doi.org/10.4271/340096

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Engine.compute_throttle_from_power
    """

    # Unpack
    altitude          = conditions.freestream.altitude
    delta_isa         = conditions.freestream.delta_ISA 
    engine_conditions = conditions.energy.converters[engine.tag] 
    omega             = engine_conditions.omega
    PSFC              = engine.power_specific_fuel_consumption  

    # Compute available power 
    Pavailable        = compute_power_available(engine, altitude, delta_isa)

    # Regulate using throttle 
    P       = Pavailable * engine_conditions.throttle  
    
    m_dot  =  PSFC * P 

    # Compute engine torque
    torque = P/omega
    
    # Determine fuel flow rate and cap at 0
    m_dot_fuel  = np.fmax(m_dot,np.zeros_like(altitude)) 
    
    # Store results 
    engine_conditions.power.propulsive                = P
    engine_conditions.power_specific_fuel_consumption = PSFC
    engine_conditions.fuel_mass_flow_rate             = m_dot_fuel
    engine_conditions.torque                          = torque

    return
