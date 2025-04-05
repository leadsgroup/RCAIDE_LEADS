# RCAIDE/Library/Methods/Powertrain/Converters/Engine/compute_power_from_throttle.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
 # RCAIDE imports 
import RCAIDE

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
            - sea_level_power : float
                Maximum power output at sea level [W]
            - flat_rate_altitude : float
                Altitude below which power remains constant [m]
            - power_specific_fuel_consumption : float
                Power specific fuel consumption [kg/(W·s)]
    engine_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Engine operating conditions with:
            - throttle : numpy.ndarray
                Throttle setting [dimensionless]
            - speed : numpy.ndarray
                Engine angular velocity [rad/s]
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - freestream.altitude : numpy.ndarray
                Current altitude [m]
            - freestream.delta_ISA : numpy.ndarray
                Temperature offset from standard atmosphere [K]

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
    [1] Gudmundsson, S. (2014). General Aviation Aircraft Design: Applied Methods 
        and Procedures. Butterworth-Heinemann.
    [2] Gagg and Ferrar

    See Also
    --------
    RCAIDE.Library.Methods.Propulsors.Converters.Engine.compute_throttle_from_power
    """

    # Unpack
    altitude          = conditions.freestream.altitude
    delta_isa         = conditions.freestream.delta_ISA 
    PSLS              = engine.sea_level_power  
    h_flat            = engine.flat_rate_altitude
    engine_conditions = conditions.energy.converters[engine.tag] 
    omega             = engine_conditions.omega
    PSFC              = engine.power_specific_fuel_consumption  

    # shift in power lapse due to flat rate
    altitude_virtual = altitude - h_flat       
    altitude_virtual[altitude_virtual<0.] = 0. 

    # Compute the sea-level ISA atmosphere conditions
    atmo             = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_values_0    = atmo.compute_values(0,0) 
    rho0             = atmo_values_0.density[0,0]     
    atmo_values      = atmo.compute_values(altitude_virtual,delta_isa) 
    rho              = atmo_values.density 
    
    # Compute the density ratio:
    sigma = rho / rho0
    
    # Compute available power 
    Pavailable                    = PSLS * (sigma - 0.117) / 0.883        
    Pavailable[h_flat > altitude] = PSLS

    # Regulate using throttle 
    P       = Pavailable * engine_conditions.throttle  
    
    m_dot  =  PSFC * P 

    # Compute engine torque
    torque = P/omega
    
    # Determine fuel flow rate and cap at 0
    fuel_flow_rate  = np.fmax(m_dot,np.zeros_like(altitude)) 
    
    # Store results 
    engine_conditions.power                           = P
    engine_conditions.power_specific_fuel_consumption = PSFC
    engine_conditions.fuel_flow_rate                  = fuel_flow_rate
    engine_conditions.torque                          = torque

    return
