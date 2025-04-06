# RCAIDE/Library/Methods/Powertrain/Converters/Engine/compute_throttle_from_power.py
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
 # RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core                                         import Units

# package imports
import numpy as np 

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
            - sea_level_power : float
                Maximum power output at sea level [W]
            - flat_rate_altitude : float
                Altitude below which power remains constant [m]
            - power_specific_fuel_consumption : float
                Power specific fuel consumption [kg/(W·s)]
    engine_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Engine operating conditions with:
            - power : numpy.ndarray
                Required power output [W]
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
    [2] Gagg and Ferrar

    See Also
    --------
    RCAIDE.Library.Methods.Propulsors.Converters.Engine.compute_power_from_throttle
    RCAIDE.Library.Attributes.Atmospheric.Earth.US_Standard_1976
    """

    # Unpack atmospheric conditions 
    delta_isa         = conditions.freestream.delta_ISA
    altitude          = conditions.freestream.altitude
    
    # Unpack engine operating conditions 
    engine_conditions = conditions.energy.converters[engine.tag] 
    PSLS              = engine.sea_level_power
    h_flat            = engine.flat_rate_altitude
    P                 = engine_conditions.power*1.0
    PSFC              = engine.power_specific_fuel_consumption
    
    altitude_virtual = altitude - h_flat        
    altitude_virtual[altitude_virtual<0.] = 0.   
    atmo             = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_values_0    = atmo.compute_values(0,0) 
    rho0             = atmo_values_0.density[0,0] 
    atmo_values      = atmo.compute_values(altitude_virtual,delta_isa) 
    rho              = atmo_values.density 

    #Compute density ratio 
    sigma        = rho / rho0 
    Pavailable   = PSLS * (sigma - 0.117) / 0.883        
    Pavailable[h_flat > altitude] = PSLS 
 
    # Compute throttle 
    throttle = P/Pavailable 
    P[P<0.] = 0. 

    # Compute fuel flow rate
    SFC             = PSFC* Units['lb/hp/hr']
    a               = np.zeros_like(altitude)
    fuel_flow_rate  = np.fmax(P*SFC,a)
    
    # Store outputs 
    engine_conditions.power_specific_fuel_consumption = PSFC
    engine_conditions.fuel_flow_rate                  = fuel_flow_rate
    engine_conditions.throttle                        = throttle

    return
