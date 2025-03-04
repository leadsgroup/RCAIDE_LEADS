# RCAIDE/Library/Methods/Powertrain/Converters/Engine/compute_throttle_from_power.py
# (c) Copyright 2023 Aerospace Research Community LLC
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
def compute_throttle_from_power(engine, engine_conditions,conditions):
    """ The internal combustion engine output power and specific power consumption.
    The following perperties are computed:
    engine 
       .outputs.power_specific_fuel_consumption (numpy.ndarray): Power (brake) SFC [lbf/(HP · h) ]
       .outputs.fuel_flow_rate                  (numpy.ndarray): Fuel flow rate    [kg/s]
       .outputs.throttle                        (numpy.ndarray): throttle setting  [unitless]    
    
    Source:
        Gagg and Ferrar model (ref: S. Gudmundsson, 2014 - eq. 7-16)
    
    Assumtions:
        None 
    
    Args:
        engine
          .sea_level_power                        (float): sea-level power      [W]
          .flat_rate_altitude                     (float): flat rate altitude   [m]
          .power_specific_fuel_consumption        (float): PSFC                 [RPM]
          .throttle setting               (numpy.ndarray): throttle             [unitless]
          .inputs.speed                   (numpy.ndarray): angular velocity     [rad/s]
        freestream conditions 
          .altitude                               (float): altitde              [m]
          .delta_isa                              (float): altitude offset      [m]
          
    Returns:
        None 
    """

    # Unpack atmospheric conditions 
    delta_isa  = conditions.freestream.delta_ISA
    altitude   = conditions.freestream.altitude
    
    # Unpack engine operating conditions 
    PSLS     = engine.sea_level_power
    h_flat   = engine.flat_rate_altitude
    P        = engine_conditions.power*1.0
    PSFC     = engine.power_specific_fuel_consumption
    
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
