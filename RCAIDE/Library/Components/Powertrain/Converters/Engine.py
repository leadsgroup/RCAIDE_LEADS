# RCAIDE/Library/Components/Propulsors/Converters/Engine.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports  
from .Converter  import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Engine.append_engine_conditions import append_engine_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  Engine Class
# ----------------------------------------------------------------------------------------------------------------------  
class Engine(Converter):
    """
    An internal combustion engine component model for propulsion systems.

    Attributes
    ----------
    tag : str
        Identifier for the engine. Default is 'internal_combustion_engine'.
        
    sea_level_power : float
        Maximum power output at sea level conditions [W]. Default is 0.0.
        
    flat_rate_altitude : float
        Altitude up to which engine maintains sea level power [m]. Default is 0.0.
        
    rated_speed : float
        Engine speed at rated power [rad/s]. Default is 0.0.
        
    power_split_ratio : float
        Ratio of power distribution when engine drives multiple loads. Default is 0.0.
        
    power_specific_fuel_consumption : float
        Fuel consumption per unit power output [kg/W/s]. Default is 0.36.

    Notes
    -----
    The Engine class models an internal combustion engine's performance characteristics
    including:
    * Sea level power rating
    * Power lapse with altitude
    * Fuel consumption characteristics
    * Power distribution capabilities
    * Operating speed effects

    **Major Assumptions**
    * Constant power specific fuel consumption
    * Fuel properties remain constant
    * Steady-state operation (no transient effects)

    **Definitions**
    
    'Flat Rate Altitude'
        Maximum altitude at which the engine can maintain sea level power output

    'Power Specific Fuel Consumption'
        Amount of fuel consumed per unit of power output per unit time

    'Power Split Ratio'
        Fraction of total power delivered to primary load in multi-load applications

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Engine
    """           
    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
            None

        Source:
            None 
        """      
        self.tag                             = 'internal_combustion_engine' 
        self.sea_level_power                 = 0.0
        self.flat_rate_altitude              = 0.0
        self.rated_speed                     = 0.0  
        self.power_split_ratio               = 0.0
        self.power_specific_fuel_consumption = 0.36

    def append_operating_conditions(self,segment,energy_conditions,noise_conditions=None):   
        append_engine_conditions(self,segment,energy_conditions,noise_conditions) 
        return                

