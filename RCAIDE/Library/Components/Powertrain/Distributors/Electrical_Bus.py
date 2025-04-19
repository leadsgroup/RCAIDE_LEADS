# RCAIDE/Library/Components/Powertrain/Distributors/Electrical_Bus.py 
# 
# Created:  Jul 2023, M. Clarke 
# Modofied: Jan 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
import RCAIDE 
from RCAIDE.Library.Components                                 import Component
from RCAIDE.Library.Components.Component                       import Container
from RCAIDE.Library.Methods.Powertrain.Distributors.Electrical_Bus import *


# ----------------------------------------------------------------------------------------------------------------------
#  Electrical_Bus
# ---------------------------------------------------------------------------------------------------------------------- 
class Electrical_Bus(Component):
    """
    Class for managing power distribution between aircraft electrical components
    
    Attributes
    ----------
    tag : str
        Identifier for the electrical bus (default: 'bus')
        
    battery_modules : Container
        Collection of battery modules connected to this bus
        
    assigned_propulsors : list
        List of propulsion systems powered by this bus
        
    solar_panel : Component, optional
        Solar panel system connected to this bus
        
    avionics : Component
        Aircraft avionics system
        
    payload : Component
        Aircraft payload system
        
    identical_battery_modules : bool
        Flag indicating if all battery modules are identical (default: True)
        
    active : bool
        Flag indicating if the bus is operational (default: True)
        
    efficiency : float
        Power distribution efficiency (default: 1.0)
        
    voltage : float
        Bus voltage in volts (default: 0.0)
        
    power_split_ratio : float
        Ratio of power distribution between multiple buses (default: 1.0)
        
    nominal_capacity : float
        Total capacity of connected batteries (default: 0.0)
        
    charging_c_rate : float
        Battery charging rate in C (default: 1.0)
        
    battery_module_electric_configuration : str
        Configuration of battery modules ('Series' or 'Parallel') (default: 'Series')

    Notes
    -----
    The electrical bus manages power distribution between sources and consumers,
    handling voltage regulation, power splitting, and battery management. It supports
    both series and parallel battery configurations.

    **Definitions**

    'C-rate'
        Rate at which a battery is charged/discharged relative to its capacity
        
    'Power Split Ratio'
        Fraction of total power handled by this bus in multi-bus configurations

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules
        Battery module components
    RCAIDE.Library.Components.Powertrain.Sources.Solar_Panels
        Solar panel components
    """
    
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """                
        self.tag                                    = 'bus' 
        self.battery_modules                        = Container()
        self.fuel_cell_stacks                       = Container()
        self.cryogenic_tanks                        = Container()
        self.assigned_propulsors                    = []
        self.solar_panel                            = None 
        self.avionics                               = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
        self.payload                                = RCAIDE.Library.Components.Payloads.Payload()         
        self.identical_battery_modules              = True      
        self.identical_fuel_cell_stacks             = True  
        self.active                                 = True
        self.efficiency                             = 1.0
        self.voltage                                = 0.0 
        self.power_split_ratio                      = 1.0
        self.nominal_capacity                       = 0.0
        self.charging_c_rate                        = 1.0 
        self.battery_module_electric_configuration  = "Series"
        self.fuel_cell_stack_electric_configuration = "Series"
        
    def append_operating_conditions(self, segment):
        """
        Append operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing operating conditions
        """
        append_bus_conditions(self, segment)
        return
        
    def append_segment_conditions(self, conditions, segment):
        """
        Append segment-specific conditions to the bus
        
        Parameters
        ----------
        conditions : Data
            Container for segment conditions
        segment : Segment
            Flight segment data
        """
        append_bus_segment_conditions(self, conditions, segment)
        return    
    
    def initialize_bus_properties(self):
        """
        Initialize electrical bus properties
        
        Sets up initial values for bus voltage, capacity, and other electrical
        properties based on connected components.
        """
        initialize_bus_properties(self)
        return
        
    def compute_distributor_conditions(self,state,t_idx, delta_t):
        """
        Compute electrical conditions during operation
        
        Parameters
        ----------
        state : Data
            Current system state
        t_idx : int
            Time index
        delta_t : float
            Time step
        """
        compute_bus_conditions(self,state,t_idx, delta_t)
        return    