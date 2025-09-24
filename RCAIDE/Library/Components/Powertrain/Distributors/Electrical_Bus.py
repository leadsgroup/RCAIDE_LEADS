# RCAIDE/Library/Components/Powertrain/Distributors/Electrical_Bus.py 
# 
# Created:  Jul 2023, M. Clarke 
# Modified: Jan 2025, M. Clarke 
#           Sep 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
import RCAIDE 
from RCAIDE.Framework.Core                                      import Data
from RCAIDE.Library.Components                                     import Component
from RCAIDE.Library.Components.Component                           import Container
from RCAIDE.Library.Methods.Powertrain.Distributors.Electrical_Bus import *
from RCAIDE.Library.Attributes.Materials                           import Copper, Polyimide

# ----------------------------------------------------------------------------------------------------------------------
#  Electrical_Line
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
        
    avionics : Component
        Aircraft avionics system 
        
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
    """
    
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """                
        self.tag                                    = 'electrical_line' 
        self.bus_type                               = 'DC'
        self.battery_modules                        = Container()
        self.fuel_cell_stacks                       = Container()
        self.fuel_tanks                             = Container()
        self.electrical_line                        = Electrical_Line()
        self.assigned_propulsors                    = Data()
        self.assigned_converters                    = Data() 
        self.assigned_distributors                  = Data() 
        self.assigned_modulators                    = Data() 
        self.assigned_sources                       = Data()
        self.systems                                = RCAIDE.Library.Components.Powertrain.Systems.Systems()
        self.identical_battery_modules              = True      
        self.identical_fuel_cell_stacks             = True  
        self.active                                 = True
        self.efficiency                             = 1.0
        self.voltage                                = 0.0 
        self.voltage_phase_to_neutral               = 115.0 
        self.voltage_phase_to_phase                 = 200.0
        self.frequency                              = 0.0
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
        
    def append_segment_conditions(self, segment):
        """
        Append segment-specific conditions to the bus
        
        Parameters
        ----------
        conditions : Data
            Container for segment conditions
        segment : Segment
            Flight segment data
        """
        append_bus_segment_conditions(self,segment)
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
    
class Electrical_Line(Component):
    def __defaults__(self):
        self.tag                               = 'electrical_line'
        self.to                                = None
        self.from_                             = None
        self.current_type                      = 'DC'  
        self.voltage                           = 0  
        self.efficiency                        = 1
        self.length                            = 0
        self.design_ambient_temperature        = 273 # kelvin
        self.maximum_insulator_electric_field  = 0 # NEED TO CHECK 
        self.maximum_operating_temperature     = 0 # NEED TO CHECK 
        self.maximum_current                   = 0 # NEED TO CHECK
        self.maximum_temperature               = 0 # NEED TO CHECK
        self.environmental_external_thermal_resistance = 0 # CHECK  IEC 60287-2-1 Section 4.2.1.1.
        self.conductor_radius                  = None
        self.conductor_material                = Copper()  # Default conductor material
        self.insulator_radius                  = None
        self.insulator_material                = Polyimide()  # Default insulator material 
        self.duplicate_wires                   =  2# Number of duplicate cables for redundancy
    


