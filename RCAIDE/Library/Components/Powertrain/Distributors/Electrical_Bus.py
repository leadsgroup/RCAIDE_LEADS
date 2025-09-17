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
        self.assigned_propulsors                    = []
        self.assigned_converters                    = [] 
        self.assigned_modulators                    = [] 
        self.assigned_sources                       = []
        self.avionics                               = RCAIDE.Library.Components.Powertrain.Systems.Avionics()
        self.systems                                = RCAIDE.Library.Components.Powertrain.Systems.Systems()
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
        

    # def __init__ (self, bus=None):

    #     # loop over batteries and create lines 
    #     for battery_module in bus.battery_modules:
    #         electrical_line       = Electrical_Line()
    #         electrical_line.to    = battery_module.tag
    #         electrical_line.from_ = bus.tag
    #         self.electrical_lines.append(electrical_line) 
 
    #     # loop over fuel_cell and create lines
    #     for fuel_cell_stack in bus.fuel_cell_stacks:
    #         electrical_line       = Electrical_Line()
    #         electrical_line.to    = fuel_cell_stack.tag
    #         electrical_line.from_ = bus.tag
    #         self.electrical_lines.append(electrical_line) 

    #     # loop over propulsors and create lines 
    #     for propulsor_tag in bus.assigned_propulsors:
    #         electrical_line       = Electrical_Line()
    #         electrical_line.to    = propulsor_tag
    #         electrical_line.from_ = bus.tag
    #         self.electrical_lines.append(electrical_line) 

    #     # loop over converters  and create lines
    #     for converter_tag in bus.assigned_converters:
    #         electrical_line       = Electrical_Line()
    #         electrical_line.to    = converter_tag
    #         electrical_line.from_ = bus.tag
    #         self.electrical_lines.append(electrical_line) 
        
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
        self.tag = 'electrical_line'
        self.to = None
        self.from_ = None
        self.current_type = 'DC'  
        self.voltage = 0  
        self.efficiency = 1
        self.length = 0  
        self.diameter_conductor = 0.005  # Default conductor diameter
        self.diameter_insulator = 0.01  # Default insulator diameter
        self.conductor_material = Copper()  # Default conductor material
        self.insulator_material = Polyimide()  # Default insulator material
        self.duplicates = 2  # Number of duplicate cables for redundancy
    


