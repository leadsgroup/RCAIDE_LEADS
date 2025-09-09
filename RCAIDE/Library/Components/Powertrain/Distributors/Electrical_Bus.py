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
        self.assigned_propulsors                    = []
        self.assigned_converters                    = [] 
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

class Electrical_Line(Electrical_Bus):
    """
    Subclass of Electrical_Bus for managing specific electrical line configurations.
    """
    def __defaults__(self):
        """Set default values specific to Electrical_Line."""
        super().__defaults__()  # Call parent defaults
        self.tag = 'electrical_line'
        self.current_type = 'DC'  # Default current type

        # Conditional defaults based on current type
        if self.current_type == 'DC':
            self.voltage = 400.0  # Default voltage for DC
            self.efficiency = 0.95  # Default efficiency for DC
        elif self.current_type == 'AC':
            self.voltage = 230.0  # Default voltage for AC
            self.efficiency = 0.90  # Default efficiency for AC

        self.length = 10.0  # Default length for electrical line
        self.diameter_conductor = 0.005  # Default conductor diameter
        self.diameter_insulator = 0.01  # Default insulator diameter
        self.conductor_material = Copper()  # Default conductor material
        self.insulator_material = Polyimide()  # Default insulator material
    
def cable_mass(V, E0, r_cond, rho, rho_theta_insul,L, rho_cond, rho_insul, theta_a, I, T_4):

    # Equation (18): Cable Insulation Radius based on voltage and electric field constraints
    # E0 is the electric field
    r_insul = r_cond * np.exp(V / (E0 * r_cond))  # Equation (18)

    # Equation (20): Conductor Resistance (thermal constraint based on material properties)
    R_prime = rho / (np.pi * r_cond ** 2)  # Equation (20)

    # Equation (21): Thermal Resistance of the insulation
    T_1 = rho_theta_insul / (2 * np.pi) * np.log(r_insul / r_cond)  # Equation (21)

    # Equation (22): Total Cable Mass calculation based on conductor and insulation volume and density
    M_cable = np.pi * L * (r_cond ** 2 * rho_cond + (r_insul ** 2 - r_cond ** 2) * rho_insul)  # Equation (22)

    # Equation (19): Maximum Temperature (conductor temperature based on current, resistance, and thermal resistances)
    theta_max = theta_a + I**2 * R_prime * (T_1 + T_4)  # Equation (19)
    
    return M_cable, theta_max