# RCAIDE/Library/Components/Powertrain/Distributors/Electrical_Bus.py 
# 
# Created:  Jul 2023, M. Clarke 
# Modified: Jan 2025, M. Clarke 
#           Sep 2025, M. Guidotti
#           Apr 2026, S. Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
import RCAIDE
from RCAIDE.Library.Methods.Powertrain.Distributors.Fuel_Line.compute_fuel_line_distribution_losses import compute_fuel_line_distribution_losses  
from .Distributor                                                  import Distributor 
from RCAIDE.Library.Methods.Powertrain.Distributors.Electrical_Bus import * 
from RCAIDE.Library.Attributes.Materials                           import Copper, Polyimide 
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_distributor_moment_of_inertia import *
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity.compute_distributor_center_of_gravity import * 
from RCAIDE.Library.Methods.Powertrain.Distributors.Electrical_Bus.size_electrical_cable import size_electrical_cable

# ----------------------------------------------------------------------------------------------------------------------
#  Electrical_Bus
# ---------------------------------------------------------------------------------------------------------------------- 
class Electrical_Bus(Distributor):
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
        
    active : bool
        Flag indicating if the bus is operational (default: True)
        
    efficiency : float
        Power distribution efficiency (default: 1.0)
        
    voltage : float
        Bus voltage in volts (default: 0.0) 
        
    nominal_capacity : float
        Total capacity of connected batteries (default: 0.0)
         
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
    RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules
        Battery module components
    """
    
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """                
        self.tag                                       = 'electrical_line' 
        self.domain                                    = 'electrical'  
        self.active                                    = True
        self.design_power                              = 0.0
        self.design_voltage                            = 0.0  
        self.frequency                                 = 0.0    
        self.current_type                              = 'DC'   
        self.efficiency                                = 1
        self.length                                    = 1
        self.number_of_parallel_wires                  = 1
        self.design_ambient_temperature                = 273 # kelvin
        self.maximum_insulator_electric_field          = 0 # NEED TO CHECK 
        self.maximum_operating_temperature             = 0 # NEED TO CHECK 
        self.maximum_current                           = 0 # NEED TO CHECK
        self.maximum_temperature                       = 423 # NEED TO CHECK
        self.environmental_external_thermal_resistance = 1 # CHECK  IEC 60287-2-1 Section 4.2.1.1. (T4)
        self.conductor                                 = Component()
        self.conductor.radius                          = None
        self.conductor.material                        = Copper()  # Default conductor material
        self.conductor.resistance                  = None
        self.insulator                                 = Component()
        self.insulator.radius                          = None
        self.insulator.material                        = Polyimide()  # Default insulator material 
        self.duplicate_wires                           = 2# Number of duplicate cables for redundancy


    def unpack_unknowns(self,segment):
        return 

    def pack_residuals(self,segment): 
        return        

    def append_unknowns_and_residuals(self,segment):
        return
    
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
    
    
    def compute_distribution_losses(self, component_conditions, state, network):
        compute_electrical_bus_distribution_losses(self, component_conditions, state, network) 
        return

    def compute_moments_of_inertia(self,vehicle,center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for the fuel line.

        Parameters
        ----------
        center_of_gravity : list, optional
            Reference point coordinates for moment calculation, defaults to [[0, 0, 0]] 

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.moments_of_inertia.compute_fuselage_moment_of_inertia
            Implementation of the moment of inertia calculation
        """
        # _ , _ = compute_distributor_moment_of_inertia(self,center_of_gravity= center_of_gravity)  
        return
 
    def initialize(self,network):
        size_electrical_cable(self)
        return
    
    def compute_center_of_gravity(self,vehicle): 
        """
        Computes the center of gravity for the distributor. 

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.center_of_gravity.compute_fuselage_center_of_gravity
            Implementation of the moment of inertia calculation
        """
        # _  = compute_distributor_center_of_gravity(self,vehicle) 
        return 
