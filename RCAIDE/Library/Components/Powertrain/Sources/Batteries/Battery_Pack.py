# RCAIDE/Library/Components/Powertrain/Powertrain/Sources/Batteries/Battery_Pack.py
# 
# 
# Created:  Mar 2024, M. Clarke
# Modified: Sep 2024, S. Shekar
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE 
from RCAIDE.Framework.Core     import Data, Container
from RCAIDE.Library.Components.Powertrain.Sources.Source    import Source   
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common.append_battery_conditions import append_battery_conditions, append_battery_segment_conditions
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common.compute_battery_pack_performance import compute_battery_pack_performance
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common.compute_battery_pack_properties import compute_battery_pack_properties
# ----------------------------------------------------------------------------------------------------------------------
#  Battery
# ----------------------------------------------------------------------------------------------------------------------      
class Battery_Pack(Source):
    """
    Base class for battery module implementations
    
    Attributes
    ----------
    energy_density : float
        Energy stored per unit volume [J/m^3] (default: 0.0) 
        
    current_capacitor_charge : float
        Current charge level of capacitor [C] (default: 0.0)
        
    capacity : float
        Total energy capacity [J] (default: 0.0)
        
    length : float
        Physical length of battery module [m] (default: 0.0)
        
    width : float
        Physical width of battery module [m] (default: 0.0)
        
    height : float
        Physical height of battery module [m] (default: 0.0)
        
    volume_packaging_factor : float
        Factor accounting for packaging volume (default: 1.05)
        
    BMS_additional_weight_factor : float
        Factor for battery management system weight (default: 1.42)
        
    orientation_euler_angles : list
        Euler angles defining battery orientation [rad] (default: [0,0,0])
        
    cell : Data
        Container for cell-specific attributes
            - chemistry : str
                Battery chemistry type (default: None)
            - discharge_performance_map : Data
                Discharge performance characteristics
            - ragone : Data
                Ragone plot parameters
            
    electrical_configuration : Data
        Battery electrical arrangement
            - series : int
                Number of cells in series (default: 1)
            - parallel : int
                Number of parallel strings (default: 1)
            
    geometrtic_configuration : Data
        Physical arrangement of cells
            - normal_count : int
                Cells in normal direction (default: 1)
            - parallel_count : int
                Cells in parallel direction (default: 1)
            - normal_spacing : float
                Spacing between normal cells [m] (default: 0.02)
            - stacking_rows : int
                Number of stacking rows (default: 3)
            - parallel_spacing : float
                Spacing between parallel cells [m] (default: 0.02)

    Notes
    -----
    This base class provides the framework for implementing specific battery types.
    It includes physical, electrical, and geometric parameters needed to model
    battery performance and integration.

    **Definitions**

    'Battery Management System (BMS)'
        System that monitors and controls battery operation, adding weight
        accounted for by BMS_additional_weight_factor
        
    'Volume Packaging Factor'
        Ratio of total battery volume to cell volume, accounting for
        structural components and thermal management

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Lithium_Ion_NMC
        Example implementation of a specific battery type
    """
    
    def __defaults__(self):
        """
        Sets default values for battery module attributes
        """
        
        self.tag                                   = 'battery_pack'
        self.energy_density                        = 0.0 
        self.capacity                              = 0.0
        self.voltage                               = 0.0
        self.modules                               = Container()
        self.identical_modules                     = True
        self.orientation_euler_angles              = [0.,0.,0.]
        self.number_of_active_modules              = 0
        self.charging_c_rate                       = 1.0 
        self.battery_module_electric_configuration = "Series" 
        self.maximum_energy                        = 0.0
        self.specific_energy                       = 0.0
        self.maximum_power                         = 0.0
        self.specific_power                        = 0.0
        self.maximum_voltage                       = 0.0
        self.initial_maximum_energy                = 0.0
        self.nominal_capacity                      = 0.0 
    
    def compute_performance(self,state,network): 
        """
        Computes the state of the NMC battery cell
        
        This method calculates the battery's electrical performance and thermal
        behavior during operation, including voltage, current, power, and 
        temperature distributions.

        Parameters
        ----------
        state : Data
            Current system state containing:
            - Temperature distributions
            - Power demands
            - Operating conditions
            
        bus : Component
            Connected electrical bus containing:
            - Voltage requirements
            - Power requirements
            - Load characteristics
            
        coolant_lines : Component
            Thermal management system containing:
            - Coolant properties
            - Flow conditions
            - Heat exchanger parameters
            
        t_idx : int
            Current time index in the simulation
            
        delta_t : float
            Time step size [s]

        Returns
        -------
        stored_results_flag : bool
            Flag indicating if results were stored for future reuse
            
        stored_battery_tag : str
            Identifier for stored battery state data

        Notes
        -----
        The calculation includes:
        - Voltage and current based on load demand
        - Heat generation from internal resistance
        - Thermal distribution with cooling effects
        - State of charge tracking
        """

        inputs, outputs, stored_results_flag, stored_source_tag =  compute_battery_pack_performance(self,state,network)
        return inputs, outputs, stored_results_flag, stored_source_tag
    
 
    def append_operating_conditions(self,segment):  
        """
        Append battery operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
        bus : Component
            Electrical bus connected to this battery
        """
        append_battery_conditions(self,segment)

        for m_i, module in enumerate(self.modules):
            if module.active: 
                self.number_of_active_modules += 1 
                if (self.identical_modules == False) or m_i == 0: 
                    module.append_operating_conditions(self,segment) 
        return

    def initialize(self,network):
        compute_battery_pack_properties(self,network)
        return
    
    def append_segment_conditions(self,segment):
        """
        Append segment-specific battery conditions
        
        Parameters
        ----------
        bus : Component
            Electrical bus connected to this battery
        conditions : Data
            Container for segment conditions
        segment : Segment
            Flight segment data
        """
        append_battery_segment_conditions(self,segment)
        for m_i, module in enumerate(self.modules):
            if module.active and (self.identical_modules == False or m_i == 0): 
                module.append_segment_conditions(self,segment) 
        return 

    def unpack_unknowns(self,segment):
        for m_i, module in enumerate(self.modules):
            if module.active and (self.identical_modules == False or m_i == 0): 
                module.unpack_unknowns(self,segment) 
        return 

    def pack_residuals(self,segment): 
        for m_i, module in enumerate(self.modules):
            if module.active and (self.identical_modules == False or m_i == 0): 
                module.pack_residuals(self,segment) 
        return        
       
    def append_unknowns_and_residuals(self,segment):

        """
        Append battery unknowns and residuals  flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
        bus : Component
            Electrical bus connected to this battery
        network: 

        """
        for m_i, module in enumerate(self.modules):
            if module.active and (self.identical_modules == False or m_i == 0): 
                module.append_unknowns_and_residuals(self,segment)   
        return

    
    def update_battery_age(self,segment,battery_conditions,increment_battery_age_by_one_day = False):  
        """
        Updates battery aging parameters based on usage and environmental conditions
        
        This method tracks battery degradation by considering factors such as:
        cycle count, depth of discharge, temperature exposure, and calendar aging.

        Parameters
        ----------
        segment : Segment
            Flight segment containing:
            - Duration
            - Operating conditions
            - Power profile
            
        battery_conditions : Data
            Battery state data including:
            - Temperature history
            - Current rates
            - State of charge history
            
        increment_battery_age_by_one_day : bool, optional
            Flag to increment calendar age (default: False)

        Notes
        -----
        The aging model accounts for:
        - Capacity fade from cycling
        - Calendar aging effects
        - Temperature-dependent degradation
        - Current rate impacts
        """

        for m_i, module in enumerate(self.modules): 
            module.update_battery_age(self,segment,increment_battery_age_by_one_day = increment_battery_age_by_one_day)
        return
    
        
    def append_module(self,module):
        """
        Adds a new segment to the boom's segment container.

        Parameters
        ----------
        segment : Data
            Boom segment to be added
        """

        # Assert database type
        if not issubclass(type(module),RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Generic_Battery_Module):
            raise Exception('input component must be of type battery module')

        # Store data
        self.modules.append(module)

        return    
    