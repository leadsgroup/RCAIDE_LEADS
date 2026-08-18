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
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common.compute_battery_performance import compute_battery_performance
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common.compute_battery_properties import compute_battery_properties
# ----------------------------------------------------------------------------------------------------------------------
#  Battery
# ----------------------------------------------------------------------------------------------------------------------      
class Battery_Pack(Source):
    """
    A pack of one or more battery modules wired together and connected to a
    distributor (electrical bus).

    Attributes
    ----------
    tag : str
        Identifier for the pack (default: 'battery_pack')

    domain : str
        Power domain this source provides (default: 'electrical')

    energy_density : float
        Pack-level energy density [J/m^3] (default: 0.0)

    capacity : float
        Total energy capacity [J] (default: 0.0)

    voltage : float
        Pack voltage [V] (default: 0.0)

    modules : Container
        Collection of `Generic_Battery_Module` instances making up this pack

    identical_modules : bool
        Whether all modules are treated as identical for solver reuse
        purposes (default: True)

    orientation_euler_angles : list
        Euler angles defining pack orientation [rad] (default: [0,0,0])

    number_of_active_modules : int
        Count of currently active modules (default: 0)

    charging_c_rate : float
        C-rate used when this pack is being recharged (default: 1.0)

    battery_module_electric_configuration : str
        How modules are wired together, e.g. 'Series' (default: 'Series')

    maximum_energy : float
        Maximum usable energy [J] (default: 0.0)

    specific_energy : float
        Energy per unit mass [J/kg] (default: 0.0)

    maximum_power : float
        Maximum power the pack can deliver [W] (default: 0.0)

    specific_power : float
        Power per unit mass [W/kg] (default: 0.0)

    maximum_voltage : float
        Maximum pack voltage [V] (default: 0.0)

    initial_maximum_energy : float
        Maximum energy at the start of the mission, before any degradation
        (default: 0.0)

    nominal_capacity : float
        Nominal pack capacity (default: 0.0)

    design_voltage : float
        Target pack voltage [V] used to size each module's cell series/parallel
        configuration when a module's electrical_configuration is left unset
        (default: None)

    design_power : float
        Target pack power [W] used the same way as design_voltage (default: None)

    design_capacity : float
        Target pack energy capacity [J] used the same way as design_voltage
        (default: None)

    Notes
    -----
    This class provides the framework for implementing specific battery pack
    types. Cell/module-level physical, electrical, and geometric parameters
    live on the `Generic_Battery_Module` instances in `modules`, not on the
    pack itself.

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
        self.domain                                = 'electrical'
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
        self.design_voltage                        = None
        self.design_power                          = None
        self.design_capacity                       = None
    
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

        network : RCAIDE.Framework.Networks.Network
            The network this pack belongs to, used to resolve its assigned
            distributor(s) and any connected thermal management components

        Returns
        -------
        inputs : Data
            Pack input conditions (e.g. power.electrical delivered to the pack)

        outputs : Data
            Pack output conditions (e.g. power.electrical drawn from the pack)

        stored_results_flag : bool
            Flag indicating if results were stored for future reuse

        stored_source_tag : str
            Identifier for stored battery state data

        Notes
        -----
        The calculation includes:
        - Voltage and current based on load demand
        - Heat generation from internal resistance
        - Thermal distribution with cooling effects
        - State of charge tracking
        """

        inputs, outputs, stored_results_flag, stored_source_tag =  compute_battery_performance(self,state,network)
        return inputs, outputs, stored_results_flag, stored_source_tag
    
 
    def append_operating_conditions(self,segment):  
        """
        Append battery operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
        """
        append_battery_conditions(self,segment)
        return

    def initialize(self,network):
        compute_battery_properties(self,network)
        return
    
    def append_segment_conditions(self,segment):
        """
        Append segment-specific battery conditions

        Parameters
        ----------
        segment : Segment
            Flight segment data
        """
        append_battery_segment_conditions(self,segment) 
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
        Append battery unknowns and residuals for a flight segment

        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
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
        Adds a battery module to the pack's module container.

        Parameters
        ----------
        module : Generic_Battery_Module
            Battery module to be added
        """

        # Assert database type
        if not issubclass(type(module),RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Generic_Battery_Module):
            raise Exception('input component must be of type battery module')

        # Store data
        self.modules.append(module)

        return    
    