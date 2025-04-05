# RCAIDE/Library/Components/Powertrain/Energy/Sources/Battery_Modules/Generic_Battery_Module.py
# 
# 
# Created:  Mar 2024, M. Clarke
# Modified: Sep 2024, S. Shekar
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Framework.Core        import Data
from RCAIDE.Library.Components    import Component   
from RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common.append_battery_conditions import append_battery_conditions, append_battery_segment_conditions

# ----------------------------------------------------------------------------------------------------------------------
#  Battery
# ----------------------------------------------------------------------------------------------------------------------      
class Generic_Battery_Module(Component):
    """
    Base class for battery module implementations
    
    Attributes
    ----------
    energy_density : float
        Energy stored per unit volume [J/m^3] (default: 0.0)
        
    current_energy : float
        Current energy stored in battery [J] (default: 0.0)
        
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
    RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Lithium_Ion_NMC
        Example implementation of a specific battery type
    """
    
    def __defaults__(self):
        """
        Sets default values for battery module attributes
        """
        self.energy_density                                    = 0.0
        self.current_energy                                    = 0.0
        self.current_capacitor_charge                          = 0.0
        self.capacity                                          = 0.0
            
        self.length                                            = 0.0
        self.width                                             = 0.0
        self.height                                            = 0.0
        self.volume_packaging_factor                           = 1.05
        self.BMS_additional_weight_factor                      = 1.42
                 
        self.orientation_euler_angles                          = [0.,0.,0.]  # vector of angles defining default orientation of rotor        
                     
        self.cell                                              = Data()
        self.cell.chemistry                                    = None                             
        self.cell.discharge_performance_map                    = None  
        self.cell.ragone                                       = Data()
        self.cell.ragone.const_1                               = 0.0     # used for ragone functions; 
        self.cell.ragone.const_2                               = 0.0     # specific_power=ragone_const_1*10^(specific_energy*ragone_const_2)
        self.cell.ragone.lower_bound                           = 0.0     # lower bound specific energy for which ragone curves no longer make sense
        self.cell.ragone.i                                     = 0.0 
 
        self.electrical_configuration                          = Data()
        self.electrical_configuration.series                   = 1
        self.electrical_configuration.parallel                 = 1   
        
        self.geometrtic_configuration                          = Data() 
        self.geometrtic_configuration.normal_count             = 1
        self.geometrtic_configuration.parallel_count           = 1
        self.geometrtic_configuration.normal_spacing           = 0.02
        self.geometrtic_configuration.stacking_rows            = 3
        self.geometrtic_configuration.parallel_spacing         = 0.02                
 
    def append_operating_conditions(self,segment,bus):  
        """
        Append battery operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
        bus : Component
            Electrical bus connected to this battery
        """
        append_battery_conditions(self,segment,bus)  
        return
    
    def append_battery_segment_conditions(self,bus, conditions, segment):
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
        append_battery_segment_conditions(self,bus, conditions, segment)
        return