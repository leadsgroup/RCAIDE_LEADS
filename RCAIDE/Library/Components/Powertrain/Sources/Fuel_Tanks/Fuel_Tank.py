# RCAIDE/Library/Components/Powertrain/Sources/Fuel_Tanks/Fuel_Tank.py
# 
# 
# Created:  Mar 2024, M. Clarke 
# Modified: Aug 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Library.Components          import Component
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks  import * 

# ----------------------------------------------------------------------------------------------------------------------
#  Fuel Tank
# ---------------------------------------------------------------------------------------------------------------------     
class Fuel_Tank(Component):
    """
    Base class for aircraft fuel tank implementations
    
    Attributes
    ----------
    tag : str
        Identifier for the fuel tank (default: 'fuel_tank')
        
    fuel_selector_ratio : float
        Ratio of fuel flow allocation (default: 1.0)
        
    mass_properties.empty_mass : float
        Mass of empty tank structure [kg] (default: 0.0)
        
    secondary_fuel_flow : float
        Secondary fuel flow rate [kg/s] (default: 0.0)
        
    fuel : Component, optional
        Fuel type stored in tank (default: None)

    Notes
    -----
    The fuel tank base class provides common attributes and methods for
    different types of aircraft fuel tanks. It handles basic fuel storage
    and flow management functionality.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Central_Fuel_Tank
        Center section fuel tank
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Wing_Fuel_Tank
        Wing-mounted fuel tank
    """
    
    def __defaults__(self):
        """
        Sets default values for fuel tank attributes
        """          
        self.tag                                   = 'fuel_tank'  
        self.fuel                                  = None
        self.secondary_mass_flow_rate              = 0.0
        self.fuel_selector_ratio                   = 1.0    
        self.wall_clearance                        = 0.0
        self.wall_thickness                        = 1E-3
        self.xz_plane_symmetric                    = True
        self.wing_tag                              = None
        self.fuselage_tag                          = None
        self.inner_length                          = 0.0
        self.outer_length                          = 0.0 
        self.outer_width                           = 0.0
        self.outer_height                          = 0.0
        self.inner_diameter                        = 0.0
        self.outer_diameter                        = 0.0
 
    def append_operating_conditions(self,segment):  
        """
        Append fuel tank operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
        fuel_line : Component
            Connected fuel line component
        """
        append_fuel_tank_conditions(self,segment)  
        return
    
    def compute_tank_properties(self,state,fuel_line):
        compute_fuel_tank_properties(self,state,fuel_line)
        return