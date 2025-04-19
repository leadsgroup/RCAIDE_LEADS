# RCAIDE/Library/Components/Powertrain/Energy/Sources/Fuel_Tanks/Wing_Fuel_Tank.py
# 
# Created:  Sep 2024, A. Molloy and M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from .Fuel_Tank  import Fuel_Tank 
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.append_fuel_tank_conditions import append_fuel_tank_conditions 

# ----------------------------------------------------------------------------------------------------------------------
#  Fuel Tank
# ---------------------------------------------------------------------------------------------------------------------    
class Wing_Fuel_Tank(Fuel_Tank):
    """Fuel tank compoment.
    
    Class for modeling wing-mounted fuel tank characteristics and behavior
    
    Attributes
    ----------
    tag : str
        Identifier for the fuel tank (default: 'wing_fuel_tank')
        
    fuel_selector_ratio : float
        Ratio of fuel flow allocation (default: 1.0)
        
    mass_properties.empty_mass : float
        Mass of empty tank structure [kg] (default: 0.0)
        
    secondary_fuel_flow : float
        Secondary fuel flow rate [kg/s] (default: 0.0)
        
    length : float
        Tank length [m] (default: 0.0)
        
    width : float
        Tank width [m] (default: 0.0)
        
    height : float
        Tank height [m] (default: 0.0)
        
    fuel : Component, optional
        Fuel type stored in tank (default: None)

    Notes
    -----
    The wing fuel tank is integrated into the aircraft wing structure,
    utilizing the space between wing spars and ribs for fuel storage.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank
        Base fuel tank class
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Central_Fuel_Tank
        Center section fuel tank
    """
    
    def __defaults__(self):
        """
        Sets default values for wing fuel tank attributes
        """          
        self.tag                         = 'wing_fuel_tank'
        self.fuel_selector_ratio         = 1.0 
        self.mass_properties.empty_mass  = 0.0   
        self.secondary_fuel_flow         = 0.0
        self.length                      = 0.0
        self.width                       = 0.0
        self.height                      = 0.0
        self.fuel                        = None
         

    def append_operating_conditions(self,segment,fuel_line):  
        """
        Append fuel tank operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
        fuel_line : Component
            Connected fuel line component
        """
        append_fuel_tank_conditions(self,segment, fuel_line)  
        return                                          