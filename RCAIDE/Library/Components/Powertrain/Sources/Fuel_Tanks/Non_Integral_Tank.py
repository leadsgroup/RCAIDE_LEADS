# RCAIDE/Library/Components/Powertrain/Energy/Sources/Fuel_Tanks/Non_Integral_Tank.py
# 
# 
# Created:  September 2024, A. Molloy and M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from .Fuel_Tank  import Fuel_Tank 
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.append_fuel_tank_conditions import append_fuel_tank_conditions 

# ----------------------------------------------------------------------------------------------------------------------
#  Fuel Tank
# ---------------------------------------------------------------------------------------------------------------------    
class Non_Integral_Tank(Fuel_Tank):
    """
    Class for modeling a non integral tank characteristics and behavior
    
    Attributes
    ----------
    tag : str
        Identifier for the fuel tank (default: 'Central_fuel_tank')
        
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
    The central fuel tank is located in the aircraft's center section,
    often integrated with the wing box or fuselage structure. 

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank
        Base fuel tank class
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank
        Integral fuel tank class
    """
    
    def __defaults__(self):
        """
        Sets default values for central fuel tank attributes
        """          
        self.tag                         = 'non_integral_tank'
        self.fuel_selector_ratio         = 1.0 
        self.mass_properties.empty_mass  = 0.0   
        self.secondary_fuel_flow         = 0.0 
        self.fuel                        = None 

    def __init__ (self, compoment=None):
        """
        Initialize  
        """ 
        if compoment is not None:
            if isinstance(compoment, RCAIDE.Library.Components.Wings.Wing): 
                self.wing      = compoment 
            if isinstance(compoment, RCAIDE.Library.Components.Fuselages.Fuselage): 
                self.fuselage     = compoment           
       
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