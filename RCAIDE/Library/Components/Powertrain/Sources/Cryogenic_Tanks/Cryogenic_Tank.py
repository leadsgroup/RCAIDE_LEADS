# RCAIDE/Library/Compoments/Powertrain/Sources/Cryogenic_Tanks/Cryogenic_Tank.py
# 
# 
# Created:  Jan 2025, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Library.Components          import Component
from RCAIDE.Library.Methods.Powertrain.Sources.Cryogenic_Tanks.append_cryogenic_tank_conditions import append_cryogenic_tank_conditions 

# ----------------------------------------------------------------------------------------------------------------------
#  Cryogenic Tank
# ---------------------------------------------------------------------------------------------------------------------     
class Cryogenic_Tank(Component):
    """
    Base class for aircraft cryogenic tank implementations
    
    Attributes
    ----------
    tag : str
        Identifier for the cryogenic tank (default: 'cryogenic_tank')
        
    cryogenic_selector_ratio : float
        Ratio of cryogenic flow allocation (default: 1.0)
        
    mass_properties.empty_mass : float
        Mass of empty tank structure [kg] (default: 0.0)
        
    secondary_cryogenic_flow : float
        Secondary cryogenic flow rate [kg/s] (default: 0.0)
        
    cryogenic : Component, optional
        Cryogenic type stored in tank (default: None)

    Notes
    -----
    The cryogenic tank base class provides common attributes and methods for
    different types of aircraft cryogenic tanks. It handles basic cryogenic storage
    and flow management functionality.
    """
    
    def __defaults__(self):
        """
        Sets default values for cryogenic tank attributes
        """          
        self.tag                         = 'cryogenic_tank'
        self.pressure                    = 1.0 
        self.mass_properties.empty_mass  = 0.0   
        self.croygen_selector_ratio      = 1.0 
        self.secondary_cryogenic_flow    = 0.0 
        self.cryogen                     = None
         

    def append_operating_conditions(self,segment,bus):  
        """
        Append cryogenic tank operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
        bus : Component
            Connected bus component
        """
        append_cryogenic_tank_conditions(self,segment, bus)  
        return                                          