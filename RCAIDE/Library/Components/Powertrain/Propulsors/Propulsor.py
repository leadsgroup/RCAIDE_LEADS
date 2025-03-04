# RCAIDE/Library/Components/Propulsors/Propuslor.py
#  
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
from RCAIDE.Library.Components                   import Component 

# ----------------------------------------------------------------------------------------------------------------------
#  Propusor
# ---------------------------------------------------------------------------------------------------------------------- 
class Propulsor(Component):
    """
    Base class for all propulsion system components in RCAIDE.
    
    Attributes
    ----------
    tag : str
        Identifier for the propulsor, defaults to 'propulsor'
        
    active : bool
        Flag indicating if the propulsor is operational, defaults to True
        
    wing_mounted : bool
        Flag indicating if the propulsor is mounted on a wing, defaults to True
    
    Notes
    -----
    This class serves as the foundation for all propulsion system implementations 
    in RCAIDE. It provides the basic structure and common attributes needed for:
    - Electric propulsion systems (rotors, ducted fans)
    - Internal combustion engine systems (fixed and constant-speed propellers)
    - Gas turbine systems (turbofans, turbojets, turboprops)
    
    The class inherits from Component, providing basic component functionality
    while adding propulsion-specific features. Derived classes must implement
    their own performance calculation methods and system-specific attributes.
    
    **Definitions**

    'Propulsor'
        Any device that generates thrust or lift force through the manipulation 
        of fluid momentum (air in most cases)
    
    'Wing-mounted'
        Configuration where the propulsor is attached to the aircraft's wing 
        rather than the fuselage or other structures
    
    See Also
    --------
    RCAIDE.Library.Components.Component
    RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor
    RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Ducted_Fan
    RCAIDE.Library.Components.Powertrain.Propulsors.ICE_Propeller
    RCAIDE.Library.Components.Powertrain.Propulsors.Constant_Speed_ICE_Propeller
    """
    
    
    def __defaults__(self):
        """ This sets the default values.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            None
    
            Outputs:
            None
    
            Properties Used:
            None
        """          
        self.tag                          = 'propulsor' 
        self.active                       = True 
        self.wing_mounted                 = True 
        
    