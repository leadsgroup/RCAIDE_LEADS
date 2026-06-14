# RCAIDE/Library/Components/Powertrain/Systems/Systems.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports   
from RCAIDE.Library.Components import Component 
from RCAIDE.Library.Methods.Powertrain.Systems.append_systems_conditions import append_systems_conditions
 
# ----------------------------------------------------------------------------------------------------------------------
# System
# ----------------------------------------------------------------------------------------------------------------------            
class Systems(Component):
    """
    Base class for aircraft systems providing core functionality for modeling 
    onboard equipment and subsystems.

    Attributes
    ----------
    tag : str
        Unique identifier for the system component, defaults to 'System'
        
    origin : list
        3D coordinates [x, y, z] defining the system's reference point, 
        defaults to [[0.0, 0.0, 0.0]]
        
    control : Data
        Control system interface parameters, defaults to None
        
    accessories : Data
        Associated auxiliary components and equipment, defaults to None

    Notes
    -----
    The system class serves as the foundation for modeling various aircraft systems:
        * Avionics and electronics
        * Environmental control systems
        * Hydraulic systems
        * Fuel systems
        * Auxiliary power units
    
    **Major Assumptions**
        * Systems are treated as point masses at their origin
        * Control interfaces are simplified
        * No dynamic response modeling
    
    **Definitions**

    'Origin'
        Reference point for system location and mass properties
        
    'Control Interface'
        Parameters defining how the system interacts with aircraft controls

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Systems.Avionics
        Implementation for aircraft avionics
    """  
    def __defaults__(self): 
        """
        Sets default values for the system attributes.
        """        
        self.tag         = 'System' 
        self.power_draw  = 0.0
        self.length      = 0
        self.width       = 0
        self.height      = 0
        self.control     = None
        self.accessories = None 
        self.mass_properties.calculated_flag = False

    def append_operating_conditions(self, segment, bus): 
        """
        Adds operating conditions for the avionics system to a mission segment.

        Parameters
        ----------
        segment : Data
            Mission segment to which conditions are being added
        bus : Data
            Electrical bus supplying power to the avionics
        """
        append_systems_conditions(self, segment, bus)
        return         