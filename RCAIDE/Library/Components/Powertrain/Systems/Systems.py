# RCAIDE/Library/Components/Powertrain/Systems/Systems.py
# 
# Created:  Mar 2024, M. Clarke 
# Modified: Sep 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
# RCAIDE imports  
from RCAIDE.Framework.Core import Data
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
        self.control     = None
        self.accessories = None 
        self._children   = Data() 

    def __setattr__(self, name, value):
        super().__setattr__(name, value)
        if name.startswith("_"):
            return
        try:
            if isinstance(value, Systems):
                if not hasattr(self, "_children") or self._children is None:
                    super().__setattr__("_children", {})
                self._children[name] = value
            else:
                if hasattr(self, "_children") and name in self._children:
                    self._children.pop(name, None)
        except Exception:
            pass 

    @property
    def power_draw(self) -> float:
        """
        Total power draw for this node: own '_own_power_draw' plus
        the aggregated 'power_draw' of all registered child Systems.
        """
        total = float(getattr(self, "_own_power_draw", 0.0))
        children = getattr(self, "_children", {}) or {}
        for child in children.values():
            try:
                total += float(child.power_draw)
            except Exception:
                pass
        return total

    @power_draw.setter
    def power_draw(self, val: float):
        """Set this node's own/base power draw (does not overwrite children)."""
        self._own_power_draw = float(val)

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

class Hydraulic_System(Systems):
    """
    Subclass representing a hydraulic system.

    Attributes
    ----------
    power_draw : float
        Power consumption of the hydraulic system, defaults to 5.0
    """
    def __defaults__(self):
        super().__defaults__()
        self.power_draw = 0.0
        self.tag        = 'hydraulic_system'

class Pneumatic_System(Systems):
    """
    Subclass representing a pneumatic system.

    Attributes
    ----------
    power_draw : float
        Power consumption of the pneumatic system, defaults to 3.0
    """
    def __defaults__(self):
        super().__defaults__()
        self.power_draw = 0.0
        self.tag        = 'pneumatic_system'

class Avionics_System(Systems):
    """
    Subclass representing an avionic system.

    Attributes
    ----------
    power_draw : float
        Power consumption of the avionic system, defaults to 3.0
    """
    def __defaults__(self):
        super().__defaults__()
        self.power_draw = 0.0
        self.tag        = 'avionics_system'

class Environmental_Control_System(Systems):
    """
    Subclass representing an avionic system.

    Attributes
    ----------
    power_draw : float
        Power consumption of the environmental control system, defaults to 3.0
    """
    def __defaults__(self):
        super().__defaults__()
        self.power_draw = 0.0
        self.tag        = 'environmental_control_system'

        