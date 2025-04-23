# RCAIDE/Library/Components/Powertrain/Distributors/Coolant_Line.py 
# 
# Created:  Aug 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports  
from RCAIDE.Library.Components                                import Component
from RCAIDE.Library.Components.Component                      import Container    

# ----------------------------------------------------------------------------------------------------------------------
#  Coolant Line
# ---------------------------------------------------------------------------------------------------------------------- 
class Coolant_Line(Component):
    """
    Class for modeling coolant distribution lines in thermal management systems
    
    This class represents coolant distribution lines that connect various thermal 
    management components like heat exchangers, reservoirs, and battery cooling systems.

    Attributes
    ----------
    tag : str
        Identifier for the coolant line (default: 'coolant_line')
        
    heat_exchangers : Container
        Collection of heat exchangers connected to the coolant line
        
    reservoirs : Container
        Collection of coolant reservoirs in the system
        
    active : bool
        Flag indicating if the coolant line is operational (default: True)
        
    efficiency : float
        Distribution efficiency of the coolant line (default: 1.0)
        
    battery_modules : Container, optional
        Collection of battery cooling systems, created when batteries are present
        
    identical_battery_modules : bool, optional
        Flag indicating if all battery modules use identical cooling systems

    Notes
    -----
    The coolant line serves as a connection framework between thermal management
    components. It manages the routing of coolant between heat sources (like batteries)
    and heat sinks (like heat exchangers and reservoirs).

    **Definitions**

    'Container'
        A specialized dictionary-like object for storing RCAIDE components
        
    'Heat Acquisition System'
        Components and geometry for removing heat from battery modules

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Distributors.Heat_Exchanger
        Heat exchanger components for thermal management
    RCAIDE.Library.Components.Powertrain.Distributors.Reservoir
        Coolant reservoir components
    """
    
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """          
        self.tag                            = 'coolant_line' 
        self.heat_exchangers                = Container()
        self.reservoirs                     = Container() 

                    
    def __init__ (self, distributor=None):
        """
        Initialize coolant line and set up containers for thermal management components
        
        This method initializes empty containers for heat acquisition systems for battery
        modules present on a particular bus.

        Parameters
        ----------
        distributor : Component, optional
            Component containing battery modules that need thermal management
            
        Notes
        -----
        When a distributor with battery modules is provided, the method creates
        containers to store the cooling system components for each battery.
        """               
        self.active                        = True 
        self.efficiency                    = 1.0
        if distributor is not None:
            for tag, item in  distributor.items():
                self.identical_battery_modules  =  distributor.identical_battery_modules
                if tag == 'battery_modules':
                    if not hasattr(self, 'battery_modules'):
                        self.battery_modules = Container()
                    for battery in item:
                        self.battery_modules[battery.tag] = Container()