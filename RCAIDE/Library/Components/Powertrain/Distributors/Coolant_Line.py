# RCAIDE/Library/Components/Powertrain/Distributors/Coolant_Line.py 
# 
# Created:  Aug 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                                    import Data
from RCAIDE.Library.Components                                import Component
from RCAIDE.Library.Components.Component                      import Container    
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_distributor_moment_of_inertia import compute_distributor_moment_of_inertia 
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity.compute_distributor_center_of_gravity import compute_distributor_center_of_gravity 

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
        self.tag                                  = 'coolant_line' 
        self.heat_exchangers                      = Container()
        self.reservoirs                           = Container()
        self.connector_weight_factor              = 1.1  
        self.pipe                                 = Data()
        self.pipe.rigid_material                  = RCAIDE.Library.Attributes.Materials.Aluminum()
        self.pipe.flexible_material               = RCAIDE.Library.Attributes.Materials.Stainless_Steel_304()
        self.pipe.flexible_material_ratio         = 0.25
        self.pipe.diameters                       = Data()
        self.pipe.diameters.external              = 0.0
        self.pipe.diameters.internal              = 0.0
        self.insulation                           = Data()
        self.insulation.rigid_material            = RCAIDE.Library.Attributes.Materials.Aluminum() 
        self.insulation.flexible_material         = RCAIDE.Library.Attributes.Materials.Stainless_Steel_304() 
        self.insulation.flexible_material_ratio   = 0.25
        self.insulation.diameters                 = Data()
        self.insulation.diameters.external        = 0.0
        self.insulation.diameters.internal        = 0.0
        self.fuel_probe_unit_mass                 = 0.0
        self.valve_unit_mass                      = 0.0      
        self.boost_pump_unit_mass                 = 0.0

                    
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
            for item in distributor:
                self.identical_battery_modules  =  item.identical_battery_modules
                if 'battery_modules' in item:
                    if not hasattr(self, 'battery_modules'):
                        self.battery_modules = Container()
                    for battery in item.battery_modules:
                        self.battery_modules[battery.tag] = Container() 
                        

    def compute_moments_of_inertia(self,vehicle,center_of_gravity=[[0, 0, 0]]): 
        """
        Computes the moment of inertia tensor for the fuel line.

        Parameters
        ----------
        center_of_gravity : list, optional
            Reference point coordinates for moment calculation, defaults to [[0, 0, 0]] 

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.moments_of_inertia.compute_fuselage_moment_of_inertia
            Implementation of the moment of inertia calculation
        """
        _ , _ = compute_distributor_moment_of_inertia(self,center_of_gravity= center_of_gravity) 
        return
    

    def compute_center_of_gravity(self,vehicle): 
        """
        Computes the center of gravity for the distributor. 

        See Also
        --------
        RCAIDE.Library.Methods.weights.vehicle.center_of_gravity.compute_fuselage_center_of_gravity
            Implementation of the moment of inertia calculation
        """
        _  = compute_distributor_center_of_gravity(self,vehicle) 
        return                        