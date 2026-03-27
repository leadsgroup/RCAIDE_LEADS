# RCAIDE/Library/Components/Powertrain/Distributors/Fuel_Line.py 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core                                    import Data, Units
from RCAIDE.Library.Components                                import Component
from RCAIDE.Library.Components.Component                      import Container    
from RCAIDE.Library.Methods.Powertrain.Distributors.Fuel_Line import * 
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_distributor_moment_of_inertia import compute_distributor_moment_of_inertia 
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity.compute_distributor_center_of_gravity import compute_distributor_center_of_gravity  

# ----------------------------------------------------------------------------------------------------------------------
#  Fuel Line
# ---------------------------------------------------------------------------------------------------------------------- 
class Fuel_Line(Component):
    """
    Class for managing fuel distribution between aircraft fuel system components
    
    Attributes
    ----------
    tag : str
        Identifier for the fuel line (default: 'fuel_line')
        
    fuel_tanks : Container
        Collection of fuel tanks connected to this line
        
    assigned_propulsors : list
        List of propulsion systems supplied by this fuel line
        
    active : bool
        Flag indicating if the fuel line is operational (default: True)
        
    efficiency : float
        Fuel transfer efficiency (default: 1.0)

    Notes
    -----
    The fuel line manages fuel distribution between tanks and engines, handling
    fuel transfer and flow control. It supports multiple fuel tanks and propulsors
    in various aircraft configurations.

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks
        Fuel tank components
    RCAIDE.Library.Components.Powertrain.Propulsors
        Aircraft propulsion system components
    """ 
    
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """          
        self.tag                                  = 'fuel_line'  
        self.fuel_tanks                           = Container() 
        self.assigned_propulsors                  = [] 
        self.assigned_converters                  = []
        self.converters                           = Container()       
        self.active                               = True 
        self.additional_line_flow_rate            = 0.0
        self.efficiency                           = 1.0
        self.valve_unit_mass                      = 3 * Units.lbs
        self.fuel_probe_unit_mass                 = 2.5 * Units.lbs
        self.boost_pump_unit_mass                 = 12.5 * Units.lbs
        self.venting_system_length                = 0.0
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

    def append_operating_conditions(self, segment):
        """
        Append operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing operating conditions
        """
        append_fuel_line_conditions(self, segment)
        return

        
    def append_segment_conditions(self, segment):
        """
        Append segment-specific conditions to the bus
        
        Parameters
        ----------
        conditions : Data
            Container for segment conditions
        segment : Segment
            Flight segment data
        """
        append_fuel_line_segment_conditions(self, segment)
        return    

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
        _ , _ = compute_distributor_moment_of_inertia(self,vehicle,center_of_gravity= center_of_gravity) 
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