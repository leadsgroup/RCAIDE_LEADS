# RCAIDE/Library/Components/Powertrain/Distributors/Fuel_Line.py 
# 
# Created:  Jul 2023, M. Clarke 
# Modified: Sep. 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Core                                  import Data, Units
from .Distributor                                           import Distributor   
from RCAIDE.Library.Methods.Powertrain.Distributors.Fuel_Line import * 

# ----------------------------------------------------------------------------------------------------------------------
#  Fuel Line
# ---------------------------------------------------------------------------------------------------------------------- 
class Fuel_Line(Distributor):
    """
    Class for managing fuel distribution between aircraft fuel system components
    
    Attributes
    ----------
    tag : str
        Identifier for the fuel line (default: 'fuel_line')
        
    fuel_tanks : Container
        Collection of fuel tanks connected to this line
        
    assigned_distributors : list
        List of distributors (inherited from Distributor) that reference this
        fuel line by tag
        
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
        self.active                               = True 
        self.domain                               = 'chemical'
        self.efficiency                           = 1.0 
        self.length                               = 1  
        self.working_fluid                        = None    
        self.additional_line_flow_rate            = 0.0
        self.efficiency                           = 1.0
        self.valve_unit_mass                      = 3 * Units.lbs
        self.fuel_probe_unit_mass                 = 2.5 * Units.lbs
        self.boost_pump_unit_mass                 = 12.5 * Units.lbs
        self.venting_system_length                = 0.0 
        self.pipe                                 = Data()
        self.pipe.surface_roughness               = 0.015
        self.pipe.rigid_material                  = RCAIDE.Library.Attributes.Materials.Aluminum()
        self.pipe.flexible_material               = RCAIDE.Library.Attributes.Materials.Stainless_Steel_304()
        self.pipe.flexible_material_ratio         = 0.25
        self.pipe.diameters                       = Data()
        self.pipe.diameters.external              = 0.625 *  Units.inches 
        self.pipe.diameters.internal              = 0.625 *  Units.inches -  (2 * 0.035)*  Units.inches
        self.pipe.k_factors                       = Data() 
        self.pipe.k_factors.bend_90_deg           =  0 
        self.pipe.k_factors.bend_45_deg           = 0.4 
        self.pipe.k_factors.pipe_entrance_rounded = 0.05
        self.pipe.k_factors.pipe_exit             = 1.0 
        self.insulation                           = Data()
        self.insulation.rigid_material            = RCAIDE.Library.Attributes.Materials.Aluminum() 
        self.insulation.flexible_material         = RCAIDE.Library.Attributes.Materials.Stainless_Steel_304() 
        self.insulation.flexible_material_ratio   = 0.25
        self.insulation.diameters                 = Data()
        self.insulation.diameters.external        = 0.0
        self.insulation.diameters.internal        = 0.0 
        self.pump                                 = RCAIDE.Library.Components.Powertrain.Converters.Pump() 

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
    
    def compute_distribution_losses(self, component_conditions, state, network):
        compute_fuel_line_distribution_losses(self, component_conditions, state, network) 
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