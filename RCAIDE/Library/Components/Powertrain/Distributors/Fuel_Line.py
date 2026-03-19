# RCAIDE/Library/Components/Powertrain/Distributors/Fuel_Line.py 
# 
# Created:  Jul 2023, M. Clarke 
# Modified: Sep. 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Core                                    import Data
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
        self.tag                           = 'fuel_line'  
        self.active                        = True 
        self.domain                        = 'chemical'
        self.efficiency                    = 1.0
        self.inner_diameter                = 0.03
        self.outer_diameter                = 0.05
        self.length                        = 1  
        self.pressure                      = 150000.0  # Pa

    def unpack_unknowns(self,segment):
        return 

    def pack_residuals(self,segment): 
        return        

    def append_unknowns_and_residuals(self,segment):
        return
    
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

    def compute_performance(self, state):

        inputs = Data()
        outputs = Data()

        inputs.power.mechanical  = state.conditions.energy.distributors[self.tag].inputs.power.mechanical
        inputs.power.electrical  = state.conditions.energy.distributors[self.tag].inputs.power.electrical
        inputs.power.chemical    = state.conditions.energy.distributors[self.tag].inputs.power.chemical  
        inputs.power.pneumatic   = state.conditions.energy.distributors[self.tag].inputs.power.pneumatic 
        inputs.power.hydraulic   = state.conditions.energy.distributors[self.tag].inputs.power.hydraulic 
        inputs.power.thermal     = state.conditions.energy.distributors[self.tag].inputs.power.thermal  

        outputs.power.mechanical = state.conditions.energy.distributors[self.tag].outputs.power.mechanical
        outputs.power.electrical = state.conditions.energy.distributors[self.tag].outputs.power.electrical
        outputs.power.chemical   = state.conditions.energy.distributors[self.tag].outputs.power.chemical  
        outputs.power.pneumatic  = state.conditions.energy.distributors[self.tag].outputs.power.pneumatic 
        outputs.power.hydraulic  = state.conditions.energy.distributors[self.tag].outputs.power.hydraulic 
        outputs.power.thermal    = state.conditions.energy.distributors[self.tag].outputs.power.thermal  

        return inputs, outputs