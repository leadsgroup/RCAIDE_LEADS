# RCAIDE/Library/Components/Powertrain/Modulators/Transformer_Rectifier_Unit.py
#  
# Created:  Sep. 2025, M. Guidotti 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# RCAIDE imports
from RCAIDE.Framework.Core import Data  
from .Modulator            import Modulator
from RCAIDE.Library.Methods.Powertrain.Modulators.Transformer_Rectifier_Unit.append_TRU_conditions   import append_TRU_conditions 
from RCAIDE.Library.Methods.Powertrain.Modulators.Transformer_Rectifier_Unit.compute_TRU_performance import compute_TRU_performance
 
# ----------------------------------------------------------------------------------------------------------------------
#  Transformer_Rectifier_Unit Class
# ---------------------------------------------------------------------------------------------------------------------- 
class Transformer_Rectifier_Unit(Modulator):
    """
    Class for modeling Transformer_Rectifier_Units in electric propulsion systems
    
    Attributes
    ----------
    tag : str
        Identifier for the Transformer_Rectifier_Unit (default: 'Transformer_Rectifier_Unit')
        
    efficiency : float
        Power conversion efficiency of the Transformer_Rectifier_Unit (default: 0.0)

    Notes
    -----
    The Transformer_Rectifier_Unit

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Motor
        Electric motor components controlled by the Transformer_Rectifier_Unit
    """
    
    def __defaults__(self):
        """
        Sets default values for Transformer_Rectifier_Unit attributes
        
        Notes
        -----
        Initializes the Transformer_Rectifier_Unit with a default tag and zero efficiency. The efficiency
        should be set to an appropriate value based on the specific Transformer_Rectifier_Unit being modeled.
        """         

        self.tag                   = 'transformer_rectifier_unit'   
        self.turns_ratio           = 1.0
        self.diode_drop            = 0.0
        self.R_load                = 0.0
        self.Idc_set               = 50.0
        self.Pdc_max               = 5000.0

    def append_operating_conditions(self,segment): 
        """
        Append Transformer_Rectifier_Unit operating conditions for a flight segment
        
        Parameters
        ----------
        segment : Segment
            Flight segment containing state conditions
        propulsor : Component
            Propulsor component associated with this Transformer_Rectifier_Unit
            
        Notes
        -----
        Updates the segment conditions with Transformer_Rectifier_Unit-specific parameters including
        power throughput and losses.
        """ 
        append_TRU_conditions(self,segment)
        return 
    
    def compute_performance(self, network, state):

        inputs, outputs, stored_results_flag, stored_modulator_tag = compute_TRU_performance(self, network, state)
        return inputs, outputs, stored_results_flag, stored_modulator_tag
    

    def append_segment_conditions(self,segment):
        energy_conditions  = segment.state.conditions.energy    
        energy_conditions.modulators[self.tag].inputs.power.electrical[:,0]  = 0.0 
        energy_conditions.modulators[self.tag].outputs.power.electrical[:,0] = 0.0     