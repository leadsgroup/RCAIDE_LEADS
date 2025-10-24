# RCAIDE/Library/Components/Powertrain/Converters/Auxiliary_Power_Unit.py
# 
#  
# Created:  Sep. 2025, M. Guidotti 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
## RCAIDE imports
from RCAIDE.Framework.Core                  import Data 
from .Turboelectric_Generator               import Turboelectric_Generator
from RCAIDE.Library.Methods.Powertrain.Converters.Turboelectric_Generator import compute_turboelectric_generator_performance, append_turboelectric_generator_conditions 
 
# ----------------------------------------------------------------------
#  Auxiliary_Power_Unit
# ----------------------------------------------------------------------
class Auxiliary_Power_Unit(Turboelectric_Generator):
    """
    A Auxiliary_Power_Unit propulsion system model that simulates the performance of a Auxiliary_Power_Unit.
   
    """ 
    def __defaults__(self):

        self.tag                       = 'Auxiliary_Power_Unit'
        self.gearbox                   = Data()
        self.gearbox.gear_ratio        = None  
        self.inverse_calculation       = False

    def append_operating_conditions(self,segment):
        """
        Appends operating conditions of the combustor.
        """ 
        append_turboelectric_generator_conditions(self,segment)
        return
    
    def compute_performance(self, state):
        inputs, outputs = compute_turboelectric_generator_performance(self,state)
        return inputs, outputs

