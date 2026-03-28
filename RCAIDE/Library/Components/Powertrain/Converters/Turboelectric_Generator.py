# RCAIDE/Library/Components/Powertrain/Converters/Turboelectric_Generator.py 
# 
#  
# Created:  Jan 2025, M. Clarke 
# Modified: Oct. 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
from RCAIDE.Framework.Core                  import Data 
from .Converter                             import Converter
from RCAIDE.Library.Methods.Powertrain.Converters.Turboelectric_Generator   import design_turboelectric_generator  
from RCAIDE.Library.Methods.Powertrain.Converters.Turboelectric_Generator.append_turboelectric_generator_conditions      import append_turboelectric_generator_conditions  
from RCAIDE.Library.Methods.Powertrain.Converters.Turboelectric_Generator.compute_turboelectric_generator_performance    import compute_turboelectric_generator_performance, reuse_stored_turboelectric_generator_data
 
# ----------------------------------------------------------------------
#  Turboelectric_Generator
# ----------------------------------------------------------------------
class Turboelectric_Generator(Converter):
    """
    A Turboelectric_Generator propulsion system model that simulates the performance of a Turboelectric_Generator engine.
   

    Attributes
    ----------
    tag : str
        Identifier for the shaft engine. Default is 'turboshaft'. 
        
    turboshaft : Component
        Turboshaft component. Default is the Turboshaft Class.
        
    generator : Component
        Generator component. Default is DC_Generator Class.
        
    gearbox : Component
        Gearbox data structure. Default is None. 
        
    inverse_calculation : Component
        Flag that determines the how calculations are performed. Default is False    

    Notes
    -----
    The Turboelectric_Generator class inherits from the Turboshaft class and implements
    methods for computing Turboelectric_Generator engine performance. Unlike other gas turbine
    engines that produce thrust, a Turboelectric_Generator engine's primary output is shaft
    power, typically used to drive a helicopter rotor or other mechanical systems. 

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Propulsors.Turboshaft 
    """ 
    def __defaults__(self):
        # setting the default values
        self.tag                       = 'turboelectric_generator'
        self.turboshaft                = None
        self.generator                 = None
        self.gearbox                   = Data()
        self.gearbox.gear_ratio        = None  
        self.inverse_calculation       = False 
        self.assigned_converters       = Data()
        
    def initialize(self, network): 
        design_turboelectric_generator(self, network) 
        return

    def append_operating_conditions(self,segment): 
        """
        Appends operating conditions of the segment.
        """  
        append_turboelectric_generator_conditions(self,segment) 
        return
 
    def compute_performance(self,state,fuel_line = None,bus = None):
        """
        Computes Turboelectric_Generator performance including power.
        """
        inputs, outputs, stored_results_flag, stored_converter_tag = compute_turboelectric_generator_performance(self,state,fuel_line, bus)
        return inputs, outputs, stored_results_flag, stored_converter_tag
    
    def reuse_stored_data(self,state,network,stored_conveter_tag,fuel_line, bus):
        inputs, outputs  = reuse_stored_turboelectric_generator_data(self,state,network,stored_conveter_tag,fuel_line, bus)
        return inputs, outputs
