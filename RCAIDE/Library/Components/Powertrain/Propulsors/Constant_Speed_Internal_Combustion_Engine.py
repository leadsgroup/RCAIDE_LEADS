# RCAIDE/Library/Components/Powertrain/Propulsors/Constant_Speed_ICE_Propeller.py
# 
#  
# Created:  Mar 2024, M. Clarke
# Modified: Oct 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports   
from RCAIDE.Framework.Core                    import Data
from .                import Propulsor 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Constant_Speed_Internal_Combustion_Engine.append_constant_speed_internal_combustion_engine_conditions  import append_constant_speed_internal_combustion_engine_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Constant_Speed_Internal_Combustion_Engine.compute_constant_speed_internal_combustion_engine_performance  import compute_constant_speed_internal_combustion_engine_performance, reuse_stored_constant_speed_internal_combustion_engine_data 
 
# python imports 
import numpy as np 
# ---------------------------------------------------------------------------------------------------------------------- 
#  Constant_Speed_ICE_Propeller
# ---------------------------------------------------------------------------------------------------------------------- 
class Constant_Speed_Internal_Combustion_Engine(Propulsor):
    """
    A propulsion system class that combines an internal combustion engine with a constant-speed propeller.
    
    Attributes
    ----------
    tag : str
        Identifier for the propulsion system, defaults to 'ice_constant_speed_propeller'
        
    engine : None or Engine
        The internal combustion engine component
        
    propeller : None or Propeller
        The constant-speed propeller component
    
    Notes
    -----
    This class models a propulsion system that pairs an internal combustion engine with 
    a constant-speed propeller. The constant-speed propeller maintains a specified RPM 
    by adjusting blade pitch.
    
    **Definitions**

    'Constant-Speed Propeller'
        A propeller that maintains a constant rotational speed by automatically 
        adjusting blade pitch to match power requirements

    'Governor'
        Mechanical or electronic device that controls propeller pitch to maintain 
        desired RPM
    
    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Propulsors.Propulsor
    """ 
    def __defaults__(self):    
        # setting the default values
        self.tag                   = 'ice_constant_speed_propeller'    
        self.engine                = None
        self.propeller             = None
        self.diameter              = 0.4    
        self.length                = 0.5

    def append_operating_conditions(self,segment):
        append_constant_speed_internal_combustion_engine_conditions(self,segment)
        return

    def unpack_propulsor_unknowns(self,segment):   
        return 

    def pack_propulsor_residuals(self,segment): 
        return        

    def append_propulsor_unknowns_and_residuals(self,segment): 
        return
        
    def compute_performance(self,state,center_of_gravity = [[0, 0, 0]]):
        inputs, outputs, stored_results_flag, stored_propulsor_tag =  compute_constant_speed_internal_combustion_engine_performance(self,state,center_of_gravity)
        return inputs, outputs, stored_results_flag, stored_propulsor_tag
    
    def reuse_stored_data(ICE_cs_prop,state,network,stored_propulsor_tag = None,center_of_gravity = [[0, 0, 0]]): 
        inputs, outputs = reuse_stored_constant_speed_internal_combustion_engine_data(ICE_cs_prop,state,network,stored_propulsor_tag,center_of_gravity)
        return inputs, outputs
  
