# RCAIDE/Library/Components/Propulsors/Constant_Speed_ICE_Propeller.py
# 
#  
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports   
from .                import Propulsor 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Constant_Speed_ICE_Propulsor.append_ice_cs_propeller_conditions  import append_ice_cs_propeller_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Constant_Speed_ICE_Propulsor.compute_cs_ice_performance          import compute_cs_ice_performance, reuse_stored_ice_cs_prop_data
 
# ---------------------------------------------------------------------------------------------------------------------- 
#  Constant_Speed_ICE_Propeller
# ---------------------------------------------------------------------------------------------------------------------- 
class Constant_Speed_ICE_Propeller(Propulsor):
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
        self.tag                          = 'ice_constant_speed_propeller'    
        self.engine                       = None
        self.propeller                    = None  
          

    def append_operating_conditions(self,segment):
        append_ice_cs_propeller_conditions(self,segment)
        return

    def unpack_propulsor_unknowns(self,segment):   
        return 

    def pack_propulsor_residuals(self,segment): 
        return        

    def append_propulsor_unknowns_and_residuals(self,segment): 
        return
        
    def compute_performance(self,state,center_of_gravity = [[0, 0, 0]]):
        thrust,moment,power,stored_results_flag,stored_propulsor_tag =  compute_cs_ice_performance(self,state,center_of_gravity)
        return thrust,moment,power,stored_results_flag,stored_propulsor_tag
    
    def reuse_stored_data(ICE_cs_prop,state,network,stored_propulsor_tag,center_of_gravity = [[0, 0, 0]]):
        thrust,moment,power  = reuse_stored_ice_cs_prop_data(ICE_cs_prop,state,network,stored_propulsor_tag,center_of_gravity)
        return thrust,moment,power           
 
