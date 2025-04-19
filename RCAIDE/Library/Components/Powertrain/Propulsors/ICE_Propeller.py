# RCAIDE/Library/Components/Propulsors/ICE_Propeller.py
# 
#  
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports   
from .                import Propulsor  
from RCAIDE.Library.Methods.Powertrain.Propulsors.ICE_Propulsor.unpack_ice_propeller_unknowns   import unpack_ice_propeller_unknowns
from RCAIDE.Library.Methods.Powertrain.Propulsors.ICE_Propulsor.pack_ice_propeller_residuals    import pack_ice_propeller_residuals
from RCAIDE.Library.Methods.Powertrain.Propulsors.ICE_Propulsor.append_ice_propeller_conditions import append_ice_propeller_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.ICE_Propulsor.compute_ice_performance         import compute_ice_performance, reuse_stored_ice_data
from RCAIDE.Library.Methods.Powertrain.Propulsors.ICE_Propulsor.append_ice_residual_and_unknown import  append_ice_residual_and_unknown
 

# ---------------------------------------------------------------------------------------------------------------------- 
# ICE_Propeller
# ---------------------------------------------------------------------------------------------------------------------- 
class ICE_Propeller(Propulsor):
    """
    A propulsion system class that combines an internal combustion engine with a fixed-pitch propeller.
    
    Attributes
    ----------
    tag : str
        Identifier for the propulsion system, defaults to 'ice_propeller'
        
    engine : None or Engine
        The internal combustion engine component
        
    propeller : None or Propeller
        The fixed-pitch propeller component
    
    Notes
    -----
    This class models a conventional propulsion system that pairs an internal 
    combustion engine with a fixed-pitch propeller. Unlike constant-speed propellers, 
    the blade pitch remains fixed during operation, making RPM directly dependent 
    on throttle setting and flight conditions.
    
    Key characteristics:
    - Simpler mechanical design than constant-speed systems
    - RPM varies with airspeed and power setting
    - Optimized for a specific flight condition
    - Lower cost and maintenance requirements
    
    **Definitions**

    'Fixed-Pitch Propeller'
        A propeller with blades set at a fixed angle, optimized for 
        a specific flight regime (typically cruise)
    
    'Blade Angle'
        The angle between the blade's chord line and the plane of rotation,
        measured at a specific radial station
    
    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Propulsors.Propulsor
    RCAIDE.Library.Components.Powertrain.Propulsors.Constant_Speed_ICE_Propeller
    """
    def __defaults__(self):    
        # setting the default values
        self.tag                          = 'ice_propeller'    
        self.engine                       = None
        self.propeller                    = None
        self.engine_diameter              = 0.0      
        self.engine_length                = 0.0
        self.engine_mass                  = 0.0

    def append_operating_conditions(self,segment):
        """
        Appends operating conditions to the segment.
        """
        append_ice_propeller_conditions(self,segment)
        return

    def unpack_propulsor_unknowns(self,segment):  
        """
        Unpacks propulsor unknowns from the segment.
        """
        unpack_ice_propeller_unknowns(self,segment)
        return 

    def pack_propulsor_residuals(self,segment): 
        """
        Packs propulsor residuals into the segment.
        """
        pack_ice_propeller_residuals(self,segment)
        return

    def append_propulsor_unknowns_and_residuals(self,segment):
        """
        Appends propulsor unknowns and residuals to the segment.
        """
        append_ice_residual_and_unknown(self,segment)
        return    
    
    def compute_performance(self,state,center_of_gravity = [[0, 0, 0]]):
        """
        Computes propulsor performance including thrust, moment, and power.
        """
        thrust,moment,power,stored_results_flag,stored_propulsor_tag =  compute_ice_performance(self,state,center_of_gravity)
        return thrust,moment,power,stored_results_flag,stored_propulsor_tag
    
    def reuse_stored_data(ICE_prop,state,network,stored_propulsor_tag,center_of_gravity = [[0, 0, 0]]):
        """
        Reuses stored propulsor data for performance calculations.
        """
        thrust,moment,power  = reuse_stored_ice_data(ICE_prop,state,network,stored_propulsor_tag,center_of_gravity)
        return thrust,moment,power 