# RCAIDE/Library/Components/Propulsors/ICE_Propeller.py
# 
#  
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports   
from .                import Propulsor  
from RCAIDE.Library.Methods.Powertrain.Propulsors.Internal_Combustion_Engine.unpack_internal_combustion_engine_unknowns   import unpack_internal_combustion_engine_unknowns
from RCAIDE.Library.Methods.Powertrain.Propulsors.Internal_Combustion_Engine.pack_internal_combustion_engine_residuals    import pack_internal_combustion_engine_residuals
from RCAIDE.Library.Methods.Powertrain.Propulsors.Internal_Combustion_Engine.append_internal_combustion_engine_conditions import append_internal_combustion_engine_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Internal_Combustion_Engine.compute_internal_combustion_engine_performance         import compute_internal_combustion_engine_performance, reuse_stored_internal_combustion_engine_data
from RCAIDE.Library.Methods.Powertrain.Propulsors.Internal_Combustion_Engine.append_internal_combustion_engine_residual_and_unknown import append_internal_combustion_engine_residual_and_unknown
 

# ---------------------------------------------------------------------------------------------------------------------- 
# Internal_Combustion_Engine
# ---------------------------------------------------------------------------------------------------------------------- 
class Internal_Combustion_Engine(Propulsor):
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
        self.tag         = 'ice_propeller'    
        self.engine      = None
        self.propeller   = None
        self.diameter    = 0.4    
        self.length      = 0.5

    def append_operating_conditions(self,segment,energy_conditions,noise_conditions=None):
        """
        Appends operating conditions to the segment.
        """
        append_internal_combustion_engine_conditions(self,segment,energy_conditions,noise_conditions)
        return

    def unpack_propulsor_unknowns(self,segment):  
        """
        Unpacks propulsor unknowns from the segment.
        """
        unpack_internal_combustion_engine_unknowns(self,segment)
        return 

    def pack_propulsor_residuals(self,segment): 
        """
        Packs propulsor residuals into the segment.
        """
        pack_internal_combustion_engine_residuals(self,segment)
        return

    def append_propulsor_unknowns_and_residuals(self,segment):
        """
        Appends propulsor unknowns and residuals to the segment.
        """
        append_internal_combustion_engine_residual_and_unknown(self,segment)
        return    
    
    def compute_performance(self,state,fuel_line = None,bus = None,center_of_gravity = [[0, 0, 0]]):
        """
        Computes propulsor performance including thrust, moment, and power.
        """
        thrust,moment,power_mech,power_elec,stored_results_flag,stored_propulsor_tag =  compute_internal_combustion_engine_performance(self,state,fuel_line,bus,center_of_gravity)
        return thrust,moment,power_mech,power_elec,stored_results_flag,stored_propulsor_tag
    
    def reuse_stored_data(ICE_prop, state,network,fuel_line = None, bus = None, stored_propulsor_tag = None,center_of_gravity = [[0, 0, 0]]):
        """
        Reuses stored propulsor data for performance calculations.
        """
        thrust,moment,power_mech,power_elec = reuse_stored_internal_combustion_engine_data(ICE_prop,state,network,fuel_line,bus,stored_propulsor_tag,center_of_gravity)
        return thrust,moment,power_mech,power_elec