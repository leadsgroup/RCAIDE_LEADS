# RCAIDE/Library/Methods/Powertrain/Propulsors/Constant_Speed_ICE_Propulsor/append_ice_cs_propeller_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_ice_cs_propeller_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_ice_cs_propeller_conditions(ice_cs_propeller,segment):  
    """
    Initializes the state conditions for a constant speed ICE-driven propeller system.
    
    Parameters
    ----------
    ice_cs_propeller : RCAIDE.Library.Components.Propulsors.Constant_Speed_ICE_Propulsor
        The constant speed ICE propeller system object
    segment : RCAIDE.Framework.Mission.Segments.Segment
        The mission segment to which the conditions will be appended
        
    Returns
    -------
    None
        
    Notes
    -----
    This function initializes various performance parameters for the ICE-propeller
    system including thrust, power, fuel flow, and RPM states. All values except
    RPM are initialized to zero and updated during mission analysis.
    
    The following conditions are initialized:
        - throttle
        - commanded thrust vector angle
        - thrust vector
        - power
        - moment
        - fuel flow rate
        - RPM (copied from segment conditions)
        - inputs/outputs containers
        - noise conditions
    
    See Also
    --------
    RCAIDE.Library.Components.Propulsors.Constant_Speed_ICE_Propeller
    RCAIDE.Framework.Mission.Common.Conditions
    """
    
    ones_row    = segment.state.ones_row                  
    segment.state.conditions.energy[ice_cs_propeller.tag]                               = Conditions()  
    segment.state.conditions.energy[ice_cs_propeller.tag].throttle                      = 0. * ones_row(1)      
    segment.state.conditions.energy[ice_cs_propeller.tag].commanded_thrust_vector_angle = 0. * ones_row(1)  
    segment.state.conditions.energy[ice_cs_propeller.tag].thrust                        = 0. * ones_row(3) 
    segment.state.conditions.energy[ice_cs_propeller.tag].power                         = 0. * ones_row(1) 
    segment.state.conditions.energy[ice_cs_propeller.tag].moment                        = 0. * ones_row(3) 
    segment.state.conditions.energy[ice_cs_propeller.tag].fuel_flow_rate                = 0. * ones_row(1)
    segment.state.conditions.energy[ice_cs_propeller.tag].rpm                           = segment.state.conditions.energy.rpm * ones_row(1)      
    segment.state.conditions.energy[ice_cs_propeller.tag].inputs                        = Conditions()
    segment.state.conditions.energy[ice_cs_propeller.tag].outputs                       = Conditions() 
    segment.state.conditions.noise[ice_cs_propeller.tag]                                = Conditions()  
    return 