# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan_Propulsor/append_turbofan_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turbofan_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turbofan_conditions(turbofan,segment):  
    """
    Initializes the turbofan-specific conditions in the mission segment state. This function creates and populates the necessary 
    data structures to store energy, noise, and operational parameters for a turbofan propulsion system during mission analysis.
    
    Parameters
    ----------
    turbofan : RCAIDE.Library.Components.Propulsors.Turbofan
        The turbofan propulsion system instance
    segment : RCAIDE.Framework.Mission.Segments.Segment
        The mission segment instance where conditions will be appended
    
    Returns
    -------
    None
        Function modifies the segment state directly
    
    Notes
    -----
    This function initializes the following condition groups:
        * Energy conditions (thrust, power, fuel flow, etc.)
        * Noise conditions (core nozzle, fan nozzle, fan)
        * Input/output conditions for the turbofan system
    
    The conditions are initialized as zero arrays with appropriate dimensions based on the segment's state.
    
    **Major Assumptions**
        * All conditions are initially set to zero
        * Thrust vector is represented in 3D space
        * Single throttle setting per time step
        
    See Also
    --------
    RCAIDE.Framework.Mission.Segments.Segment
    RCAIDE.Library.Components.Propulsors.Turbofan
    """ 
    ones_row    = segment.state.ones_row                  
    segment.state.conditions.energy[turbofan.tag]                               = Conditions()  
    segment.state.conditions.energy[turbofan.tag].throttle                      = 0. * ones_row(1)      
    segment.state.conditions.energy[turbofan.tag].commanded_thrust_vector_angle = 0. * ones_row(1)  
    segment.state.conditions.energy[turbofan.tag].thrust                        = 0. * ones_row(3) 
    segment.state.conditions.energy[turbofan.tag].power                         = 0. * ones_row(1) 
    segment.state.conditions.energy[turbofan.tag].moment                        = 0. * ones_row(3) 
    segment.state.conditions.energy[turbofan.tag].fuel_flow_rate                = 0. * ones_row(1)
    segment.state.conditions.energy[turbofan.tag].inputs                        = Conditions()
    segment.state.conditions.energy[turbofan.tag].outputs                       = Conditions() 
    segment.state.conditions.noise[turbofan.tag]                                = Conditions() 
    segment.state.conditions.noise[turbofan.tag].turbofan                       = Conditions() 
    segment.state.conditions.noise[turbofan.tag].turbofan.core_nozzle           = Conditions() 
    segment.state.conditions.noise[turbofan.tag].turbofan.fan_nozzle            = Conditions() 
    segment.state.conditions.noise[turbofan.tag].turbofan.fan                   = Conditions()  
    return 