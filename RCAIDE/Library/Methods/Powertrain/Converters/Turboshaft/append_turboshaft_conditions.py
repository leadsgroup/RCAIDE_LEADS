# RCAIDE/Library/Methods/Powertrain/Converters/Turboshaft/append_turboshaft_conditions.py
# 
# Created:  Jun 2024, M. Clarke  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turboshaft_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turboshaft_conditions(turboshaft, segment):
    """
    Appends data structures for storing turboshaft operating conditions during mission analysis.
    
    Parameters
    ----------
    turboshaft : RCAIDE.Components.Energy.Converters.Turboshaft
        The turboshaft component for which conditions are being appended
    segment : RCAIDE.Analyses.Mission.Segments
        The mission segment being evaluated
        
    Returns
    -------
    None
        This function modifies the segment.state.conditions.energy and segment.state.conditions.noise objects in-place
    
    Notes
    -----
    This function initializes the condition structure for a turboshaft engine with zero values
    for throttle, thrust vector angle, power, and fuel flow rate. It then recursively calls
    the append_operating_conditions method for all subcomponents of the turboshaft.
    
    The function creates a hierarchical data structure that mirrors the component hierarchy
    of the turboshaft engine, allowing for detailed tracking of operating conditions at each
    level during mission analysis.
    
    **Major Assumptions**
        * The segment state contains a ones_row method for creating arrays
        * All subcomponents have properly implemented append_operating_conditions methods
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Turboshaft.compute_turboshaft_performance
    RCAIDE.Library.Components.Powertrain.Converters.Turboshaft
    """
    ones_row    = segment.state.ones_row
    
    segment.state.conditions.energy.converters[turboshaft.tag]                               = Conditions() 
    segment.state.conditions.energy.converters[turboshaft.tag].throttle                      = 0. * ones_row(1)     
    segment.state.conditions.energy.converters[turboshaft.tag].commanded_thrust_vector_angle = 0. * ones_row(1)   
    segment.state.conditions.energy.converters[turboshaft.tag].power                         = 0. * ones_row(1)
    segment.state.conditions.energy.converters[turboshaft.tag].fuel_mass_flow_rate           = 0. * ones_row(1)
    segment.state.conditions.energy.converters[turboshaft.tag].inputs                        = Conditions()
    segment.state.conditions.energy.converters[turboshaft.tag].outputs                       = Conditions()
    
    segment.state.conditions.energy.converters[turboshaft.tag].power                   = Conditions()
    segment.state.conditions.energy.converters[turboshaft.tag].power.propulsive        = 0 * ones_row(1)
    segment.state.conditions.energy.converters[turboshaft.tag].power.mechanical        = 0 * ones_row(1)
    segment.state.conditions.energy.converters[turboshaft.tag].power.electrical        = 0 * ones_row(1)
    segment.state.conditions.energy.converters[turboshaft.tag].power.chemical          = 0 * ones_row(1)
    segment.state.conditions.energy.converters[turboshaft.tag].power.pneumatic         = 0 * ones_row(1)
    segment.state.conditions.energy.converters[turboshaft.tag].power.hydraulic         = 0 * ones_row(1)
    segment.state.conditions.energy.converters[turboshaft.tag].power.thermal           = 0 * ones_row(1)
 
    for tag, item in  turboshaft.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment) 
            for sub_tag, sub_item in  item.items(): 
                if issubclass(type(sub_item), RCAIDE.Library.Components.Component):
                    sub_item.append_operating_conditions(segment) 
    return 