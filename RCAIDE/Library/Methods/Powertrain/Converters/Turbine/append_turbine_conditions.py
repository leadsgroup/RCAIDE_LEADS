# RCAIDE/Library/Methods/Powertrain/Converters/Turbine/append_turbine_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turbine_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turbine_conditions(turbine, segment): 
    """
    Initializes and appends turbine conditions data structures to the energy conditions dictionary.
    
    Parameters
    ----------
    turbine : Turbine
        The turbine component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the turbine is operating.
    
    Returns
    -------
    None
        This function modifies the propulsor_conditions dictionary in-place.
    
    Notes
    -----
    This function creates empty Conditions objects for the turbine's inputs and outputs
    within the segment.state.conditions.energy dictionary. These conditions will be populated during
    the mission analysis process.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Turbine.compute_turbine_performance
    """
    ones_row    = segment.state.ones_row  
    segment.state.conditions.energy.converters[turbine.tag]                                             = Conditions()
    segment.state.conditions.energy.converters[turbine.tag].inputs                                      = Conditions()
    segment.state.conditions.energy.converters[turbine.tag].outputs                                     = Conditions()
    segment.state.conditions.energy.converters[turbine.tag].inputs.fan                                  = Conditions()
    segment.state.conditions.energy.converters[turbine.tag].inputs.fan.work_done                        = 0*ones_row(1) 
    return 