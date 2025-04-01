# RCAIDE/Library/Methods/Powertrain/Converters/Turbine/append_turbine_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turbine_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turbine_conditions(turbine, segment, propulsor_conditions): 
    """
    Initializes and appends turbine conditions data structures to the propulsor conditions dictionary.
    
    Parameters
    ----------
    turbine : Turbine
        The turbine component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the turbine is operating.
    propulsor_conditions : dict
        Dictionary containing conditions for all propulsion components.
    
    Returns
    -------
    None
        This function modifies the propulsor_conditions dictionary in-place.
    
    Notes
    -----
    This function creates empty Conditions objects for the turbine's inputs and outputs
    within the propulsor_conditions dictionary. These conditions will be populated during
    the mission analysis process.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Turbine.compute_turbine_performance
    """
    ones_row    = segment.state.ones_row 
    propulsor_conditions[turbine.tag]                                       = Conditions()
    propulsor_conditions[turbine.tag].inputs                                = Conditions()
    propulsor_conditions[turbine.tag].outputs                               = Conditions()
    propulsor_conditions[turbine.tag].inputs.fan                            = Conditions()
    propulsor_conditions[turbine.tag].inputs.fan.work_done                  = 0*ones_row(1)  
    propulsor_conditions[turbine.tag].inputs.shaft_power_off_take           = Conditions()
    propulsor_conditions[turbine.tag].inputs.shaft_power_off_take.work_done = 0*ones_row(1) 
    return 