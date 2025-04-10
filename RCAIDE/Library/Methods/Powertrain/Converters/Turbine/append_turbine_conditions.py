# RCAIDE/Library/Methods/Powertrain/Converters/Turbine/append_turbine_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turbine_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turbine_conditions(turbine, segment, energy_conditions): 
    """
    Initializes and appends turbine conditions data structures to the energy conditions dictionary.
    
    Parameters
    ----------
    turbine : Turbine
        The turbine component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the turbine is operating.
    energy_conditions : dict
        Dictionary containing conditions for all propulsion components.
    
    Returns
    -------
    None
        This function modifies the propulsor_conditions dictionary in-place.
    
    Notes
    -----
    This function creates empty Conditions objects for the turbine's inputs and outputs
    within the energy_conditions dictionary. These conditions will be populated during
    the mission analysis process.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Turbine.compute_turbine_performance
    """
    ones_row    = segment.state.ones_row 
    energy_conditions.converters[turbine.tag]                                             = Conditions()
    energy_conditions.converters[turbine.tag].inputs                                      = Conditions()
    energy_conditions.converters[turbine.tag].outputs                                     = Conditions()
    energy_conditions.converters[turbine.tag].inputs.fan                                  = Conditions()
    energy_conditions.converters[turbine.tag].inputs.fan.work_done                        = 0*ones_row(1)
    return 