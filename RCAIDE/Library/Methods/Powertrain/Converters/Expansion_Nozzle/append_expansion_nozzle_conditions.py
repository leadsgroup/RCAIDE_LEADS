# RCAIDE/Library/Methods/Powertrain/Converters/Expansion_Nozzle/append_expansion_nozzle_conditions.py 
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
# append_expansion_nozzle_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_expansion_nozzle_conditions(expansion_nozzle,segment,propulsor_conditions):   
    """
    Initializes and appends expansion nozzle conditions to the propulsor conditions dictionary.
    
    Parameters
    ----------
    expansion_nozzle : ExpansionNozzle
        The expansion nozzle component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the expansion nozzle is operating.
    propulsor_conditions : dict
        Dictionary containing conditions for all propulsion components.
    
    Returns
    -------
    None
    
    Notes
    -----
    This function creates empty Conditions objects for the expansion nozzle's inputs and outputs
    within the propulsor_conditions dictionary. These conditions will be populated during
    the mission analysis process.
    
    The expansion nozzle conditions typically include thermodynamic properties such as
    temperatures, pressures, and velocities at the inlet and outlet of the nozzle.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Expansion_Nozzle.compute_expansion_nozzle_performance
    """
    propulsor_conditions[expansion_nozzle.tag]                      = Conditions()
    propulsor_conditions[expansion_nozzle.tag].inputs               = Conditions()
    propulsor_conditions[expansion_nozzle.tag].outputs              = Conditions() 
    return 