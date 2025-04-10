# RCAIDE/Library/Methods/Powertrain/Converters/Supersonic_Nozzle/append_supersonic_nozzle_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
# append_supersonic_nozzle_conditions 
# ----------------------------------------------------------------------------------------------------------------------    
def append_supersonic_nozzle_conditions(supersonic_nozzle, segment, energy_conditions): 
    """
    Initializes and appends supersonic nozzle conditions data structures to the propulsor conditions dictionary.
    
    Parameters
    ----------
    supersonic_nozzle : SupersonicNozzle
        The supersonic nozzle component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the supersonic nozzle is operating.
    energy_conditions : dict
        Dictionary containing conditions for all propulsion components.
    
    Returns
    -------
    None
        This function modifies the propulsor_conditions dictionary in-place.
    
    Notes
    -----
    This function creates empty Conditions objects for the supersonic nozzle's inputs and outputs
    within the propulsor_conditions dictionary. These conditions will be populated during
    the mission analysis process.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Supersonic_Nozzle.compute_supersonic_nozzle_performance
    """
    energy_conditions.converters[supersonic_nozzle.tag]                     = Conditions()
    energy_conditions.converters[supersonic_nozzle.tag].inputs              = Conditions()
    energy_conditions.converters[supersonic_nozzle.tag].outputs             = Conditions()
    return 