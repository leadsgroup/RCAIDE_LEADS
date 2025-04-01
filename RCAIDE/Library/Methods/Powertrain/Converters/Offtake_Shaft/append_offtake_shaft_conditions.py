# RCAIDE/Library/Methods/Powertrain/Converters/offtake_shaft/append_offtake_shaft_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_offtake_shaft_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_offtake_shaft_conditions(offtake_shaft, segment, propulsor_conditions): 
    """
    Initializes and appends offtake shaft conditions to the propulsor conditions dictionary.
    
    Parameters
    ----------
    offtake_shaft : OfftakeShaft
        The offtake shaft component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the offtake shaft is operating.
    propulsor_conditions : dict
        Dictionary containing conditions for all propulsion components.
    
    Returns
    -------
    None
    
    Notes
    -----
    This function creates empty Conditions objects for the offtake shaft's inputs and outputs
    within the propulsor_conditions dictionary. These conditions will be populated during
    the mission analysis process.
    
    The offtake shaft conditions typically include mechanical properties such as
    torque, power, and rotational speed at the input and output of the shaft,
    as well as performance metrics like efficiency and power extraction.
    
    An offtake shaft is a component that extracts power from a main shaft (typically
    from a gas turbine engine) to power auxiliary systems or components.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Offtake_Shaft.compute_offtake_shaft_performance
    """
    propulsor_conditions[offtake_shaft.tag]                              = Conditions() 
    propulsor_conditions[offtake_shaft.tag].inputs                       = Conditions() 
    propulsor_conditions[offtake_shaft.tag].outputs                      = Conditions() 
    return 