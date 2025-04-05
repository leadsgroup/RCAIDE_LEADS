# RCAIDE/Library/Methods/Powertrain/Converters/compressor/append_compressor_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_compressor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_compressor_conditions(compressor,segment,energy_conditions): 
    """
    Initializes empty condition containers for compressor analysis in the propulsion system.
    
    Parameters
    ----------
    compressor : Compressor
        The compressor component being analyzed
    segment : Segment
        The mission segment being analyzed
    propulsor_conditions : Conditions
        Container for storing propulsion system conditions
    
    Returns
    -------
    None
    
    Notes
    -----
    This function creates empty Conditions containers that will be populated
    during compressor performance calculations with thermodynamic states
    and operating parameters.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Compressor.compute_compressor_performance
    """
    
    ones_row    = segment.state.ones_row 
    energy_conditions.converters[compressor.tag]                                   = Conditions()
    energy_conditions.converters[compressor.tag].inputs                            = Conditions()
    energy_conditions.converters[compressor.tag].outputs                           = Conditions()
    energy_conditions.converters[compressor.tag].outputs.external_shaft_work_done  = 0*ones_row(1)
    energy_conditions.converters[compressor.tag].outputs.external_electrical_power = 0*ones_row(1)
    return 