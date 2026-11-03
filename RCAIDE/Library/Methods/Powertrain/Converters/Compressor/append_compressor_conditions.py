# RCAIDE/Library/Methods/Powertrain/Converters/compressor/append_compressor_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_compressor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_compressor_conditions(compressor,segment): 
    """
    Initializes empty condition containers for compressor analysis in the propulsion system.
    
    Parameters
    ----------
    compressor : Compressor
        The compressor component being analyzed
    segment : Segment
        The mission segment being analyzed
    energy_conditions : Conditions
        Container for storing energy system conditions
    
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
    segment.state.conditions.energy.converters[compressor.tag]                                   = Conditions()
    segment.state.conditions.energy.converters[compressor.tag].inputs                            = Conditions()
    segment.state.conditions.energy.converters[compressor.tag].outputs                           = Conditions()
    segment.state.conditions.energy.converters[compressor.tag].outputs.external_shaft_work_done  = 0*ones_row(1)
    segment.state.conditions.energy.converters[compressor.tag].outputs.external_electrical_power = 0*ones_row(1) 
    conditions       = segment.state.conditions.energy.converters[compressor.tag]
    rows             = ones_row(1).shape[0]
    for node in (conditions.inputs, conditions.outputs):   # station states, NaN until computed
        node.stagnation_temperature = np.full((rows, 1), np.nan)
        node.stagnation_pressure    = np.full((rows, 1), np.nan)
    conditions.omega = np.full((rows, 1), np.nan)
    return 