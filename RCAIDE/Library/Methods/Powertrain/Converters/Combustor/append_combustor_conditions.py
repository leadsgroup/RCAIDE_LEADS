# RCAIDE/Library/Methods/Powertrain/Converters/Combustor/append_combustor_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_combustor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_combustor_conditions(combustor, segment, energy_conditions):
    """
    Initializes combustor operating conditions for a mission segment.
    
    Parameters
    ----------
    combustor : RCAIDE.Library.Components.Converters.Combustor
        Combustor component with the following attributes:
            - tag : str
                Identifier for the combustor
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment with the following attributes:
            - state : Data
                Segment state
                    - ones_row : function
                        Function to create array of ones with specified length
    energy_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Energy conditions container where combustor conditions will be stored
    
    Returns
    -------
    None
        Results are stored in energy_conditions.converters[combustor.tag]
    
    Notes
    -----
    This function initializes the necessary data structures for storing combustor
    operating conditions during a mission segment. It creates a container for the
    combustor in the energy conditions and initializes the non-dimensional mass ratio
    with ones.
    
    The function initializes the following in energy_conditions.converters[combustor.tag]:
        - inputs : Conditions
            Input conditions container
                - nondim_mass_ratio : numpy.ndarray
                    Non-dimensional mass ratio, initialized with ones
        - outputs : Conditions
            Output conditions container (empty)
    
    The non-dimensional mass ratio represents the ratio of mass flow at the combustor exit
    to the mass flow at the combustor inlet, accounting for the addition of fuel.
    
    **Major Assumptions**
        * Non-dimensional mass ratio is initialized with ones (no fuel addition initially)
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Combustor.compute_combustor_performance
    """
    ones_row    = segment.state.ones_row 
    energy_conditions.converters[combustor.tag]                           = Conditions() 
    energy_conditions.converters[combustor.tag].inputs                    = Conditions() 
    energy_conditions.converters[combustor.tag].inputs.nondim_mass_ratio  = ones_row(1)
    energy_conditions.converters[combustor.tag].outputs                   = Conditions()
    return 