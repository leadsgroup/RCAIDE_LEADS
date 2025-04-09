# RCAIDE/Library/Methods/Powertrain/Converters/Generator/append_generator_conditions.py
# 
# Created:  Feb 2025, M. Guidotti 
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_generator_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_generator_conditions(generator, segment, conditions):
    """
    Initializes generator operating conditions for a mission segment.
    
    Parameters
    ----------
    generator : RCAIDE.Library.Components.Converters.Generator
        Generator component with the following attributes:
            - tag : str
                Identifier for the generator
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment with the following attributes:
            - state : Data
                Segment state
                    - ones_row : function
                        Function to create array of ones with specified length
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Energy conditions container where generator conditions will be stored
    
    Returns
    -------
    None
        Results are stored in conditions.converters[generator.tag]
    
    Notes
    -----
    This function initializes the necessary data structures for storing generator
    operating conditions during a mission segment. It creates zero-filled arrays
    for various input and output parameters.
    
    The function initializes the following in conditions.converters[generator.tag]:
        - inputs : Conditions
            Input conditions container
                - torque : numpy.ndarray
                    Input torque [N·m], initialized with zeros
                - power : numpy.ndarray
                    Input mechanical power [W], initialized with zeros
                - omega : numpy.ndarray
                    Angular velocity [rad/s], initialized with zeros
        - outputs : Conditions
            Output conditions container
                - current : numpy.ndarray
                    Output current [A], initialized with zeros
                - voltage : numpy.ndarray
                    Output voltage [V], initialized with zeros
        
    These parameters will be populated during the mission analysis to track the
    generator's performance as it converts mechanical power to electrical power.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Generator.compute_generator_performance
    """
    ones_row                                             = segment.state.ones_row                  
    conditions.converters[generator.tag]                 = Conditions()
    conditions.converters[generator.tag].inputs          = Conditions()
    conditions.converters[generator.tag].inputs.torque   = 0. * ones_row(1) 
    conditions.converters[generator.tag].inputs.power    = 0. * ones_row(1)
    conditions.converters[generator.tag].inputs.omega    = 0. * ones_row(1)
    conditions.converters[generator.tag].outputs         = Conditions()
    conditions.converters[generator.tag].outputs.current = 0. * ones_row(1) 
    conditions.converters[generator.tag].outputs.voltage = 0. * ones_row(1) 
    
    return 

