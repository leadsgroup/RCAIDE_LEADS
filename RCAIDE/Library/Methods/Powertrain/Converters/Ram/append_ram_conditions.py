# RCAIDE/Library/Methods/Powertrain/Converters/Ram/append_ram_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_ram_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_ram_conditions(ram, segment, energy_conditions):
    """
    Initializes ram air converter operating conditions for a mission segment.
    
    Parameters
    ----------
    ram : RCAIDE.Library.Components.Converters.Ram
        Ram air converter component with the following attributes:
            - tag : str
                Identifier for the ram air converter
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment with the following attributes:
            - state : Data
                Segment state
                    - ones_row : function
                        Function to create array of ones with specified length
    energy_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Energy conditions container where ram air converter conditions will be stored
    
    Returns
    -------
    None
    
    Notes
    -----
    This function initializes the necessary data structures for storing ram air converter
    operating conditions during a mission segment. It creates empty containers for
    input and output conditions that will be populated during the mission analysis.
    
    The function initializes the following in energy_conditions.converters[ram.tag]:
        - inputs : Conditions
            Input conditions container (empty)
        - outputs : Conditions
            Output conditions container (empty)
    
    The ram air converter is a component that captures the energy of the incoming airflow
    and converts it to a form usable by the propulsion system. It typically represents
    the inlet of a gas turbine engine or other air-breathing propulsion system.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Ram.compute_ram_performance
    """
    energy_conditions.converters[ram.tag]                              = Conditions() 
    energy_conditions.converters[ram.tag].inputs                       = Conditions() 
    energy_conditions.converters[ram.tag].outputs                      = Conditions() 
    return 