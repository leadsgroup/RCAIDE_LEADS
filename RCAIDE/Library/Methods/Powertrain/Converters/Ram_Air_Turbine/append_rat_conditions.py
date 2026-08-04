# RCAIDE/Library/Methods/Powertrain/Converters/Ram_Air_Turbine/append_rat_conditions.py
#
# Created:  Jun 2024, M. Clarke

from RCAIDE.Framework.Mission.Common     import   Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  append_rat_conditions
# ----------------------------------------------------------------------------------------------------------------------
def append_rat_conditions(rat, segment):
    """
    Initializes ram air turbine (RAT) operating conditions for a mission segment.

    Parameters
    ----------
    rat : RCAIDE.Library.Components.Powertrain.Converters.Ram_Air_Turbine
        Ram air turbine component with the following attributes:
            - tag : str
                Identifier for the ram air turbine
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment with the following attributes:
            - state : Data
                Segment state
                    - ones_row : function
                        Function to create array of ones with specified length

    Returns
    -------
    None

    Notes
    -----
    This function initializes the necessary data structures for storing ram air
    turbine operating conditions during a mission segment. It creates empty
    containers for input and output conditions that will be populated during
    the mission analysis.

    The function initializes the following in segment.state.conditions.energy.converters[rat.tag]:
        - inputs : Conditions
            Input conditions container (empty)
        - outputs : Conditions
            Output conditions container (empty)

    The ram air turbine is a small turbine deployed into the freestream airflow
    to generate emergency electrical/hydraulic power, typically after a loss of
    the aircraft's primary power sources.

    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Ram_Air_Turbine.compute_rat_performance
    """
    segment.state.conditions.energy.converters[rat.tag]                              = Conditions()
    segment.state.conditions.energy.converters[rat.tag].inputs                       = Conditions()
    segment.state.conditions.energy.converters[rat.tag].outputs                      = Conditions()
    return
