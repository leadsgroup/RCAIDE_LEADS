# RCAIDE/Library/Methods/Powertrain/Propulsors/Constant_Speed_ICE_Propulsor/append_constant_speed_internal_combustion_engine_conditions.py
# 
# Created:  Jun 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Mission.Common     import   Conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Common.append_propulsor_conditions import append_propulsor_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_constant_speed_internal_combustion_engine_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_constant_speed_internal_combustion_engine_conditions(propulsor, segment):
    """
    Initializes constant speed internal combustion engine operating conditions for a mission segment.
    
    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.Constant_Speed_Internal_Combustion_Engine
        Constant speed internal combustion engine propulsor component with the following attributes:
            - tag : str
                Identifier for the engine
            - items : dict
                Dictionary of subcomponents
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment with the following attributes:
            - state : Data
                Segment state
                    - ones_row : function
                        Function to create array of ones with specified length


    Returns
    -------
    None
        Results are stored in energy_conditions.propulsors[propulsor.tag] and
        segment.state.conditions.aeroacoustics[propulsor.tag]
    
    Notes
    -----
    This function initializes the necessary data structures for storing constant speed
    internal combustion engine operating conditions during a mission segment. It creates 
    zero-filled arrays for various performance parameters and recursively calls the 
    append_operating_conditions method for each subcomponent of the engine.
    
    The function initializes the following parameters in energy_conditions:
        - throttle
        - commanded_thrust_vector_angle
        - thrust
        - power
        - moment
        - fuel_mass_flow_rate
        - inputs and outputs containers
    
    It also creates a noise conditions container for the engine.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Constant_Speed_Internal_Combustion_Engine.compute_constant_speed_internal_combustion_engine_performance
    """
    # unpack 
    
    # add propulsor conditions 
    append_propulsor_conditions(propulsor, segment)
    segment.state.conditions.aeroacoustics[propulsor.tag]                                           = Conditions()

    for tag, item in propulsor.items():
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment)

    return