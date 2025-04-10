# RCAIDE/Library/Methods/Powertrain/Propulsors/Constant_Speed_ICE_Propulsor/append_constant_speed_internal_combustion_engine_conditions.py
# 
# Created:  Jun 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_constant_speed_internal_combustion_engine_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_constant_speed_internal_combustion_engine_conditions(propulsor, segment, energy_conditions, noise_conditions):
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
    energy_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Energy conditions container where engine conditions will be stored
    noise_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Noise conditions container where engine noise conditions will be stored
    
    Returns
    -------
    None
        Results are stored in energy_conditions.propulsors[propulsor.tag] and
        segment.state.conditions.noise[propulsor.tag]
    
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
        - fuel_flow_rate
        - inputs and outputs containers
    
    It also creates a noise conditions container for the engine.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Constant_Speed_Internal_Combustion_Engine.compute_constant_speed_internal_combustion_engine_performance
    """
    # unpack 
    ones_row          = segment.state.ones_row 
    
    # add propulsor conditions 
    energy_conditions.propulsors[propulsor.tag]                               = Conditions()  
    energy_conditions.propulsors[propulsor.tag].throttle                      = 0. * ones_row(1)      
    energy_conditions.propulsors[propulsor.tag].commanded_thrust_vector_angle = 0. * ones_row(1)  
    energy_conditions.propulsors[propulsor.tag].thrust                        = 0. * ones_row(3) 
    energy_conditions.propulsors[propulsor.tag].power                         = 0. * ones_row(1) 
    energy_conditions.propulsors[propulsor.tag].moment                        = 0. * ones_row(3) 
    energy_conditions.propulsors[propulsor.tag].fuel_flow_rate                = 0. * ones_row(1)      
    energy_conditions.propulsors[propulsor.tag].inputs                        = Conditions()
    energy_conditions.propulsors[propulsor.tag].outputs                       = Conditions() 
    segment.state.conditions.noise[propulsor.tag]                             = Conditions()
 
    # parse propulsor for comoonent and append 
    for tag, item in  propulsor.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,energy_conditions,noise_conditions=noise_conditions) 
            for sub_tag, sub_item in  item.items(): 
                if issubclass(type(sub_item), RCAIDE.Library.Components.Component): 
                    sub_item.append_operating_conditions(segment,energy_conditions)    
    return 