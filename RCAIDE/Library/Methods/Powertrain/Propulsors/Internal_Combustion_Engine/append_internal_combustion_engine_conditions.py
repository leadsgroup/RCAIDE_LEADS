# RCAIDE/Library/Methods/Powertrain/Propulsors/Internal_Combustion_Engine/append_internal_combustion_engine_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Mission.Common     import   Conditions 

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_internal_combustion_engine_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_internal_combustion_engine_conditions(propulsor, segment):
    """
    Initializes internal combustion engine operating conditions for a mission segment.
    
    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.Internal_Combustion_Engine
        Internal combustion engine propulsor component with the following attributes:
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
    This function initializes the necessary data structures for storing internal combustion
    engine operating conditions during a mission segment. It creates zero-filled arrays
    for various performance parameters and recursively calls the append_operating_conditions
    method for each subcomponent of the engine.
    
    The function initializes the following parameters in energy_conditions:
        * throttle
        * commanded_thrust_vector_angle
        * thrust
        * power
        * moment
        * fuel_mass_flow_rate
        * inputs and outputs containers
    
    It also creates a noise conditions container for the engine.
    
    **Major Assumptions**
        * All arrays are initialized with zeros
        * Each component has an append_operating_conditions method
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Internal_Combustion_Engine.compute_internal_combustion_engine_performance
    """
    # unpack 
    ones_row                                                                                = segment.state.ones_row 
    
    # add propulsor conditions 
    segment.state.conditions.energy.propulsors[propulsor.tag]                               = Conditions()  
    segment.state.conditions.energy.propulsors[propulsor.tag].throttle                      = 0. * ones_row(1)      
    segment.state.conditions.energy.propulsors[propulsor.tag].commanded_thrust_vector_angle = 0. * ones_row(1)  
    segment.state.conditions.energy.propulsors[propulsor.tag].thrust                        = 0. * ones_row(3) 
    segment.state.conditions.energy.propulsors[propulsor.tag].moment                        = 0. * ones_row(3)
    segment.state.conditions.energy.propulsors[propulsor.tag].power                         = Conditions()  
    segment.state.conditions.energy.propulsors[propulsor.tag].power.propulsive              = 0 * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].power.mechanical              = 0 * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].power.electrical              = 0 * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].power.chemical                = 0 * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].power.pneumatic               = 0 * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].power.hydraulic               = 0 * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].power.thermal                 = 0 * ones_row(1)  
    segment.state.conditions.energy.propulsors[propulsor.tag].fuel_mass_flow_rate           = 0. * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].inputs                        = Conditions()
    segment.state.conditions.energy.propulsors[propulsor.tag].outputs                       = Conditions() 
    segment.state.conditions.noise[propulsor.tag]                                           = Conditions()  
                 
    return 