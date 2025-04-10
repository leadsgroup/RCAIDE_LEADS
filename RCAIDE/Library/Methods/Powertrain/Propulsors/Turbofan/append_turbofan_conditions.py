# RCAIDE/Library/Methods/Powertrain/Propulsors/Turbofan/append_turbofan_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_propulsor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turbofan_conditions(propulsor, segment, energy_conditions, noise_conditions):
    """
    Initializes turbofan operating conditions for a mission segment.
    
    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.Turbofan
        Turbofan propulsor component with the following attributes:
            - tag : str
                Identifier for the turbofan
            - items : dict
                Dictionary of subcomponents
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment with the following attributes:
            - state : Data
                Segment state
                    - ones_row : function
                        Function to create array of ones with specified length
    energy_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Energy conditions container where turbofan conditions will be stored
    noise_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Noise conditions container where turbofan noise conditions will be stored
    
    Returns
    -------
    None
        Results are stored in energy_conditions.propulsors[propulsor.tag] and
        noise_conditions.propulsors[propulsor.tag]
    
    Notes
    -----
    This function initializes the necessary data structures for storing turbofan
    operating conditions during a mission segment. It creates zero-filled arrays
    for various performance parameters and recursively calls the append_operating_conditions
    method for each subcomponent of the turbofan.
    
    The function initializes the following parameters in energy_conditions:
        * throttle
        * commanded_thrust_vector_angle
        * thrust
        * power
        * moment
        * fuel_flow_rate
        * inputs and outputs containers
    
    It also creates the following containers in noise_conditions:
        * core_nozzle
        * fan_nozzle
        * fan
    
    **Major Assumptions**
        * All arrays are initialized with zeros
        * Each component has an append_operating_conditions method
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.compute_turbofan_performance
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.compute_thurst
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
    noise_conditions.propulsors[propulsor.tag]                                = Conditions()  
    noise_conditions.propulsors[propulsor.tag].core_nozzle                    = Conditions() 
    noise_conditions.propulsors[propulsor.tag].fan_nozzle                     = Conditions() 
    noise_conditions.propulsors[propulsor.tag].fan                            = Conditions()
 
    for tag, item in  propulsor.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,energy_conditions) 
            for sub_tag, sub_item in  item.items(): 
                if issubclass(type(sub_item), RCAIDE.Library.Components.Component): 
                    sub_item.append_operating_conditions(segment,energy_conditions)    
    return 