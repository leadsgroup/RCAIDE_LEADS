# RCAIDE/Library/Methods/Powertrain/Propulsors/Electric_Rotor_Propulsor/append_electric_rotor_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Mission.Common                      import Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append electric rotor network conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_electric_rotor_conditions(propulsor, segment, energy_conditions, noise_conditions):
    """
    Appends data structures arrays for storing electric rotor conditions.
    
    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.Electric_Rotor
        Electric rotor propulsor component with the following attributes:
            - tag : str
                Identifier for the propulsor
            - items : dict
                Dictionary of subcomponents
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment with the following attributes:
            - state : Data
                Segment state
                - ones_row : function
                    Function to create array of ones with specified length
    energy_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Energy conditions container where electric rotor conditions will be stored
    noise_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Noise conditions container where electric rotor noise conditions will be stored
    
    Returns
    -------
    None
        Results are stored in energy_conditions.propulsors[propulsor.tag] and
        noise_conditions.propulsors[propulsor.tag]
    
    Notes
    -----
    This function initializes the necessary data structures for storing electric rotor
    operating conditions during a mission segment. It creates zero-filled arrays
    for various performance parameters and recursively calls the append_operating_conditions
    method for each subcomponent of the electric rotor propulsor.
    
    The function initializes the following parameters in energy_conditions:
        - throttle
        - commanded_thrust_vector_angle
        - thrust
        - power
        - moment
    
    It also creates a noise conditions container for the electric rotor.
    
    **Major Assumptions**
        * All arrays are initialized with zeros
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Rotor.compute_electric_rotor_performance
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
    noise_conditions.propulsors[propulsor.tag]                                = Conditions() 
       
    # parse propulsor for comoonent and append 
    for tag, item in  propulsor.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,energy_conditions,noise_conditions)
    return
