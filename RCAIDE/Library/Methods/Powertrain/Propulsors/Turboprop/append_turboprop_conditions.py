# RCAIDE/Library/Methods/Powertrain/Propulsors/Turboprop/append_turboprop_conditions.py
# 
# Created:  Jun 2024, M. Clarke  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turboprop_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turboprop_conditions(propulsor, segment):
    """
    Initializes turboprop operating conditions for a mission segment.
    
    Parameters
    ----------
    propulsor : RCAIDE.Library.Components.Propulsors.Turboprop
        Turboprop propulsor component with the following attributes:
            - tag : str
                Identifier for the turboprop
            - items : dict
                Dictionary of subcomponents
    segment : RCAIDE.Framework.Mission.Segments.Segment
        Mission segment with the following attributes:
            - state : Data
                Segment state
                    - ones_row : function
                        Function to create array of ones with specified length
    energy_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Energy conditions container where turboprop conditions will be stored
    noise_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Noise conditions container where turboprop noise conditions will be stored
    
    Returns
    -------
    None
        Results are stored in energy_conditions.propulsors[propulsor.tag] and
        noise_conditions.propulsors[propulsor.tag]
    
    Notes
    -----
    This function initializes the necessary data structures for storing turboprop
    operating conditions during a mission segment. It creates zero-filled arrays
    for various performance parameters and recursively calls the append_operating_conditions
    method for each subcomponent of the turboprop.
    
    The function initializes the following parameters:
        * throttle
        * commanded_thrust_vector_angle
        * power
        * fuel_mass_flow_rate
        * inputs and outputs containers
    
    It also creates a core_nozzle container in the noise conditions.
    
    **Major Assumptions**
        * All arrays are initialized with zeros
        * Each component has an append_operating_conditions method
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.compute_turboprop_performance
    RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.compute_thrust
    """
    # unpack 
    ones_row                                                                                = segment.state.ones_row
    
    # add propulsor conditions    
    segment.state.conditions.energy.propulsors[propulsor.tag]                               = Conditions()  
    segment.state.conditions.energy.propulsors[propulsor.tag].throttle                      = 0. * ones_row(1)     
    segment.state.conditions.energy.propulsors[propulsor.tag].commanded_thrust_vector_angle = 0. * ones_row(1) 
    segment.state.conditions.energy.propulsors[propulsor.tag].fuel_mass_flow_rate           = 0. * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].power                         = Conditions()  
    segment.state.conditions.energy.propulsors[propulsor.tag].power.propulsive              = 0 * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].power.mechanical              = 0 * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].power.electrical              = 0 * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].power.chemical                = 0 * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].power.pneumatic               = 0 * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].power.hydraulic               = 0 * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].power.thermal                 = 0 * ones_row(1) 
    segment.state.conditions.energy.propulsors[propulsor.tag].inputs                        = Conditions()
    segment.state.conditions.energy.propulsors[propulsor.tag].outputs                       = Conditions() 
    segment.state.conditions.noise.propulsors[propulsor.tag]                                = Conditions()  
    segment.state.conditions.noise.propulsors[propulsor.tag].core_nozzle                    = Conditions()
         
    return 