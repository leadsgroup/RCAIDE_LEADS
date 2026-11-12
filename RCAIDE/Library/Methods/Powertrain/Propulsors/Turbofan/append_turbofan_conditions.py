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
from RCAIDE.Library.Methods.Powertrain.Propulsors.Common.append_propulsor_conditions import append_propulsor_conditions
import numpy as np
from RCAIDE.Library.Methods.Powertrain.reset_power_conditions import reset_power_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_propulsor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turbofan_conditions(propulsor, segment):
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
        * fuel_mass_flow_rate
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
    ones_row                                                                                = segment.state.ones_row 
    
    # add propulsor conditions          
    append_propulsor_conditions(propulsor, segment)
    # segment.rating_code, if set, seeds the surrogate rating-code table used for this
    # segment (e.g. 'FID' to route a descent/idle segment to real flight-idle deck data
    # instead of the RC=0 part-power sweep) -- unset segments keep the previous default (None)
    segment.state.conditions.energy.propulsors[propulsor.tag].rating_code                   = getattr(segment, 'rating_code', None)
    segment.state.conditions.energy.propulsors[propulsor.tag].thrust                        = 0. * ones_row(3) 
    segment.state.conditions.energy.propulsors[propulsor.tag].below_matching_floor          = 0. * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].moment                        = 0. * ones_row(3) 
    segment.state.conditions.energy.propulsors[propulsor.tag].inputs.current                = 0. * ones_row(1)
    segment.state.conditions.energy.propulsors[propulsor.tag].outputs.current               = 0. * ones_row(1)
    segment.state.conditions.aeroacoustics.propulsors[propulsor.tag]                                = Conditions()  
    segment.state.conditions.aeroacoustics.propulsors[propulsor.tag].core_nozzle                    = Conditions() 
    segment.state.conditions.aeroacoustics.propulsors[propulsor.tag].fan_nozzle                     = Conditions() 
    segment.state.conditions.aeroacoustics.propulsors[propulsor.tag].fan                            = Conditions()

    # performance and noise-source outputs of every turbofan method, NaN until computed
    rows                 = ones_row(1).shape[0]
    turbofan_conditions  = segment.state.conditions.energy.propulsors[propulsor.tag]
    for key in ['thrust_specific_fuel_consumption', 'bypass_ratio', 'flow_through_core', 'flow_through_fan', 'fuel_to_air_ratio',
                'core_mass_flow_rate', 'total_temperature_reference', 'total_pressure_reference', 'overall_efficiency']:
        turbofan_conditions[key] = np.full((rows, 1), np.nan)
    noise_conditions     = segment.state.conditions.aeroacoustics.propulsors[propulsor.tag]
    for nozzle in (noise_conditions.core_nozzle, noise_conditions.fan_nozzle):
        for key in ['exit_static_temperature', 'exit_static_pressure', 'exit_stagnation_temperature', 'exit_stagnation_pressure',
                    'exit_velocity']:
            nozzle[key] = np.full((rows, 1), np.nan)
    noise_conditions.fan.angular_velocity = np.full((rows, 1), np.nan)

    for tag, item in  propulsor.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment) 
            for sub_tag, sub_item in  item.items(): 
                if issubclass(type(sub_item), RCAIDE.Library.Components.Component): 
                    sub_item.append_operating_conditions(segment)    
    return


def append_turbofan_segment_conditions(self,segment): 
    energy_conditions  = segment.state.conditions.energy    
    reset_power_conditions(energy_conditions.propulsors[self.tag])