# RCAIDE/Library/Methods/Powertrain/Systems/compute_flight_controls_power_draw.py
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
# package imports
def compute_flight_controls_power_draw(flight_controls,vehicle,bus,state):
    """
    Computes the power draw of an flight_controls system.
    
    Parameters
    ----------
    flight_controls : flight_controls
        The flight_controls component with the following attributes:
            - power_draw : float
                Power consumption of the flight_controls component [W]
    flight_controls_conditions : Conditions
        Object to store flight_controls power conditions with the following attributes:
            - power : numpy.ndarray
                Array to store the computed power draw values [W]
    conditions : Conditions
        Object containing mission conditions (not directly used in this function)
    
    Returns
    -------
    None
        This function modifies the flight_controls_conditions.power array in-place.
    
    Notes
    -----
    This function assigns the constant power draw value from the flight_controls component
    to the power array in the flight_controls_conditions object. The power draw is assumed
    to be constant throughout the mission segment.
    
    For more complex flight_controls models, this function could be extended to calculate
    power draw based on operating mode, altitude, or other mission parameters.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Systems.append_flight_controls_conditions
    """
    bus_conditions                        = state.conditions.energy.busses[bus.tag]
    flight_controls_conditions            = bus_conditions[flight_controls.tag]    
    flight_controls_conditions.power[:,0] = flight_controls.power_draw 
    bus_conditions.power_draw             += flight_controls_conditions.power*bus.power_split_ratio /bus.efficiency    
    return 