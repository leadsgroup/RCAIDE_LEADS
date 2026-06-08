# RCAIDE/Library/Methods/Powertrain/Systems/compute_avionics_power_draw.py
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    

def compute_avionics_power_draw(avionics,vehicle,bus,state):
    """
    Computes the electrical power draw of the aircraft's avionics system.
    
    Parameters
    ----------
    avionics : Avionics
        The avionics component with the following attributes:
            - power_draw : float
                Constant power consumption of the avionics suite [W]
    vehicle : Vehicle()
        The vehicle object (unused in this specific method, but required by 
        the RCAIDE network evaluation signature)
    bus : Electrical_Bus
        The electrical bus that powers the avionics system
    state : State
        Object containing the current state of the aircraft
    
    Returns
    -------
    None
        This function modifies the avionics_conditions.power array in-place.
    
    Notes
    -----
    This function assigns the constant power draw value from the avionics component
    to the power array in the avionics_conditions object. The power draw is assumed
    to be constant throughout the mission segment.
    
    For more complex avionics models, this function could be extended to calculate
    power draw based on operating mode, altitude, or other mission parameters.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Systems.append_avionics_conditions
    """
    bus_conditions                 = state.conditions.energy.busses[bus.tag]
    avionics_conditions            = bus_conditions[avionics.tag]    
    avionics_conditions.power[:,0] = avionics.power_draw 
    bus_conditions.power_draw      += avionics_conditions.power*bus.power_split_ratio /bus.efficiency    
    return 