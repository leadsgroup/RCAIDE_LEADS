# RCAIDE/Library/Methods/Powertrain/Systems/compute_system_power_draw.py
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
# package imports
def compute_system_power_draw(system,bus,conditions):
    """
    Computes the power draw of a generic system.
    
    Parameters
    ----------
    system : System
        The system component with the following attributes:
            - power_draw : float
                Power consumption of the system component [W]
    system_conditions : Conditions
        Object to store system power conditions with the following attributes:
            - power : numpy.ndarray
                Array to store the computed power draw values [W]
    conditions : Conditions
        Object containing mission conditions (not directly used in this function)
    
    Returns
    -------
    None
        This function modifies the system_conditions.power array in-place.
    
    Notes
    -----
    This function assigns the constant power draw value from the system component
    to the power array in the system_conditions object. The power draw is assumed
    to be constant throughout the mission segment.
    
    For more complex system models, this function could be extended to calculate
    power draw based on operating mode, altitude, or other mission parameters.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Systems.append_system_conditions
    """
    bus_conditions                 = conditions.energy.busses[bus.tag]
    system_conditions              = bus_conditions[system.tag]    
    system_conditions.power[:,0]   = system.power_draw 
    bus_conditions.power_draw      += system_conditions.power*bus.power_split_ratio /bus.efficiency    
    return 