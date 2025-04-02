# RCAIDE/Library/Methods/Powertrain/Systems/compute_avionics_power_draw.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
# package imports
def compute_avionics_power_draw(avionics, avionics_conditions, conditions): 
    """
    Computes the power draw of an avionics system.
    
    Parameters
    ----------
    avionics : Avionics
        The avionics component with the following attributes:
            - power_draw : float
                Power consumption of the avionics component [W]
    avionics_conditions : Conditions
        Object to store avionics power conditions with the following attributes:
            - power : numpy.ndarray
                Array to store the computed power draw values [W]
    conditions : Conditions
        Object containing mission conditions (not directly used in this function)
    
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
    avionics_conditions.power[:,0] = avionics.power_draw  
    return 