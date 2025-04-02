# RCAIDE/Library/Methods/Powertrain/Systems/compute_payload_power_draw.py
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
# package imports  
def compute_payload_power_draw(payload, payload_conditions, conditions): 
    """
    Computes the power draw of a payload system.
    
    Parameters
    ----------
    payload : Payload
        The payload component with the following attributes:
            - power_draw : float
                Constant power consumption of the payload system [W]
    payload_conditions : Conditions
        Object to store payload power conditions with the following attributes:
            - power : numpy.ndarray
                Array to store the computed power draw values [W]
    conditions : Conditions
        Object containing mission conditions (not directly used in this function)
    
    Returns
    -------
    None
    
    Notes
    -----
    This function assigns the constant power draw value from the payload component
    to the power array in the payload_conditions object. The power draw is assumed
    to be constant throughout the mission segment.
    
    For more complex payload models, this function could potentially be extended to calculate
    power draw based on operating mode, mission phase, or other parameters.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Systems.append_payload_conditions
    RCAIDE.Library.Methods.Powertrain.Systems.compute_avionics_power_draw
    """
    payload_conditions.power[:,0] = payload.power_draw  
    return 