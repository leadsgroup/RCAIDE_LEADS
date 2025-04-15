# RCAIDE/Library/Methods/Powertrain/Systems/compute_payload_power_draw.py
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
# package imports  
def compute_payload_power_draw(payload, bus, conditions):
    """
    Computes the power draw of a payload component on an electrical bus.
    
    Parameters
    ----------
    payload : RCAIDE.Library.Components.Systems.Payload
        Payload component with the following attributes:
            - tag : str
                Identifier for the payload
            - power_draw : float
                Power required by the payload [W]
    bus : RCAIDE.Library.Components.Systems.Electrical_Bus
        Electrical bus component with the following attributes:
            - tag : str
                Identifier for the electrical bus
            - power_split_ratio : float
                Ratio of power allocation to this bus
            - efficiency : float
                Efficiency of the bus [0-1]
    conditions : RCAIDE.Framework.Mission.Common.Conditions
        Flight conditions with:
            - energy : dict
                Energy conditions indexed by component tag
                    - [bus.tag] : Data
                        Bus-specific conditions
                            - [payload.tag] : Data
                                Payload-specific conditions
                                - power : numpy.ndarray
                                    Power draw of the payload [W]
                            - power_draw : numpy.ndarray
                                Total power draw on the bus [W]
    
    Returns
    -------
    None
    
    Notes
    -----
    This function calculates the power draw of a payload component on an electrical bus
    and updates the total power draw on the bus. The power draw is adjusted by the
    bus power split ratio and efficiency.
    
    **Major Assumptions**
        * Constant payload power draw
        * Constant bus efficiency
        * Power draw is independent of flight conditions
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Systems.compute_avionics_power_draw
    """
    bus_conditions                = conditions.energy.busses[bus.tag]
    payload_conditions            = bus_conditions[payload.tag]    
    payload_conditions.power[:,0] = payload.power_draw  
    bus_conditions.power_draw     += payload_conditions.power*bus.power_split_ratio /bus.efficiency
    return 