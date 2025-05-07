# RCAIDE/Library/Methods/Powertrain/Systems/append_payload_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_payload_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_payload_conditions(payload, segment, bus):  
    """
    Initializes and appends empty payload conditions data structures to the segment state conditions.
    
    Parameters
    ----------
    payload : Payload
        The payload component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the payload is operating.
    bus : ElectricalBus
        The electrical bus that powers the payload system.
    
    Returns
    -------
    None
        This function modifies the segment.state.conditions.energy dictionary in-place.
    
    Notes
    -----
    This function creates an empty Conditions object for the payload system within
    the segment's energy conditions dictionary, indexed by the bus tag and payload tag.
    
    The payload power consumption is initialized as a zero array with the same
    length as the segment's state vector. This will be updated during mission analysis
    based on the payload power requirements.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Systems.append_avionics_conditions
    """
    ones_row    = segment.state.ones_row 
    segment.state.conditions.energy.busses[bus.tag][payload.tag]       = Conditions()
    segment.state.conditions.energy.busses[bus.tag][payload.tag].power = 0 * ones_row(1)
    return 
