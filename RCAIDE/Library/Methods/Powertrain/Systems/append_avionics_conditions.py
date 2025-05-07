# RCAIDE/Library/Methods/Powertrain/Systems/append_avionics_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_avionics_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_avionics_conditions(avionics, segment, bus):  
    """
    Initializes and appends empty avionics conditions data structures to the segment state conditions.
    
    Parameters
    ----------
    avionics : Avionics
        The avionics component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the avionics is operating.
    bus : ElectricalBus
        The electrical bus that powers the avionics system.
    
    Returns
    -------
    None
    
    Notes
    -----
    This function creates an empty Conditions object for the avionics system within
    the segment's energy conditions dictionary, indexed by the bus tag and avionics tag.
    
    The avionics power consumption is initialized as a zero array with the same
    length as the segment's state vector. This will be updated during mission analysis
    based on the avionics power requirements.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Systems.append_payload_conditions
    """
    ones_row    = segment.state.ones_row
    segment.state.conditions.energy.busses[bus.tag][avionics.tag]            = Conditions()
    segment.state.conditions.energy.busses[bus.tag][avionics.tag].power      = 0 * ones_row(1)
    
    return 
