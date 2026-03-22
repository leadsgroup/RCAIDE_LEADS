#  RCAIDE/Methods/Energy/Distributors/Electrical_Bus/append_bus_conditions.py
# 
# Created: Sep 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Mission.Common     import   Conditions
# ----------------------------------------------------------------------------------------------------------------------
#  METHODS
# ---------------------------------------------------------------------------------------------------------------------- 
def append_bus_conditions(bus,segment): 
    """
    Appends conditions for the electrical bus to the segment's energy conditions dictionary.

    Parameters
    ----------
    bus : RCAIDE.Library.Components.Distributors.Electrical_Bus
    
    Returns
    -------
    None
        This function modifies the segment.state.conditions.energy dictionary in-place.
    
    Notes
    -----
    This function creates a Conditions object for the electrical bus within the segment's
    energy conditions dictionary, indexed by the bus tag. It initializes various bus
    properties as zero arrays with the same length as the segment's state vector.
    
    The initialized properties include:
        - Battery module conditions
        - Fuel cell stack conditions
        - Power draw
        - State of charge and depth of discharge
        - Current draw and charging current
        - Voltage (open circuit and under load)
        - Heat energy generated
        - Efficiency
        - Temperature
        - Energy
        - Regenerative power
    
    For segments with an initial battery state of charge specified, the function also
    sets the initial energy and state of charge values accordingly.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Distributors.Electrical_Bus.compute_bus_conditions
    """
    ones_row                                                                = segment.state.ones_row
    segment.state.conditions.energy.distributors[bus.tag]                   = Conditions() 
    segment.state.conditions.energy.distributors[bus.tag].power             = Conditions()  
    segment.state.conditions.energy.distributors[bus.tag].links             = Conditions() 

    if bus.assigned_distributors != None:
        for distributor_tag in bus.assigned_distributors[0]:    
            link                  = Conditions()
            link.power            = Conditions()
            link.power.electrical = 0 * ones_row(1) 
            segment.state.conditions.energy.distributors[bus.tag].links[distributor_tag] = link     

    return


def append_bus_segment_conditions(bus,segment):

    return