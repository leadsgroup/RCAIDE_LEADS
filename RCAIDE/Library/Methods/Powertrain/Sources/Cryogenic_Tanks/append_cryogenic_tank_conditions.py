# RCAIDE/Methods/Powertrain/Sources/Cryogenic_Tanks/append_cryogenic_tank_conditions.py
# 
# 
# Created:  Jan 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Framework.Mission.Common     import   Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------  
def append_cryogenic_tank_conditions(cryogenic_tank, segment, bus):
    """
    Appends conditions data structure to cryogenic tank component for use during mission analysis.
    
    Parameters
    ----------
    cryogenic_tank : CryogenicTank
        The cryogenic tank component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the cryogenic tank is operating.
    bus : Bus
        The energy bus connected to the cryogenic tank.
    
    Returns
    -------
    None
    
    Notes
    -----
    This function initializes the conditions for a cryogenic tank component at the start
    of a mission segment. It creates a Conditions object for the cryogenic tank within
    the segment's energy conditions dictionary, indexed by the bus tag and tank tag.
    
    The function initializes arrays for:
        - Mass flow rate [kg/s]
        - Mass [kg]
    
    These arrays are initialized with ones of the same length as the segment's state vector,
    which will be updated during mission analysis based on the cryogenic tank's performance
    and usage.
    
    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Cryogenic_Tanks
    RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.append_fuel_tank_conditions
    """
    ones_row    = segment.state.ones_row                 
    segment.state.conditions.energy[bus.tag][cryogenic_tank.tag]                 = Conditions()  
    segment.state.conditions.energy[bus.tag][cryogenic_tank.tag].mass_flow_rate  = ones_row(1)  
    segment.state.conditions.energy[bus.tag][cryogenic_tank.tag].mass            = ones_row(1)
    
    return 
