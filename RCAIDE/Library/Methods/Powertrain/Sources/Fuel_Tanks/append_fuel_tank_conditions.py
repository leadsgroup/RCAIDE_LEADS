# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/append_fuel_tank_conditions.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Framework.Mission.Common     import   Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------  
def append_fuel_tank_conditions(fuel_tank, segment, fuel_line):
    """
    Appends initial conditions for fuel tank component during later mission analysis.
    
    Parameters
    ----------
    fuel_tank : FuelTank
        The fuel tank component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the fuel tank is operating.
    fuel_line : FuelLine
        The fuel line connected to the fuel tank.
    
    Returns
    -------
    None
    
    Notes
    -----
    This function initializes the conditions for a fuel tank component at the start
    of a mission segment. It creates a Conditions object for the fuel tank within
    the segment's energy conditions dictionary, indexed by the fuel line tag and tank tag.
    
    The function initializes arrays for:
        - Mass flow rate [kg/s]
        - Mass [kg]
    
    These arrays are initialized with ones of the same length as the segment's state vector,
    which will be updated during mission analysis based on the fuel tank's performance
    and fuel consumption.
    
    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks
    RCAIDE.Library.Methods.Powertrain.Sources.Cryogenic_Tanks.append_cryogenic_tank_conditions
    """
    ones_row    = segment.state.ones_row                 
    segment.state.conditions.energy[fuel_line.tag][fuel_tank.tag]                 = Conditions()  
    segment.state.conditions.energy[fuel_line.tag][fuel_tank.tag].mass_flow_rate  = ones_row(1)  
    segment.state.conditions.energy[fuel_line.tag][fuel_tank.tag].mass            = ones_row(1)
    
    return 
