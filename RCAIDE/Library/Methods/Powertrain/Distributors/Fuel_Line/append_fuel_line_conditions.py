#  RCAIDE/Methods/Energy/Distributors/Fuel_Line/append_fuel_line_conditions.py
# 
# Created: Sep 2024, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports   
from RCAIDE.Framework.Mission.Common     import   Conditions
# ----------------------------------------------------------------------------------------------------------------------
#  METHODS
# ---------------------------------------------------------------------------------------------------------------------- 
def append_fuel_line_conditions(fuel_line,segment): 
    """
    Appends conditions for the fuel line to the segment's energy conditions dictionary.

    Parameters
    ----------
    fuel_line : RCAIDE.Library.Components.Distributors.Fuel_Line
    
    Returns
    -------
    None
        This function modifies the segment.state.conditions.energy dictionary in-place.
    
    Notes
    -----
    This function creates a Conditions object for the fuel line within the segment's
    energy conditions dictionary, indexed by the fuel line tag. It initializes various fuel line
    properties as zero arrays with the same length as the segment's state vector.
    
    The initialized properties include: 
        - Heat energy generated
        - Efficiency
        - Temperature
        - Energy
        - Flow Rate 
        - Regenerative power
    
    For segments with an initial battery state of charge specified, the function also
    sets the initial energy and state of charge values accordingly.
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Distributors.Electrical_fuel_line.compute_fuel_line_conditions
    """
    ones_row                                                                     = segment.state.ones_row

    # ------------------------------------------------------------------------------------------------------            
    # Create fuel_line results data structure  
    # ------------------------------------------------------------------------------------------------------ 
    segment.state.conditions.energy.fuel_lines[fuel_line.tag]                                     = Conditions() 
    segment.state.conditions.energy.fuel_lines[fuel_line.tag].power_draw                          = 0 * ones_row(1)
    segment.state.conditions.energy.fuel_lines[fuel_line.tag].hybrid_power_split_ratio            = segment.hybrid_power_split_ratio * ones_row(1)  
    segment.state.conditions.energy.fuel_lines[fuel_line.tag].heat_energy_generated               = 0 * ones_row(1) 
    segment.state.conditions.energy.fuel_lines[fuel_line.tag].efficiency                          = 0 * ones_row(1)
    segment.state.conditions.energy.fuel_lines[fuel_line.tag].temperature                         = 0 * ones_row(1)
    segment.state.conditions.energy.fuel_lines[fuel_line.tag].energy                              = 0 * ones_row(1)  
    segment.state.conditions.energy.fuel_lines[fuel_line.tag].fuel_flow_rate                      = 0 * ones_row(1)  
    segment.state.conditions.energy.fuel_lines[fuel_line.tag].fuel_tanks                          = Conditions() 

    return


def append_fuel_line_segment_conditions(fuel_line,segment):
    """
    Sets the initial fuel line properties at the start of each segment based on the last point from the previous segment.
    
    Parameters
    ----------
    fuel_line : Fuel Line 
        The fuel line component for which conditions are being initialized.
    conditions : dict
        Dictionary containing conditions from the previous segment.
    segment : Segment
        The current mission segment in which the bus is operating.
    
    Returns
    -------
    None 
    
    This ensures continuity of energy states between mission segments. 
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Distributors.Fuel_Line.append_fuel_line_conditions 
    """     
    segment.state.conditions.energy.fuel_lines[fuel_line.tag].fuel_flow_rate[:,0]    = 0
    return