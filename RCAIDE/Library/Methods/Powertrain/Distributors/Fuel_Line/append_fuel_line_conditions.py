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
    segment.state.conditions.energy.fuel_lines[fuel_line.tag].fuel_mass_flow_rate                 = 0 * ones_row(1)  
    segment.state.conditions.energy.fuel_lines[fuel_line.tag].fuel_tanks                          = Conditions() 

    if fuel_line.additional_line_flow_rate != 0:
        # If there is an additional flow rate on the line then we will split it between the fuel tanks on the fuel line based on the volume of the tank
        total_mass = 0 
        for fuel_tank in fuel_line.fuel_tanks:
            total_mass += fuel_tank.fuel.mass_properties.mass 
        for fuel_tank in fuel_line.fuel_tanks:
            fuel_tank.secondary_mass_flow_rate= (fuel_tank.fuel.mass_properties.mass / total_mass) * fuel_line.additional_line_flow_rate

    # Sum all user-defined split ratios; treat missing values as 0.
    user_defined_ratio = sum(t.fuel_flow_split_ratio or 0 for t in fuel_line.fuel_tanks)

    # Tanks without a user-defined ratio will be automatically assigned one.
    auto_assigned_tanks = [t for t in fuel_line.fuel_tanks if t.fuel_flow_split_ratio is None]

    # Total fuel mass across auto-assigned tanks (used for mass-proportional splitting).
    total_auto_mass = sum(t.fuel.mass_properties.mass for t in auto_assigned_tanks)

    # Portion of ratio budget still available after user-defined assignments.
    remaining_ratio = max(0.0, 1.0 - user_defined_ratio)

    # Distribute remaining ratio across auto-assigned tanks:
    # - proportional to fuel mass when total mass is nonzero
    # - evenly if all auto-assigned tank masses sum to zero
    for t in auto_assigned_tanks:
        t.fuel_flow_split_ratio = (
            (t.fuel.mass_properties.mass / total_auto_mass) * remaining_ratio
            if total_auto_mass
            else remaining_ratio / len(auto_assigned_tanks)
        )

    # Validation: if every tank was user-defined, their ratios must total ~1.0.
    if not auto_assigned_tanks:
        if round(user_defined_ratio, 4) != 1.0:
            raise ValueError(
                f"User-defined flow_split_ratio values sum to {user_defined_ratio:.3f}, must equal 1.0"
            )

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
    segment.state.conditions.energy.fuel_lines[fuel_line.tag].fuel_mass_flow_rate[:,0]    = 0
    return