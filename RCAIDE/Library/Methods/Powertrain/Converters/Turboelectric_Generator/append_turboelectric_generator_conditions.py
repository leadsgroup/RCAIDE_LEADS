# RCAIDE/Library/Methods/Powertrain/Converters/Turboelectric_Generator/append_turboelectric_generator_conditions.py 
# 
# Created:  Feb 2025, M. Clarke  
from RCAIDE.Framework.Mission.Common     import   Conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Common.append_converter_power_conditions import append_converter_power_conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turboelectric_generator_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turboelectric_generator_conditions(turboelectric_generator,segment):  
    """
    Initializes and appends operating conditions data structures for a turboelectric generator to the energy conditions structure.
    
    Parameters
    ----------
    turboelectric_generator : RCAIDE.Components.Energy.Converters.Turboelectric_Generator
        The turboelectric generator component for which conditions are being appended
    segment : RCAIDE.Analyses.Mission.Segments
        The mission segment being evaluated
        
    Returns
    -------
    None
        This function modifies the segment.state.conditions.energy object in-place
        
    Notes
    -----
    This function initializes the condition structure for a turboelectric generator
    and its subcomponents (turboshaft and generator) with zero values, then calls
    the respective append_operating_conditions methods for each subcomponent.
    """

    ones_row  = segment.state.ones_row
    teg_conditions = append_converter_power_conditions(turboelectric_generator, segment)
    teg_conditions.fuel_mass_flow_rate = 0. * ones_row(1)

    turboshaft = turboelectric_generator.turboshaft
    generator  = turboelectric_generator.generator
    turboshaft.append_operating_conditions(segment)
    generator.append_operating_conditions(segment)
    return 