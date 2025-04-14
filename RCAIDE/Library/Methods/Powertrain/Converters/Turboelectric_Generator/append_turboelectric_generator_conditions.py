# RCAIDE/Library/Methods/Powertrain/Converters/Turboelectric_Generator/append_turboelectric_generator_conditions.py 
# 
# Created:  Feb 2025, M. Clarke  
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_turboelectric_generator_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_turboelectric_generator_conditions(turboelectric_generator,segment,energy_conditions):  
    """
    Initializes and appends operating conditions data structures for a turboelectric generator to the energy conditions structure.
    
    Parameters
    ----------
    turboelectric_generator : RCAIDE.Components.Energy.Converters.Turboelectric_Generator
        The turboelectric generator component for which conditions are being appended
    segment : RCAIDE.Analyses.Mission.Segments
        The mission segment being evaluated
    energy_conditions : RCAIDE.Framework.Mission.Common.Conditions
        Container for energy-related conditions during the mission segment
        
    Returns
    -------
    None
        This function modifies the energy_conditions object in-place
        
    Notes
    -----
    This function initializes the condition structure for a turboelectric generator
    and its subcomponents (turboshaft and generator) with zero values, then calls
    the respective append_operating_conditions methods for each subcomponent.
    """

    ones_row    = segment.state.ones_row   
 
    energy_conditions.converters[turboelectric_generator.tag] = Conditions() 
    energy_conditions.converters[turboelectric_generator.tag].throttle                                   = 0. * ones_row(1)     
    energy_conditions.converters[turboelectric_generator.tag].commanded_thrust_vector_angle              = 0. * ones_row(1)   
    energy_conditions.converters[turboelectric_generator.tag].power                                      = 0. * ones_row(1)
    energy_conditions.converters[turboelectric_generator.tag].fuel_flow_rate                             = 0. * ones_row(1)
    energy_conditions.converters[turboelectric_generator.tag].inputs                                     = Conditions()
    energy_conditions.converters[turboelectric_generator.tag].outputs                                    = Conditions() 
  
    turboshaft = turboelectric_generator.turboshaft
    generator  = turboelectric_generator.generator
    turboshaft.append_operating_conditions(segment,energy_conditions)
    generator.append_operating_conditions(segment,energy_conditions)
    return 