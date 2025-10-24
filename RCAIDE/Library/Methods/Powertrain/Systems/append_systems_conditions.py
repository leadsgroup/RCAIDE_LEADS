# RCAIDE/Library/Methods/Powertrain/Systems/append_systems_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_system_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_systems_conditions(system, segment):  
    """
    Initializes and appends empty system conditions data structures to the segment state conditions.
    
    Parameters
    ----------
    system : System
        The system component for which conditions are being initialized.
    segment : Segment
        The mission segment in which the system is operating.
    bus : ElectricalBus
        The electrical bus that powers the generic system.
    
    Returns
    -------
    None
    
    Notes
    -----
    This function creates an empty Conditions object for the generic system within
    the segment's energy conditions dictionary, indexed by the bus tag and system tag.
    
    The system power consumption is initialized as a zero array with the same
    length as the segment's state vector. This will be updated during mission analysis
    based on the system power requirements.
    
    See Also
    -------- 
    """
    ones_row                                                                     = segment.state.ones_row
    segment.state.conditions.energy.systems[system.tag]                          = Conditions()
    segment.state.conditions.energy.systems[system.tag].inputs                   = Conditions()
    segment.state.conditions.energy.systems[system.tag].outputs                  = Conditions()
    segment.state.conditions.energy.systems[system.tag].inputs.power             = Conditions()
    segment.state.conditions.energy.systems[system.tag].outputs.power            = Conditions()
    segment.state.conditions.energy.systems[system.tag].inputs.power.propulsive  = 0 * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].inputs.power.mechanical  = 0 * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].inputs.power.electrical  = 0 * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].inputs.power.chemical    = 0 * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].inputs.power.pneumatic   = 0 * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].inputs.power.hydraulic   = 0 * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].inputs.power.thermal     = 0 * ones_row(1) 
    segment.state.conditions.energy.systems[system.tag].outputs.power.propulsive = 0 * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].outputs.power.mechanical = 0 * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].outputs.power.electrical = 0 * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].outputs.power.chemical   = 0 * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].outputs.power.pneumatic  = 0 * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].outputs.power.hydraulic  = 0 * ones_row(1)
    segment.state.conditions.energy.systems[system.tag].outputs.power.thermal    = 0 * ones_row(1) 
    
    return 
