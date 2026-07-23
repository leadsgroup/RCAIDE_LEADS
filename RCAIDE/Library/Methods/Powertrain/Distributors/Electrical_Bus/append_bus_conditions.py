#  RCAIDE/Methods/Energy/Distributors/Electrical_Bus/append_bus_conditions.py
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
     
    """
    ones_row                                                                              = segment.state.ones_row
    segment.state.conditions.energy.distributors[bus.tag]                                 = Conditions()  
    segment.state.conditions.energy.distributors[bus.tag].links                           = Conditions() 
    segment.state.conditions.energy.distributors[bus.tag].voltage                         = bus.design_voltage * ones_row(1)
 

    segment.state.conditions.energy.distributors[bus.tag].inputs                        = Conditions()
    segment.state.conditions.energy.distributors[bus.tag].inputs.current                = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].inputs.power                  = Conditions()
    segment.state.conditions.energy.distributors[bus.tag].inputs.power.propulsive       = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].inputs.power.mechanical       = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].inputs.power.electrical       = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].inputs.power.chemical         = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].inputs.power.pneumatic        = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].inputs.power.hydraulic        = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].inputs.power.thermal          = 0. * ones_row(1)  
    segment.state.conditions.energy.distributors[bus.tag].outputs                       = Conditions() 
    segment.state.conditions.energy.distributors[bus.tag].outputs.power                 = Conditions()
    segment.state.conditions.energy.distributors[bus.tag].outputs.current               = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].outputs.power.propulsive      = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].outputs.power.mechanical      = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].outputs.power.electrical      = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].outputs.power.chemical        = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].outputs.power.pneumatic       = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].outputs.power.hydraulic       = 0. * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].outputs.power.thermal         = 0. * ones_row(1)     

    if bus.assigned_distributors != None:
        for distributor_tag in bus.assigned_distributors[0]:    
            link                  = Conditions()
            link.power            = Conditions()
            link.power.electrical = 0 * ones_row(1) 
            segment.state.conditions.energy.distributors[bus.tag].links[distributor_tag] = link     

    return


def append_bus_segment_conditions(bus,segment):

    elif 'initial_battery_state_of_charge' in segment:
        bus_conditions.energy[:,0] = segment.initial_battery_state_of_charge * bus.maximum_energy

    bus_conditions   = segment.state.conditions.energy.distributors[bus.tag]  
    bus_conditions.inputs.power.electrical[:,0]             = 0.0
    bus_conditions.inputs.power.thermal[:,0]                = 0.0
    bus_conditions.inputs.power.hydraulic[:,0]              = 0.0
    bus_conditions.inputs.power.propulsive[:,0]             = 0.0
    bus_conditions.inputs.power.pneumatic[:,0]              = 0.0
    bus_conditions.inputs.power.mechanical[:,0]             = 0.0
    bus_conditions.inputs.power.chemical[:,0]               = 0.0 
    bus_conditions.outputs.power.electrical[:,0]            = 0.0
    bus_conditions.outputs.power.thermal[:,0]               = 0.0
    bus_conditions.outputs.power.hydraulic[:,0]             = 0.0
    bus_conditions.outputs.power.propulsive[:,0]            = 0.0
    bus_conditions.outputs.power.pneumatic[:,0]             = 0.0
    bus_conditions.outputs.power.mechanical[:,0]            = 0.0
    bus_conditions.outputs.power.chemical[:,0]              = 0.0    

    return