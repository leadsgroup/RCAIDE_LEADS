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
    ones_row                                                                     = segment.state.ones_row
    segment.state.conditions.energy.distributors[bus.tag]                                     = Conditions()
    segment.state.conditions.energy.distributors[bus.tag].battery_modules                     = Conditions()
    segment.state.conditions.energy.distributors[bus.tag].fuel_cell_stacks                    = Conditions()
    segment.state.conditions.energy.distributors[bus.tag].fuel_tanks                          = Conditions()  
    segment.state.conditions.energy.distributors[bus.tag].state_of_charge                     = 0 * ones_row(1) 
    segment.state.conditions.energy.distributors[bus.tag].depth_of_discharge                  = 0 * ones_row(1) 
    segment.state.conditions.energy.distributors[bus.tag].current_draw                        = 0 * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].charging_current                    = 0 * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].voltage_open_circuit                = 0 * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].voltage_under_load                  = 0 * ones_row(1) 
    segment.state.conditions.energy.distributors[bus.tag].heat_energy_generated               = 0 * ones_row(1) 
    segment.state.conditions.energy.distributors[bus.tag].efficiency                          = 0 * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].temperature                         = 0 * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].energy                              = 0 * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].regenerative_power                  = 0 * ones_row(1) 
    segment.state.conditions.energy.distributors[bus.tag].fuel_mass_flow_rate                 = 0 * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].power                               = Conditions() 
    #segment.state.conditions.energy.distributors[bus.tag].power.propulsive                    = 0 * ones_row(1)
    #segment.state.conditions.energy.distributors[bus.tag].power.mechanical                    = 0 * ones_row(1) 
    segment.state.conditions.energy.distributors[bus.tag].power.electrical                    = 0 * ones_row(1)
    #segment.state.conditions.energy.distributors[bus.tag].power.chemical                      = 0 * ones_row(1)
    #segment.state.conditions.energy.distributors[bus.tag].power.pneumatic                     = 0 * ones_row(1)
    #segment.state.conditions.energy.distributors[bus.tag].power.hydraulic                     = 0 * ones_row(1)
    #segment.state.conditions.energy.distributors[bus.tag].power.thermal                       = 0 * ones_row(1)
    segment.state.conditions.energy.distributors[bus.tag].links                               = Conditions() 

    if bus.assigned_distributors != None:
        for distributor_tag in bus.assigned_distributors[0]:    
            link                  = Conditions()
            link.power            = Conditions()
            link.power.electrical = 0 * ones_row(1) 
            segment.state.conditions.energy.distributors[bus.tag].links[distributor_tag] = link     

    return


def append_bus_segment_conditions(bus,segment):
    """
    Sets the initial bus properties at the start of each segment based on the last point from the previous segment.
    
    Parameters
    ----------
    bus : ElectricalBus
        The electrical bus component for which conditions are being initialized.
    conditions : dict
        Dictionary containing conditions from the previous segment.
    segment : Segment
        The current mission segment in which the bus is operating.
    
    Returns
    -------
    None
    
    Notes
    -----
    This function initializes the power draw for the electrical bus at the start of a new segment
    by transferring thermal power draw information from the previous segment. It handles power
    requirements from battery thermal management systems (BTMS) and heat exchangers.
    
    For segments with initial conditions from a previous segment, the function also:
        1. Sets the battery discharge flag based on the segment type (false for recharge segments)
        2. Transfers the final energy state from the previous segment to the initial state of the current segment
    
    This ensures continuity of energy states between mission segments.
    
    **Major Assumptions**
        * Battery recharge segments are identified by their class type
        * Power draw is initialized from thermal management components
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Distributors.Electrical_Bus.append_bus_conditions
    RCAIDE.Library.Methods.Powertrain.Distributors.Electrical_Bus.compute_bus_conditions
    """    
    bus_conditions   = segment.state.conditions.energy.distributors[bus.tag]  
    #bus_conditions.power.propulsive[:,0]     = 0.0  
    #bus_conditions.power.mechanical[:,0]     = 0.0  
    bus_conditions.power.electrical[:,0]     = 0.0  
    #bus_conditions.power.chemical[:,0]       = 0.0  
    #bus_conditions.power.pneumatic[:,0]      = 0.0  
    #bus_conditions.power.hydraulic[:,0]      = 0.0  
    #bus_conditions.power.thermal[:,0]        = 0.0    

    if bus.assigned_distributors != None:
        for distributor_tag in bus.assigned_distributors[0]:     
            bus_conditions.links[distributor_tag].power.electrical[:,0] = 0.0  
    
    '''NEED TO FIX MATTHEW '''
    # Thermal power draw
    if segment.state.initials:
        for network in segment.analyses.vehicle.networks:
            for distributor in  network.distributors:
                if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line):
                    for tag, item in distributor.items():
                        if tag == 'battery_modules':
                            for battery in item:
                                for btms in  battery:
                                    bus_conditions.power.electrical[0,0]   +=  segment.state.initials.conditions.energy.distributors[distributor.tag][btms.tag].power.electrical[-1] 
                        if tag == 'heat_exchangers':
                            for heat_exchanger in  item:                    
                                bus_conditions.power.electrical[0,0]   +=  segment.state.initials.conditions.energy.distributors[distributor.tag][heat_exchanger.tag].power.electrical[-1] 
        # Bus Properties 
        #bus_initials            = segment.state.initials.conditions.energy.distributors[bus.tag]
        #if type(segment) ==  RCAIDE.Framework.Mission.Segments.Ground.Battery_Recharge:             
            #bus_initials.battery_discharge_flag           = False 
        #else:                   
            #bus_initials.battery_discharge_flag           = True     
        #bus_conditions.energy[0,0]          = bus_initials.energy[-1,0]


    return