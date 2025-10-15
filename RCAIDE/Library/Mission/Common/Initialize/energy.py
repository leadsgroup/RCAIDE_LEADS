# RCAIDE/Library/Missions/Common/Initialize/energy.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Sep 2024, S. Shekar

import RCAIDE

# ----------------------------------------------------------------------------------------------------------------------
#  energy
# ----------------------------------------------------------------------------------------------------------------------  
def energy(segment):
    """
    Initializes energy states for vehicle networks at mission segment start

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function initializes energy-related conditions for all energy networks
    in the vehicle, including batteries, fuel systems, and thermal management
    systems. It handles both electrical and fuel-based energy storage systems.

    The function processes:
    1. Electrical networks with busses
        - Battery module conditions
        - Thermal management systems
            * Battery cooling systems
            * Heat exchangers
            * Coolant reservoirs
    2. Fuel-based networks
        - Fuel tank conditions
        - Fuel mass tracking

    **Required Segment Components**

    segment.analyses.energy.vehicle.networks:
        Network configurations containing:
        - Electrical busses with battery modules
        - Cooling systems and heat exchangers
        - Fuel lines and tanks

    **State Variables**

    conditions.energy:
        For electrical systems:
        - Battery states
        - Thermal conditions
        - Coolant properties

        For fuel systems:
        - Fuel mass
        - Tank conditions

    **Major Assumptions**
        * Well-defined network architecture
        * Valid initial conditions
        * Compatible energy storage systems
        * Proper thermal management setup

    Returns
    -------
    None
        Updates segment conditions directly

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """ 

    conditions = segment.state.conditions.energy
    vehicle    = segment.analyses.energy.vehicle

    # loop through battery modules in networks
    for network in vehicle.networks:
        # if network has busses  
        for distributor in network.distributors:
            if isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus):
                distributor.append_segment_conditions(segment)
                for battery_module_tag in  distributor.assigned_sources:
                    for i in range(len(battery_module_tag)):
                        battery_module =  vehicle.networks[network.tag].sources[battery_module_tag[i]]
                        battery_module.append_battery_segment_conditions(segment)
                for coolant_line in  network.coolant_lines:
                    for tag, item in  coolant_line.items(): 
                        if tag == 'battery_modules':
                            for battery in item:
                                for btms in  battery:
                                    btms.append_segment_conditions(segment,coolant_line)
                        if tag == 'heat_exchangers':
                            for heat_exchanger in  item:
                                heat_exchanger.append_segment_conditions(segment,distributor,coolant_line)
                        if tag == 'reservoirs':
                            for reservoir in  item:
                                reservoir.append_segment_conditions(segment, coolant_line)
        
            elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                        
                distributor.append_segment_conditions(segment)
                for fuel_tank in distributor.assigned_sources:
                    if segment.state.initials :
                        fuel_tank_initials = segment.state.initials.conditions.energy.sources[fuel_tank[0]]
                        conditions.sources[fuel_tank[0]].fuel_mass[:,0]   = fuel_tank_initials.fuel_mass[-1,0]
                    elif  vehicle.networks[network.tag].sources[fuel_tank[0]].fuel != None:
                        conditions.sources[fuel_tank[0]].fuel_mass[:,0]   = vehicle.networks[network.tag].sources[fuel_tank[0]].fuel.mass_properties.mass