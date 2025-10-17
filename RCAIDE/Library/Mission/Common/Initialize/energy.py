# RCAIDE/Library/Missions/Common/Initialize/energy.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Sep 2024, S. Shekar
# Modified: Sep 2025, M. Guidotti

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
                # iterate all sources; pick battery modules assigned to this electrical bus
                for source in network.sources:
                    # flatten assigned distributor TAGs
                    assigned_tags = []
                    for grp in source.assigned_distributors:
                        if isinstance(grp, (list, tuple, set)):
                            assigned_tags.extend(list(grp))
                        else:
                            assigned_tags.append(grp)
                    # if this source is connected to the current bus and is a battery module, append its conditions
                    if distributor.tag in assigned_tags and issubclass(type(source), RCAIDE.Library.Components.Powertrain.Sources.Battery_Modules.Generic_Battery_Module):
                        battery_module = vehicle.networks[network.tag].sources[source.tag]
                        battery_module.append_battery_segment_conditions(segment)
                
                # coolant line components 
                for coolant_line in network.distributors:
                    if isinstance(coolant_line, RCAIDE.Library.Components.Powertrain.Distributors.Coolant_Line):
                        for tag, item in coolant_line.items(): 
                            if tag == 'battery_modules':
                                for battery in item:
                                    for btms in battery:
                                        btms.append_segment_conditions(segment, coolant_line)
                            if tag == 'heat_exchangers':
                                for heat_exchanger in item:
                                    heat_exchanger.append_segment_conditions(segment, distributor, coolant_line)
                            if tag == 'reservoirs':
                                for reservoir in item:
                                    reservoir.append_segment_conditions(segment, coolant_line)
        
            elif isinstance(distributor, RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line):
                
                distributor.append_segment_conditions(segment)
                # iterate all sources; pick fuel tanks assigned to this fuel line
                for source in network.sources:
                    # flatten assigned distributor TAGs
                    assigned_tags = []
                    for grp in source.assigned_distributors:
                        if isinstance(grp, (list, tuple, set)):
                            assigned_tags.extend(list(grp))
                        else:
                            assigned_tags.append(grp)
                    # if this source is connected to the current fuel line and is a fuel tank, initialize fuel mass
                    if distributor.tag in assigned_tags and issubclass(type(source), RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
                        src_tag = source.tag
                        if segment.state.initials:
                            fuel_tank_initials = segment.state.initials.conditions.energy.sources[src_tag]
                            conditions.sources[src_tag].fuel_mass[:,0] = fuel_tank_initials.fuel_mass[-1,0]
                        elif vehicle.networks[network.tag].sources[src_tag].fuel is not None:
                            conditions.sources[src_tag].fuel_mass[:,0] = vehicle.networks[network.tag].sources[src_tag].fuel.mass_properties.mass