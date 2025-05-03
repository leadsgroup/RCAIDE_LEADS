# RCAIDE/Library/Missions/Common/Initialize/energy.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Sep 2024, S. Shekar

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
        for bus in network.busses:
            for fuel_tank in bus.fuel_tanks:
                if segment.state.initials:
                    bus_initials       = segment.state.initials.conditions.energy.busses[bus.tag]
                    fuel_tank_initials = bus_initials[fuel_tank.tag]
                    conditions.busses[bus.tag].fuel_tanks[fuel_tank.tag].mass[:,0]   = fuel_tank_initials.mass[-1,0]
                elif vehicle.networks[network.tag].busses[bus.tag].fuel_tanks[fuel_tank.tag].fuel != None:
                        conditions.busses[bus.tag].fuel_tanks[fuel_tank.tag].mass[:,0]  = vehicle.networks[network.tag].busses[bus.tag].fuel_tanks[fuel_tank.tag].fuel.mass_properties.mass
            bus.append_segment_conditions(segment)
            for battery_module in  bus.battery_modules:
                battery_module.append_battery_segment_conditions(segment, bus)
            for coolant_line in  network.coolant_lines:
                for tag, item in  coolant_line.items(): 
                    if tag == 'battery_modules':
                        for battery in item:
                            for btms in  battery:
                                btms.append_segment_conditions(segment,coolant_line)
                    if tag == 'heat_exchangers':
                        for heat_exchanger in  item:
                            heat_exchanger.append_segment_conditions(segment,bus,coolant_line)
                    if tag == 'reservoirs':
                        for reservoir in  item:
                            reservoir.append_segment_conditions(segment, coolant_line)
                    
        # if network has fuel lines             
        for fuel_line in  network.fuel_lines:
            fuel_line.append_segment_conditions(segment)
            for fuel_tank in fuel_line.fuel_tanks:
                if segment.state.initials:
                    fuel_line_initials = segment.state.initials.conditions.energy.fuel_lines[fuel_line.tag]
                    fuel_tank_initials = fuel_line_initials.fuel_tanks[fuel_tank.tag]
                    conditions.fuel_lines[fuel_line.tag].fuel_tanks[fuel_tank.tag].mass[:,0]   = fuel_tank_initials.mass[-1,0]
                elif  vehicle.networks[network.tag].fuel_lines[fuel_line.tag].fuel_tanks[fuel_tank.tag].fuel != None:
                    conditions.fuel_lines[fuel_line.tag].fuel_tanks[fuel_tank.tag].mass[:,0]   = vehicle.networks[network.tag].fuel_lines[fuel_line.tag].fuel_tanks[fuel_tank.tag].fuel.mass_properties.mass