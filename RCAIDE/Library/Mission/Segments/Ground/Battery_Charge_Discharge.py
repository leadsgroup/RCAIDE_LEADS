# RCAIDE/Library/Missions/Segments/Ground/Battery_Charge_Discharge.py
# 
# 
# Created:  Jul 2023, M. Clarke 
import RCAIDE 
from RCAIDE.Framework.Core import  Units
import  numpy as  np 
# ----------------------------------------------------------------------------------------------------------------------  
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------    
def initialize_conditions(segment):  
    """
    Initializes conditions for battery charging or discharging on ground

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - cutoff_SOC : float
                Target state of charge to reach [-]
            - cooling_time : float
                Additional time for battery cooling [s]
            - initial_battery_state_of_charge : float, optional
                Initial SOC if no previous segment [-]
            - analyses:
                energy:
                    vehicle:
                        networks : list
                            List of power networks
                            Each network contains:
                            - busses : list
                                List of electrical busses
                                Each bus contains:
                                - charging_c_rate : float
                                    Battery charging C-rate [-]
                                - battery_modules : list
                                    List of battery modules
        
        For discharging segments:
            - time : float
                Duration of discharge [s]
    
    Returns
    -------
    None
        Updates segment conditions directly:
            - conditions.frames.inertial.time [s]
    
    Notes
    -----
    This function sets up the initial conditions for a ground segment involving 
    battery charging or discharging. For charging segments, the charging time is 
    determined by the battery with the lowest state of charge.

    **Calculation Process**
    For charging:
        1. Find lowest battery state of charge
        2. Calculate charging time based on:
            t = (SOC_target - SOC_current) / C_rate
        3. Add cooling time
        4. Discretize time points

    For discharging:
        1. Use specified discharge time
        2. Discretize time points

    **Major Assumptions**
        * Charging time determined by lowest SOC battery
        * Constant C-rate charging
        * Linear SOC increase during charging
        * Uniform cooling time added

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """    
    t_nondim   = segment.state.numerics.dimensionless.control_points

    vehicle = segment.analyses.energy.vehicle

    for network in vehicle.networks:
        for bus in  network.busses:
            bus.append_operating_conditions(segment)
            for battery_module in  bus.battery_modules:
                battery_module.append_operating_conditions(segment,bus)

            for fuel_cell_stack in  bus.fuel_cell_stacks:
                fuel_cell_stack.append_operating_conditions(segment,bus)

            for tag, bus_item in bus.items():
                if issubclass(type(bus_item), RCAIDE.Library.Components.Component):
                    bus_item.append_operating_conditions(segment,bus)

            for fuel_tank in  bus.fuel_tanks:
                fuel_tank.append_operating_conditions(segment,bus)


    if isinstance(segment, RCAIDE.Framework.Mission.Segments.Ground.Battery_Recharge):
        for network in vehicle.networks:
            time =  []
            for bus in  network.busses:
                t=0
                if segment.state.initials.keys():
                    end_of_flight_soc = 1
                    for battery_module in segment.state.conditions.energy.busses[bus.tag].battery_modules:
                        end_of_flight_soc = min(end_of_flight_soc,battery_module.cell.state_of_charge[-1])
                else:
                    end_of_flight_soc =  segment.initial_battery_state_of_charge
                
                t           =  max(((segment.cutoff_SOC-end_of_flight_soc) / bus.charging_c_rate )*Units.hrs  , t) 
                t           += segment.cooling_time
                time.append(t)
            t_initial = segment.state.conditions.frames.inertial.time[0,0]
            t_nondim  = segment.state.numerics.dimensionless.control_points
            time      = np.max(time)
            charging_time      = t_nondim * ( time ) + t_initial 
            segment.state.conditions.frames.inertial.time[:,0] = charging_time[:,0]

    else:

        t_initial = segment.state.conditions.frames.inertial.time[0,0]
        t_nondim  = segment.state.numerics.dimensionless.control_points
        time      = t_nondim * ( segment.time ) + t_initial
        segment.state.conditions.frames.inertial.time[:,0] = time[:,0]