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
    """Sets the specified conditions which are given for the segment type.

    Assumptions: 
    During recharging, the charge time associated with the largest capacity battery pack is used 
    
    Source:
    N/A

    Inputs:
    segment.overcharge_contingency              [-]
    battery.                                    [-]

    Outputs: 
    conditions.frames.inertial.time             [seconds]

    Properties Used:
    N/A
    """    
    t_nondim   = segment.state.numerics.dimensionless.control_points

    if isinstance(segment, RCAIDE.Framework.Mission.Segments.Ground.Battery_Recharge):
        for network in segment.analyses.energy.vehicle.networks:
            time =  []
            for bus in  network.busses:
                t=0
                if segment.state.initials.keys():
                    end_of_flight_soc = 1
                    for battery_module in segment.state.conditions.energy[bus.tag].battery_modules:
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