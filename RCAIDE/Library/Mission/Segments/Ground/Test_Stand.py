# RCAIDE/Library/Missions/Segments/Ground/Takeoff.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE Imports  
import RCAIDE 
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
# unpack unknowns
# ---------------------------------------------------------------------------------------------------------------------- 
def initialize_conditions(segment):
    """Sets the specified conditions which are given for the segment type.

    Assumptions:
    Builds on the initialize conditions for common

    Source:
    N/A

    Inputs:
    segment.throttle                           [unitless]
    conditions.frames.inertial.position_vector [meters]
    conditions.weights.total_mass              [kilogram]

    Outputs:
    conditions.weights.total_mass              [kilogram]
    conditions.frames.inertial.position_vector [unitless]
    conditions.propulsion.throttle             [meters]
    
    Properties Used:
    N/A
    """  

    # use the common initialization # unpack inputs
    alt      = segment.altitude
    time     = segment.time
    v0       = segment.velocity
    

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
                
    
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 *segment.state.initials.conditions.frames.inertial.position_vector[-1,2]   

    if v0  is None: 
        v0 = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])  

    t_initial = segment.state.conditions.frames.inertial.time[0,0]
    t_nondim  = segment.state.numerics.dimensionless.control_points
    time      = np.max(time)
    charging_time      = t_nondim * ( time ) + t_initial 
    segment.state.conditions.frames.inertial.time[:,0] = charging_time[:,0]
        
    # pack conditions 
    conditions = segment.state.conditions    
    conditions.frames.inertial.velocity_vector[:,0] = v0  
    conditions.freestream.altitude[:,0]             = alt
    conditions.frames.inertial.position_vector[:,2] = -alt   
    conditions.weights.total_mass[:,0]              = segment.analyses.weights.vehicle.mass_properties.takeoff
    conditions.frames.inertial.position_vector[:,:] = conditions.frames.inertial.position_vector[0,:][None,:][:,:]