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
    alt       = segment.altitude
    time      = segment.time
    v0        = segment.velocity   

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
    conditions.weights.total_mass[:,0]              = segment.analyses.vehicle.mass_properties.takeoff
    conditions.frames.inertial.position_vector[:,:] = conditions.frames.inertial.position_vector[0,:][None,:][:,:]