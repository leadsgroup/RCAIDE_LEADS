# RCAIDE/Library/Missions/Segments/Cruise/Curved_Constant_Radius_Constant_Speed_Constant_Altitude/initialize_conditions.py
# 
# 
# Created:  September 2024, A. Molloy + M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# Package imports 
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------
def initialize_conditions(segment):
    """Sets the specified conditions which are given for the segment type.

    Assumptions:
    Curved segment with constant radius, constant speed and constant altitude
    Assumes that it is a coordinated turn (true course and true heading are aligned)

    Source:
    N/A

    Inputs:
    segment.altitude                [meters] # deleted segment distance as it is now defined by the turn_angle and radius.
    segment.speed                   [meters/second]
    self.start_true_course          [degrees] true course of the vehicle before the turn
    self.turn_angle                 [degrees] angle measure of the curve. + is right hand turn, - is left hand turn. 
    self.radius                     [meters] radius of the turn
    
    Outputs: ***pretty sure that no additional outputs are need. Verify this***
    conditions.frames.inertial.velocity_vector  [meters/second]
    conditions.frames.inertial.position_vector  [meters]
    conditions.freestream.altitude              [meters]
    conditions.frames.inertial.time             [seconds]

    Properties Used:
    N/A
    """        
    
    # unpack 
    alt               = segment.altitude
    air_speed         = segment.air_speed       
    beta              = segment.sideslip_angle
    radius            = segment.turn_radius
    start_true_course = segment.true_course
    arc_sector        = segment.turn_angle
    conditions        = segment.state.conditions 

    # check for initial velocity
    if air_speed is None: 
        if not segment.state.initials: raise AttributeError('airspeed not set')
        air_speed = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])
        
    # check for initial altitude
    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]
    
    # check for turn radius
    if radius is None:
        if not segment.state.initials: raise AttributeError('radius not set')
        radius = 0.1 # minimum radius so as to approximate a near instantaneous curve
    
    # check for turn angle
    if arc_sector is None:
        if not segment.state.initials: raise AttributeError('turn angle not set')
        arc_sector = 0.0 # aircraft does not turn    

    # dimensionalize time
    v_body_x    = np.cos(beta)*air_speed # x-velocity in the body frame. 
    v_body_y    = np.sin(beta)*air_speed # y-velocity in the body frame
    t_initial   = conditions.frames.inertial.time[0,0]
    omega       = v_body_x / radius
    t_final     = abs(arc_sector) / omega + t_initial  # Time to complete the turn
    t_nondim    = segment.state.numerics.dimensionless.control_points
    time        = t_nondim * (t_final-t_initial) + t_initial
    
    true_course_control_points = start_true_course + t_nondim * arc_sector
    
    v_inertial_x = air_speed * np.cos(true_course_control_points)
    v_inertial_y = air_speed * np.sin(true_course_control_points)
    
    # pack
    segment.state.conditions.freestream.altitude[:,0]             = alt
    segment.state.conditions.frames.inertial.position_vector[:,2] = -alt # z points down
    segment.state.conditions.frames.inertial.velocity_vector[:,0] = v_inertial_x[:,0]
    segment.state.conditions.frames.inertial.velocity_vector[:,1] = v_inertial_y[:,0]
    segment.state.conditions.frames.body.velocity_vector[:,0]     = v_body_x
    segment.state.conditions.frames.body.velocity_vector[:,1]     = v_body_y
    segment.state.conditions.frames.inertial.time[:,0]            = time[:,0]
    segment.state.conditions.frames.planet.true_heading[:,0]      = true_course_control_points[:,0]
    segment.state.conditions.frames.planet.true_course[:,0]       = true_course_control_points[:,0]