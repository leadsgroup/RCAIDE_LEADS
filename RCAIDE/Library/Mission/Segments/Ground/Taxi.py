# RCAIDE/Library/Missions/Segments/Ground/Taxi.py
#
#
# Created: Aug 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------
def initialize_conditions(segment):
    """
    Initializes conditions for a constant-speed ground taxi segment.

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - altitude : float
                Ground altitude [m]
            - air_speed : float
                Ground speed to hold [m/s]
            - distance : float
                Ground distance to cover [m]
            - ground_incline : float
                Runway incline angle [rad]
            - friction_coefficient : float
                Ground friction coefficient [-]

    Returns
    -------
    None
        Updates segment conditions directly:
            - conditions.frames.inertial.velocity_vector [m/s]
            - conditions.frames.inertial.position_vector [m]
            - conditions.frames.inertial.time [s]
            - conditions.freestream.altitude [m]
            - conditions.ground.incline [rad]
            - conditions.ground.friction_coefficient [-]

    Notes
    -----
    Same distance/speed -> time relation as Constant_Speed_Constant_Altitude
    (t = x/V), plus the rolling-friction ground conditions consumed by
    RCAIDE.Library.Mission.Common.Update.ground_forces.
    """
    # unpack inputs
    alt       = segment.altitude
    xf        = segment.distance
    air_speed = segment.air_speed

    # check for initial airspeed / altitude
    if air_speed is None:
        if not segment.state.initials: raise AttributeError('air_speed not set')
        air_speed = np.linalg.norm(segment.state.initials.conditions.frames.inertial.velocity_vector[-1])

    if alt is None:
        if not segment.state.initials: raise AttributeError('altitude not set')
        alt = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1,2]

    # dimensionalize time
    conditions = segment.state.conditions
    t_initial  = conditions.frames.inertial.time[0,0]
    t_final    = xf / air_speed + t_initial
    t_nondim   = segment.state.numerics.dimensionless.control_points
    time       = t_nondim * (t_final - t_initial) + t_initial

    # pack conditions
    conditions.freestream.altitude[:,0]             = alt
    conditions.frames.inertial.position_vector[:,2] = -alt
    conditions.frames.inertial.velocity_vector[:,0] = air_speed
    conditions.frames.inertial.time[:,0]            = time[:,0]
    conditions.ground.incline[:,0]                  = segment.ground_incline
    conditions.ground.friction_coefficient[:,0]     = segment.friction_coefficient

    return
