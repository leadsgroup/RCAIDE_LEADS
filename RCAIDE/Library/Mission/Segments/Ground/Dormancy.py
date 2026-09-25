# RCAIDE/Library/Mission/Segments/Ground/Dormancy.py
#
#
# Created: Aug 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------------------------------------------------------
def initialize_conditions(segment):
    """
    Initializes conditions for a stationary ground-hold segment (pre-flight or
    post-flight gate dormancy, turnaround, overnight parking) of prescribed
    duration.

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - time : float
                Duration to hold [s]

    Returns
    -------
    None
        Updates segment conditions directly:
            - conditions.frames.inertial.time [s]

    Notes
    -----
    No flight dynamics are solved here -- the aircraft is not moving. The
    vehicle's own energy-network sources (fuel tanks, batteries, ...) still
    evolve over the held duration via the segment's normal network/energy
    update, the same as every other segment type.
    """
    t_initial = segment.state.conditions.frames.inertial.time[0,0]
    t_nondim  = segment.state.numerics.dimensionless.control_points
    time      = t_nondim * segment.time + t_initial
    segment.state.conditions.frames.inertial.time[:,0] = time[:,0]
