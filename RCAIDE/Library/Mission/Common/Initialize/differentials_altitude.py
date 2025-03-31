# RCAIDE/Library/Missions/Segments/Common/Update/dimensionless.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Update Differentials Altitude
# ---------------------------------------------------------------------------------------------------------------------- 
def differentials_altitude(segment):
    """
    Initializes the differential altitude for mission segment time calculations

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed

    Notes
    -----
    This function calculates the time differentials based on altitude changes
    in the mission segment. It uses the vertical component of velocity and
    position to determine appropriate time steps for integration.

    The function performs the following steps:
    1. Extracts control points and integration operators
    2. Calculates total altitude change
    3. Determines time step based on vertical velocity
    4. Rescales time operators
    5. Updates segment time vector

    **Required Segment State Variables**

    state.numerics.dimensionless:
        - integrate : array
            Integration operator matrix
        - control_points : array
            Normalized time points

    state.conditions.frames.inertial:
        - position_vector : array
            Vehicle position in inertial frame [m]
        - velocity_vector : array
            Vehicle velocity in inertial frame [m/s]
        - time : array
            Segment time vector [s]

    **Major Assumptions**
    * Continuous vertical velocity
    * Well-defined altitude change
    * No singularities in vertical velocity

    Returns
    -------
    None
        Updates segment state directly

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """

    # unpack
    t = segment.state.numerics.dimensionless.control_points 
    I = segment.state.numerics.dimensionless.integrate
    r = segment.state.conditions.frames.inertial.position_vector
    v = segment.state.conditions.frames.inertial.velocity_vector

    dz = r[-1,2] - r[0,2]
    vz = v[:,2,None] # maintain column array

    # get overall time step
    dt = np.dot( I[-1,:] * dz , 1/ vz[:,0] )

    # rescale operators
    t = t * dt

    # pack
    t_initial = segment.state.conditions.frames.inertial.time[0,0]
    segment.state.conditions.frames.inertial.time[:,0] = t_initial + t[:,0]

    return