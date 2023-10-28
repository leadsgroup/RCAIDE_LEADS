## @ingroup Methods-Missions-Segments-Common
# RCAIDE/Methods/Missions/Segments/Common/Update/dimensionless.py
# (c) Copyright 2023 Aerospace Research Community LLC
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
## @ingroup Methods-Missions-Segments-Common-Update
def differentials_altitude(segment):
    """  
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