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
    """ Initializes the differntial altitude
    
        Assumptions: 
            None
        
        Inputs: 
            state.conditions:           
                numerics.dimensionless.integrate        [-]
                numerics.dimensionless.control_points   [s]
                frames.position_vector                  [m]
                inertial.velocity_vector                [m/s]
            
        Outputs:
            state.conditions.frames.inertial.time       [s]
           
        Properties Used:
        N/A 
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