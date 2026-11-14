# RCAIDE/Library/Missions/Common/Update/planet_position.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Framework.Core  import Units, array_namespace

# package imports 
import numpy as np
 
# ----------------------------------------------------------------------------------------------------------------------
#  Update Planet Position
# ----------------------------------------------------------------------------------------------------------------------
def planet_position(segment):
    """ Updates the location of the vehicle relative to the Planet throughout the mission
    
        Assumptions:
        This is valid for small movements and times as it does not account for the rotation of the Planet beneath the vehicle
        
        Inputs:
        segment.state.conditions:
            freestream.velocity                      [meters/second]
            freestream.altitude                      [meters]
            frames.body.inertial_rotations           [Radians]
        segment.analyses.planet.mean_radius [meters]
        segment.state.numerics.time.integrate        [float]
            
        Outputs:
            segment.state.conditions:           
                frames.planet.latitude  [Radians]
                frames.planet.longitude [Radians]

        Properties Used:
        N/A
                                
    """        
    
    # unpack
    conditions = segment.state.conditions
    
    # unpack orientations and velocities
    V          = conditions.freestream.velocity[:,0]
    altitude   = conditions.freestream.altitude[:,0] 
    theta      = conditions.frames.body.inertial_rotations[:,1]
    psi        = segment.true_course        
    alpha      = conditions.aerodynamics.angles.alpha[:,0]
    I          = segment.state.numerics.time.integrate
    Re         = segment.analyses.planet.mean_radius  
         
    # The flight path and radius
    gamma     = theta - alpha
    R         = altitude + Re

    # Find the velocities and integrate the positions
    xp        = array_namespace(V, theta, alpha, psi)   # numpy, or jax.numpy when any of them is a JAX array
    lamdadot  = (V/R)*xp.cos(gamma)*xp.cos(psi)
    lamda     = xp.dot(I,lamdadot) / Units.deg # Latitude
    mudot     = (V/R)*xp.cos(gamma)*xp.sin(psi)/xp.cos(lamda)
    mu        = xp.dot(I,mudot) / Units.deg # Longitude

    # Reshape the size of the vectorss
    shape     = np.shape(conditions.freestream.velocity)
    mu        = xp.reshape(mu,shape)
    lamda     = xp.reshape(lamda,shape)
    cos_psi, sin_psi = xp.cos(psi) + 0., xp.sin(psi) + 0.
    phi       = xp.stack([xp.stack([cos_psi, -sin_psi, 0*cos_psi]), xp.stack([sin_psi, cos_psi, 0*cos_psi]), xp.stack([0*cos_psi, 0*cos_psi, 1 + 0*cos_psi])])

    # Pack 
    lat                                           = conditions.frames.planet.latitude[0,0]
    lon                                           = conditions.frames.planet.longitude[0,0]
    conditions.frames.planet.latitude             = lat + lamda
    conditions.frames.planet.longitude            = lon + mu
    conditions.frames.planet.true_course          = xp.broadcast_to(phi, conditions.frames.planet.true_course.shape)   # same matrix at every point

    return 