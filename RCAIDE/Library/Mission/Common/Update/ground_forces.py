# RCAIDE/Library/Missions/Common/Update/ground_forces.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE  
from RCAIDE.Framework.Core import   orientation_product, set_at
import numpy as np
from RCAIDE.Library.Mission.Common.Update.forces import forces
 
# ----------------------------------------------------------------------------------------------------------------------
#  Compute Ground Forces
# ---------------------------------------------------------------------------------------------------------------------- 
def ground_forces(segment):
    """ Compute the rolling friction on the aircraft 
    
    Assumptions:
        Does a force balance to calculate the load on the wheels using only lift. Uses only a single friction coefficient.
    
    Source:
        N/A
    
    Inputs:
    conditions:
        frames.inertial.gravity_force_vector       [meters/second^2]
        ground.friction_coefficient                [unitless]
        frames.wind.force_vector                   [newtons]
    
    Outputs:
        conditions.frames.inertial.ground_force_vector [newtons]
    
    Properties Used:
        N/A
    """   

    # unpack
    conditions             = segment.state.conditions
    W                      = conditions.frames.inertial.gravity_force_vector[:,2,None]
    friction_coeff         = conditions.ground.friction_coefficient
    wind_force_vector      = conditions.frames.wind.force_vector

    #transformation matrix to get lift in inertial frame
    T_wind2inertial = conditions.frames.wind.transform_to_inertial

    # to inertial frame
    L = orientation_product(T_wind2inertial,wind_force_vector)[:,2,None]

    #compute friction force
    N  = -(W + L)
    Ff = N * friction_coeff

    #pack results. Friction acts along x-direction
    F_ground = conditions.frames.inertial.ground_force_vector
    F_ground = set_at(F_ground, np.s_[:,2], N[:,0])
    F_ground = set_at(F_ground, np.s_[:,0], Ff[:,0])
    conditions.frames.inertial.ground_force_vector = F_ground

    forces(segment)

    # unpack forces
    conditions                   = segment.state.conditions
    total_aero_forces            = conditions.frames.inertial.total_force_vector
    inertial_ground_force_vector = conditions.frames.inertial.ground_force_vector

    # sum of the forces, including friction force
    F = total_aero_forces + inertial_ground_force_vector

    # pack
    conditions.frames.inertial.total_force_vector = F
