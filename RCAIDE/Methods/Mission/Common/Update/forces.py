## @ingroup Methods-Missions-Segments-Common-Update
# RCAIDE/Methods/Missions/Common/Update/forces.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Core  import  orientation_product  

# ----------------------------------------------------------------------------------------------------------------------
#  Update Forces
# ----------------------------------------------------------------------------------------------------------------------
## @ingroup Methods-Missions-Segments-Common-Update
def forces(segment): 
    """  
    """    

    # unpack
    conditions                    = segment.state.conditions 
    wind_lift_force_vector        = conditions.frames.wind.lift_force_vector
    wind_drag_force_vector        = conditions.frames.wind.drag_force_vector
    body_thrust_force_vector      = conditions.frames.body.thrust_force_vector
    inertial_gravity_force_vector = conditions.frames.inertial.gravity_force_vector

    # unpack transformation matrices
    T_body2inertial = conditions.frames.body.transform_to_inertial
    T_wind2inertial = conditions.frames.wind.transform_to_inertial

    # to inertial frame
    L = orientation_product(T_wind2inertial,wind_lift_force_vector)
    D = orientation_product(T_wind2inertial,wind_drag_force_vector)
    T = orientation_product(T_body2inertial,body_thrust_force_vector)
    W = inertial_gravity_force_vector

    # sum of the forces
    F = L + D + T + W
    # like a boss

    # pack
    conditions.frames.inertial.total_force_vector[:,:] = F[:,:]

    return
 