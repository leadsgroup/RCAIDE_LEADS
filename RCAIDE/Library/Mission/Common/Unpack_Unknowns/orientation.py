# RCAIDE/Library/Missions/Common/Unpack_Unknowns/orientation.py
# 
# 
# Created:  Jul 2023, M. Clarke
# ----------------------------------------------------------------------------------------------------------------------
#  Unpack Unknowns
# ----------------------------------------------------------------------------------------------------------------------
def orientation(segment): 
        
    ctrls    = segment.assigned_control_variables 

    # Body Angle Control 
    if segment.trim_lift_coefficient !=  None:
        segment.state.conditions.aerodynamics.coefficients.lift.total  = segment.trim_lift_coefficient * segment.state.ones_row(1)
    else: 
        if ctrls.body_angle.active: 
            segment.state.conditions.frames.body.inertial_rotations[:,1] = segment.state.unknowns.body_angle[:,0]  
        else: 
            segment.state.conditions.frames.body.inertial_rotations[:,1] = segment.angle_of_attack            



    if ctrls.bank_angle.active: 
        segment.state.conditions.frames.body.inertial_rotations[:,0] = segment.state.unknowns.bank_angle[:,0]
    else:
        segment.state.conditions.frames.body.inertial_rotations[:,0] = segment.bank_angle
        
    segment.state.conditions.frames.body.inertial_rotations[:,2] =  segment.state.conditions.frames.planet.true_heading[:,0] 
    
    # Velocity Control
    if ctrls.velocity.active:
        segment.state.conditions.frames.inertial.velocity_vector[:,0] = segment.state.unknowns.velocity[:,0]
        
    # Altitude Control
    if ctrls.altitude.active:
        segment.state.conditions.frames.inertial.position_vector[:,2] = -segment.state.unknowns.altitude[:,0]
        
    return 
            
            
            