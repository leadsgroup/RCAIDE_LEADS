# RCAIDE/Library/Missions/Common/Unpack_Unknowns/orientation.py
# 
# 
# Created:  Jul 2023, M. Clarke
# ----------------------------------------------------------------------------------------------------------------------
#  Unpack Unknowns
# ----------------------------------------------------------------------------------------------------------------------
def orientation(segment): 
    """
    Updates vehicle orientation states from solver unknowns

    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - assigned_control_variables : Data
                Control configurations
                    - body_angle : Control
                        Body angle control settings
                    - bank_angle : Control
                        Bank angle control settings
                    - velocity : Control
                        Velocity control settings
                    - altitude : Control
                        Altitude control settings
            - trim_lift_coefficient : float, optional
                Target lift coefficient for trim
            - angle_of_attack : float
                Fixed angle of attack [rad]
            - bank_angle : float
                Fixed bank angle [rad]
            - state : Data
                Current segment state
                - conditions : Data
                    Flight conditions
                        - frames.body.inertial_rotations : array
                            Body orientation angles [rad]
                        - frames.planet.true_heading : array
                            Vehicle heading [rad]
                        - frames.inertial.velocity_vector : array
                            Vehicle velocity [m/s]
                        - frames.inertial.position_vector : array
                            Vehicle position [m]
                - unknowns : Data
                    Solver variables
                        - body_angle : array
                            Body angle values [rad]
                        - bank_angle : array
                            Bank angle values [rad]
                        - velocity : array
                            Velocity values [m/s]
                        - altitude : array
                            Altitude values [m]
                - ones_row : function
                    Creates array of ones with specified columns

    Returns
    -------
    None
        Updates segment conditions directly

    Notes
    -----
    This function applies orientation-related solver values to the segment state,
    handling body angles, trim conditions, and trajectory controls. It manages
    both active control and fixed orientation cases.

    The function processes:
        1. Body angle control
            - Trim conditions
            - Active angle control
            - Fixed angle of attack
        2. Bank angle control
            - Active bank control
            - Fixed bank angle
        3. Heading alignment
        4. Velocity control
        5. Altitude control
    
    **Major Assumptions**
        * Valid angle definitions
        * Proper control assignments
        * Compatible trim conditions
        * Well-defined reference frames
    
    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """
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
        segment.state.conditions.frames.body.inertial_rotations[:,0] = -segment.state.unknowns.bank_angle[:,0]
    else:
        segment.state.conditions.frames.body.inertial_rotations[:,0] = -segment.bank_angle
        
    segment.state.conditions.frames.body.inertial_rotations[:,2] =  segment.state.conditions.frames.planet.true_heading[:,0] 
    
    # Velocity Control
    if ctrls.velocity.active:
        segment.state.conditions.frames.inertial.velocity_vector[:,0] = segment.state.unknowns.velocity[:,0]
        
    # Altitude Control
    if ctrls.altitude.active:
        segment.state.conditions.frames.inertial.position_vector[:,2] = -segment.state.unknowns.altitude[:,0]
        
    return 
            
            
            