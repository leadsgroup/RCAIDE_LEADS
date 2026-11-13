# RCAIDE/Library/Missions/Common/Unpack_Unknowns/orientation.py
#
#
# Created:  Jul 2023, M. Clarke
# Modified: Nov 2026, M. Clarke
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

from RCAIDE.Framework.Core import array_namespace, set_at, safe_sqrt

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
                    - pitch_angle : Control
                        Body angle control settings
                    - bank_angle : Control
                        Bank angle control settings
                    - velocity : Control
                        Velocity control settings
                    - altitude : Control
                        Altitude control settings
            - lift_coefficient : float, optional
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
                        - pitch_angle : array
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

    # set_at(A, np.s_[:,1], value) does A[:,1] = value and returns A. NumPy writes in place; JAX arrays cannot be
    # changed, so for JAX it returns a new array, which is why the result is always assigned back.
    ctrls     = segment.assigned_control_variables
    rotations = segment.state.conditions.frames.body.inertial_rotations   # columns: roll, pitch, heading

    # Body Angle Control
    if segment.lift_coefficient !=  None:
        segment.state.conditions.aerodynamics.coefficients.lift.total          = segment.lift_coefficient * segment.state.ones_row(1)
        segment.state.conditions.aerodynamics.coefficients.lift.inviscid.total = segment.lift_coefficient / segment.analyses.aerodynamics.settings.fuselage_lift_correction   * segment.state.ones_row(1)
    else:
        if ctrls.pitch_angle.active:
            rotations = set_at(rotations, np.s_[:,1], segment.state.unknowns.mission.pitch_angle[:,0])
        else:
            rotations = set_at(rotations, np.s_[:,1], segment.angle_of_attack)

    # Bank Angle
    if ctrls.bank_angle.active:
        rotations = set_at(rotations, np.s_[:,0], -segment.state.unknowns.mission.bank_angle[:,0])
    else:
        rotations = set_at(rotations, np.s_[:,0], -segment.bank_angle)
    rotations = set_at(rotations, np.s_[:,2], segment.state.conditions.frames.planet.true_heading[:,0])
    segment.state.conditions.frames.body.inertial_rotations = rotations
    inertial = segment.state.conditions.frames.inertial

    # Velocity Control
    if ctrls.velocity.active:
        inertial.velocity_vector = set_at(inertial.velocity_vector, np.s_[:,0], segment.state.unknowns.mission.velocity[:,0])

    # Altitude Control
    if ctrls.altitude.active:
        inertial.position_vector = set_at(inertial.position_vector, np.s_[:,2], -segment.state.unknowns.mission.altitude[:,0])

    # Sideslip Angle, after velocity control because a crosswind uses the airspeed at each control point
    wind = segment.state.conditions.frames.wind
    wind.body_rotations = set_at(wind.body_rotations, np.s_[:,2], sideslip_angle(segment))

    return


def sideslip_angle(segment):
    """Sideslip angle at each control point: a solver unknown, or from a crosswind, or prescribed on the segment."""
    if segment.assigned_control_variables.sideslip_angle.active:
        return segment.state.unknowns.mission.sideslip_angle[:,0]
    if segment.crosswind_speed != 0.0:
        # beta = arcsin(v/V), Etkin & Reid (1.6,4); zero where the vehicle is not moving
        V  = segment.state.conditions.frames.inertial.velocity_vector
        xp = array_namespace(V)
        V  = safe_sqrt(xp.sum(V**2, axis=1))
        # divide only where the vehicle moves; V_safe keeps the unused division finite, since JAX differentiates
        # both branches of a where and a division by zero there would make the gradient NaN
        moving = V > 0.
        V_safe = xp.where(moving, V, 1.)
        return xp.arcsin(xp.clip(xp.where(moving, segment.crosswind_speed / V_safe, 0.), -1.0, 1.0))
    return segment.sideslip_angle
