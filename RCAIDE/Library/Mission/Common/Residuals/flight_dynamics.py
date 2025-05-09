# RCAIDE/Library/Missions/Common/Residuals/flight_dynamics.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

import RCAIDE
import numpy as np 
from RCAIDE.Framework.Core   import orientation_product, orientation_transpose 

# ----------------------------------------------------------------------------------------------------------------------
#  Residual Total Forces
# ---------------------------------------------------------------------------------------------------------------------- 
def flight_dynamics(segment):
    """
    Evaluates flight dynamics residuals for mission segment analysis.
    
    Parameters
    ----------
    segment : Segment
        The mission segment being analyzed
            - state : Data
                Contains conditions, numerics, and residuals
            - analyses : Data
                Contains vehicle analysis objects
            - flight_dynamics : Data
                Contains flags for which equations to evaluate
    
    Returns
    -------
    None
        Updates segment residuals directly in segment.state.residuals
    
    Notes
    -----
    This function calculates the residuals for force and moment equations
    in all three axes. It handles special cases for transition and ground
    segments, including acceleration calculations and final velocity constraints.
    
    The function processes:
        1. Force equation residuals (F = ma)
        2. Moment equation residuals (M = Iα)
        3. Special handling for transition and ground segments
    
    **Required Segment State Variables**
    
    state.conditions.frames.inertial:
        - velocity_vector : array
            Vehicle velocity [m/s]
        - acceleration_vector : array
            Vehicle acceleration [m/s²]
        - total_force_vector : array
            Net forces [N]
        - total_moment_vector : array
            Net moments [N⋅m]
        - angular_velocity_vector : array
            Angular rates [rad/s]
        - angular_acceleration_vector : array
            Angular accelerations [rad/s²]
    
    state.conditions.weights:
        - total_mass : array
            Vehicle mass [kg]
    
    analyses.aerodynamics.vehicle.mass_properties:
        - moments_of_inertia.tensor : array
            Inertia tensor [kg⋅m²]
    
    **Segment Types**
    
    Special handling for:
    - Transition segments
        * Constant acceleration
        * Constant angle
        * Linear climb
    - Ground segments
        * Takeoff
        * Landing
        * Ground operations
    
    **Major Assumptions**
        * Rigid body dynamics
        * Principal axes aligned with body axes
        * Constant inertia properties during segment
        * Valid mass properties
        * Non-zero final velocity for ground segments (minimum 0.01 m/s)
    
    **Theory**
    
    Force equations (Newton's Second Law):
    
    .. math::
        \\frac{F_i}{m} - a_i = 0
    
    Moment equations:
    
    .. math::
        \\frac{M_i}{I_{ii}} - \\alpha_i = 0
    
    where :math:`i` represents each axis (x, y, z), :math:`F` is force, 
    :math:`a` is acceleration, :math:`M` is moment, :math:`I` is moment of inertia,
    and :math:`\\alpha` is angular acceleration.
    
    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    RCAIDE.Framework.Mission.Segments.Transition
    RCAIDE.Framework.Mission.Segments.Ground
    """

    T_wind2inertial = segment.state.conditions.frames.wind.transform_to_inertial  
    T_inertia2wind  = orientation_transpose(T_wind2inertial)
    
    transition_seg_flag =  type(segment) == RCAIDE.Framework.Mission.Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb
    ground_seg_flag =  (type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Landing) or\
        (type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Takeoff) or \
        (type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Ground)

    if transition_seg_flag or ground_seg_flag: 
        v       = segment.state.conditions.frames.inertial.velocity_vector
        D       = segment.state.numerics.time.differentiate 
        segment.state.conditions.frames.inertial.acceleration_vector = np.dot(D,v)

    FT_i = segment.state.conditions.frames.inertial.total_force_vector
    a_i  = segment.state.conditions.frames.inertial.acceleration_vector   

    if transition_seg_flag: 
        omega = segment.state.conditions.frames.inertial.angular_velocity_vector
        D   = segment.state.numerics.time.differentiate
        ang_acc_i = np.dot(D,omega)
        segment.state.conditions.frames.inertial.angular_acceleration_vector = ang_acc_i 
        segment.state.conditions.frames.wind.angular_acceleration_vector     = orientation_product(T_inertia2wind,ang_acc_i )

    MT_i      = segment.state.conditions.frames.inertial.total_moment_vector    
    ang_acc_i = segment.state.conditions.frames.inertial.angular_acceleration_vector   
    m         = segment.state.conditions.weights.total_mass
    I         = segment.analyses.aerodynamics.vehicle.mass_properties.moments_of_inertia.tensor  
 
    if ground_seg_flag:
        vf = segment.velocity_end
        if vf == 0.0: vf = 0.01 
        segment.state.residuals.force_x[:,0] = FT_i[1:,0]/m[1:,0] - a_i[1:,0] 
        segment.state.residuals.final_velocity_error = (v[-1,0] - vf)
    else: 
        if segment.flight_dynamics.force_x: 
            segment.state.residuals.force_x[:,0] = FT_i[:,0]/m[:,0] - a_i[:,0]  
        if segment.flight_dynamics.force_y: 
            segment.state.residuals.force_y[:,0] = FT_i[:,1]/m[:,0] - a_i[:,1]    
        if segment.flight_dynamics.force_z: 
            segment.state.residuals.force_z[:,0] = FT_i[:,2]/m[:,0] - a_i[:,2]  
        if  segment.flight_dynamics.moment_x:
            segment.state.residuals.moment_x[:,0] = MT_i[:,0]/I[0,0] - ang_acc_i[:,0]   
        if  segment.flight_dynamics.moment_y:
            segment.state.residuals.moment_y[:,0] = MT_i[:,1]/I[1,1] - ang_acc_i[:,1]   
        if  segment.flight_dynamics.moment_z:
            segment.state.residuals.moment_z[:,0] = MT_i[:,2]/I[2,2] - ang_acc_i[:,2] 
     
    return
