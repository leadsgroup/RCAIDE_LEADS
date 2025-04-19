# RCAIDE/Library/Missions/Common/Residuals/flight_dynamics.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

import RCAIDE
import numpy as np 

# ----------------------------------------------------------------------------------------------------------------------
#  Residual Total Forces
# ---------------------------------------------------------------------------------------------------------------------- 
def flight_dynamics(segment):
    '''Packs the residuals related to the flight dynamics.
       This includes the force and moment equations about x,y and z
    
    Assumptions:
        N/A
    
    Inputs:
        segment 
        
    Outputs:
        None   
    '''
    transition_seg_flag =  type(segment) == RCAIDE.Framework.Mission.Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb
    ground_seg_flag =  (type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Landing) or\
        (type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Takeoff) or \
        (type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Ground)

    if transition_seg_flag or ground_seg_flag: 
        v       = segment.state.conditions.frames.inertial.velocity_vector
        D       = segment.state.numerics.time.differentiate 
        segment.state.conditions.frames.inertial.acceleration_vector = np.dot(D,v)

    FT = segment.state.conditions.frames.inertial.total_force_vector
    a  = segment.state.conditions.frames.inertial.acceleration_vector    

    if transition_seg_flag: 
        omega = segment.state.conditions.frames.inertial.angular_velocity_vector
        D  = segment.state.numerics.time.differentiate   
        segment.state.conditions.frames.inertial.angular_acceleration_vector =  np.dot(D,omega)         

    MT      = segment.state.conditions.frames.inertial.total_moment_vector    
    ang_acc = segment.state.conditions.frames.inertial.angular_acceleration_vector  
    m       = segment.state.conditions.weights.total_mass
    I       = segment.analyses.aerodynamics.vehicle.mass_properties.moments_of_inertia.tensor  

    if ground_seg_flag:
        vf = segment.velocity_end
        if vf == 0.0: vf = 0.01 
        segment.state.residuals.force_x[:,0] = FT[1:,0]/m[1:,0] - a[1:,0] 
        segment.state.residuals.final_velocity_error = (v[-1,0] - vf)
    else: 
        if segment.flight_dynamics.force_x: 
            segment.state.residuals.force_x[:,0] = FT[:,0]/m[:,0] - a[:,0]  
        if segment.flight_dynamics.force_y: 
            segment.state.residuals.force_y[:,0] = FT[:,1]/m[:,0] - a[:,1]    
        if segment.flight_dynamics.force_z: 
            segment.state.residuals.force_z[:,0] = FT[:,2]/m[:,0] - a[:,2]  
        if  segment.flight_dynamics.moment_x:
            segment.state.residuals.moment_x[:,0] = MT[:,0]/I[0,0] - ang_acc[:,0]   
        if  segment.flight_dynamics.moment_y:
            segment.state.residuals.moment_y[:,0] = MT[:,1]/I[1,1] - ang_acc[:,1]   
        if  segment.flight_dynamics.moment_z:
            segment.state.residuals.moment_z[:,0] = MT[:,2]/I[2,2] - ang_acc[:,2] 
     
    return
