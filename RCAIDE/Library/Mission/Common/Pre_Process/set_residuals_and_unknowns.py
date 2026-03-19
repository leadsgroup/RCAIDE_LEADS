# RCAIDE/Library/Missions/Common/Pre_Process/set_residuals_and_unknowns.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
import RCAIDE
from RCAIDE.Framework.Core import Units
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  set_residuals_and_unknowns
# ----------------------------------------------------------------------------------------------------------------------  
def set_residuals_and_unknowns(mission):
    """
    Sets up flight dynamics residuals and control variables for mission segments

    Parameters
    ----------
    mission : Mission
        The mission containing segments to be analyzed
            - state.ones_row : function
                Creates array of ones
            - assigned_control_variables : Data
                Control variable configurations
            - flight_dynamics : Data
                Force/moment flags
            - state.residuals.mission : Data
                Storage for residuals
            - state.unknowns.mission : Data
                Storage for unknowns

    Returns
    -------
    None
        Updates mission segment states directly
    
    Notes
    -----
    This function configures the flight dynamics problem for each segment by
    setting up force/moment residuals and initializing control variables.
    It handles a comprehensive set of flight controls and dynamics states.

    The function processes:
    1. Force and moment residuals (degrees of freedom)
    2. Control variable initialization including:
        - Body angles
        - Bank angles
        - Wind angles
        - Throttle settings
        - Velocity and acceleration
        - Time parameters
        - Control surface deflections
            * Elevator
            * Rudder
            * Flaps
            * Slats
            * Ailerons
        - Thrust vectoring

    **Control Variable Initialization**
    
    For each control:
        1. Check if active
        2. Use provided initial values if available
        3. Apply default values if needed
        4. Track number of controls

    **Major Assumptions**
        * Valid control configurations
        * Proper degrees of freedom setup
        * Compatible control assignments
        * Valid initial guess values
        * Units in standard format

    See Also
    --------
    RCAIDE.Framework.Mission.Segments
    """     
    for segment in mission.segments:  
        ones_row    = segment.state.ones_row
        ones_row_m1 = segment.state.ones_row_m1
        ctrls       = segment.assigned_control_variables
        dynamics    = segment.flight_dynamics
        
        if type(segment) == RCAIDE.Framework.Mission.Segments.Climb.Constant_Dynamic_Pressure_Constant_Angle or \
           type(segment) == RCAIDE.Framework.Mission.Segments.Climb.Constant_Mach_Constant_Angle:
            segment.state.residuals.mission.altitude      = ones_row(1) * 0.0   
            segment.state.number_of_mission_residuals     += 1
        
        # assign force and moment residuals i.e. degrees of freedom
        if dynamics.final_velocity_error == True:
            segment.state.residuals.mission.final_velocity_error = 0.0   
            segment.state.number_of_mission_residuals += 1
            
        if dynamics.force_x == True: 
            if type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Takeoff or \
               type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Landing:
                pass
            else:
                segment.state.residuals.mission.force_x = ones_row(1) *0
            segment.state.number_of_mission_residuals += 1 
        if dynamics.force_y == True:
            segment.state.residuals.mission.force_y = ones_row(1) *0 
            segment.state.number_of_mission_residuals += 1
        if dynamics.force_z == True:
            segment.state.residuals.mission.force_z = ones_row(1) *0 
            segment.state.number_of_mission_residuals += 1
        if dynamics.moment_x == True:
            segment.state.residuals.mission.moment_x = ones_row(1) *0 
            segment.state.number_of_mission_residuals += 1
        if dynamics.moment_y == True:
            segment.state.residuals.mission.moment_y = ones_row(1) *0  
            segment.state.number_of_mission_residuals += 1
        if dynamics.moment_z == True:
            segment.state.residuals.mission.moment_z = ones_row(1) *0 
            segment.state.number_of_mission_residuals += 1
           
        # Body Angle  
        if ctrls.body_angle.active:
            segment.state.number_of_mission_unknowns  += 1 
            if ctrls.body_angle.initial_guess_values !=  None:
                segment.state.unknowns.mission.body_angle = ones_row(1) * ctrls.body_angle.initial_guess_values[0][0]
            else:
                segment.state.unknowns.mission.body_angle = ones_row(1) * 3.0 * Units.degrees
                
            if ctrls.body_angle.bounds !=  None:
                segment.state.unknowns_lower_bounds.mission.body_angle = ctrls.body_angle.bounds[0][0] * ones_row(1)
                segment.state.unknowns_upper_bounds.mission.body_angle = ctrls.body_angle.bounds[0][1] * ones_row(1)
            else:
                segment.state.unknowns_lower_bounds.mission.body_angle =  -np.inf * ones_row(1) 
                segment.state.unknowns_upper_bounds.mission.body_angle =   np.inf * ones_row(1)
    
        # Bank Angle  
        if ctrls.bank_angle.active:
            segment.state.number_of_mission_unknowns  += 1 
            if ctrls.bank_angle.initial_guess_values !=  None:
                segment.state.unknowns.mission.bank_angle = ones_row(1) * ctrls.bank_angle.initial_guess_values[0][0]
            else:
                segment.state.unknowns.mission.bank_angle = ones_row(1) * 0.0 * Units.degrees
    
            if ctrls.bank_angle.bounds !=  None:
                segment.state.unknowns_lower_bounds.mission.bank_angle = ctrls.bank_angle.bounds[0][0] * ones_row(1)
                segment.state.unknowns_upper_bounds.mission.bank_angle = ctrls.bank_angle.bounds[0][1] * ones_row(1)
            else:
                segment.state.unknowns_lower_bounds.mission.bank_angle =  -np.inf * ones_row(1) 
                segment.state.unknowns_upper_bounds.mission.bank_angle =   np.inf * ones_row(1)                
                
        # Wing Angle  
        if ctrls.wind_angle.active:
            segment.state.number_of_mission_unknowns  += 1 
            if ctrls.wind_angle.initial_guess_values !=  None:
                segment.state.unknowns.mission.wind_angle = ones_row(1) * ctrls.wind_angle.initial_guess_values[0][0]
            else:
                segment.state.unknowns.mission.wind_angle = ones_row(1) * 1.0 * Units.degrees
    
            if ctrls.wind_angle.bounds !=  None:
                segment.state.unknowns_lower_bounds.mission.wind_angle = ctrls.wind_angle.bounds[0][0] * ones_row(1)
                segment.state.unknowns_upper_bounds.mission.wind_angle = ctrls.wind_angle.bounds[0][1] * ones_row(1)
            else:
                segment.state.unknowns_lower_bounds.mission.wind_angle =  -np.inf * ones_row(1) 
                segment.state.unknowns_upper_bounds.mission.wind_angle =   np.inf * ones_row(1)                 
            
        # Throttle
        if ctrls.throttle.active: 
            for i in range(len(ctrls.throttle.assigned_propulsors)):
                segment.state.number_of_mission_unknowns  += 1  
                if ctrls.throttle.initial_guess_values !=  None:
                    segment.state.unknowns.mission["throttle_" + str(i)] = ones_row(1) * ctrls.throttle.initial_guess_values[i][0] 
                else:
                    segment.state.unknowns.mission["throttle_" + str(i)] = ones_row(1) *  0.5 
        
                if ctrls.throttle.bounds !=  None:
                    segment.state.unknowns_lower_bounds.mission["throttle_" + str(i)] = ctrls.throttle.bounds[i][0] * ones_row(1)
                    segment.state.unknowns_upper_bounds.mission["throttle_" + str(i)] = ctrls.throttle.bounds[i][1] * ones_row(1)
                else:
                    segment.state.unknowns_lower_bounds.mission["throttle_" + str(i)] =  -np.inf * ones_row(1) 
                    segment.state.unknowns_upper_bounds.mission["throttle_" + str(i)] =   np.inf * ones_row(1)                      
                        
        # Thrust Vector  
        if ctrls.thrust_vector_angle.active: 
            for i in range(len(ctrls.thrust_vector_angle.assigned_propulsors)): 
                segment.state.number_of_mission_unknowns  += 1 
                if ctrls.thrust_vector_angle.initial_guess_values !=  None:
                    segment.state.unknowns.mission["thrust_vector_angle_" + str(i)] = ones_row(1) * ctrls.thrust_vector_angle.initial_guess_values[i][0] 
                else:
                    segment.state.unknowns.mission["thrust_vector_angle_" + str(i)] = ones_row(1) *  0.5 
    
                if ctrls.thrust_vector_angle.bounds !=  None:
                    segment.state.unknowns_lower_bounds.mission["thrust_vector_angle_" + str(i)] = ctrls.thrust_vector_angle.bounds[i][0] * ones_row(1)
                    segment.state.unknowns_upper_bounds.mission["thrust_vector_angle_" + str(i)] = ctrls.thrust_vector_angle.bounds[i][1] * ones_row(1)
                else:
                    segment.state.unknowns_lower_bounds.mission["thrust_vector_angle_" + str(i)] =  -np.inf * ones_row(1) 
                    segment.state.unknowns_upper_bounds.mission["thrust_vector_angle_" + str(i)] =   np.inf * ones_row(1)                     

        # Blade Pitch Command 
        if ctrls.blade_pitch_command.active: 
            for i in range(len(ctrls.blade_pitch_command.assigned_rotors)): 
                segment.state.number_of_mission_unknowns  += 1 
                if ctrls.blade_pitch_command.initial_guess_values !=  None:
                    segment.state.unknowns.mission["blade_pitch_command_" + str(i)] = ones_row(1) * ctrls.blade_pitch_command.initial_guess_values[i][0] 
                else:
                    segment.state.unknowns.mission["blade_pitch_command_" + str(i)] = ones_row(1) *  0.5 
    
                if ctrls.blade_pitch_command.bounds !=  None:
                    segment.state.unknowns_lower_bounds.mission["blade_pitch_command_" + str(i)] = ctrls.blade_pitch_command.bounds[i][0] * ones_row(1)
                    segment.state.unknowns_upper_bounds.mission["blade_pitch_command_" + str(i)] = ctrls.blade_pitch_command.bounds[i][1] * ones_row(1)
                else:
                    segment.state.unknowns_lower_bounds.mission["blade_pitch_command_" + str(i)] =  -np.inf * ones_row(1) 
                    segment.state.unknowns_upper_bounds.mission["blade_pitch_command_" + str(i)] =   np.inf * ones_row(1)                      
                    
        # Velocity 
        if ctrls.velocity.active:  
            segment.state.number_of_mission_unknowns  += 1 
            if  ctrls.velocity.initial_guess_values !=  None:
                segment.state.unknowns.mission.velocity = ones_row(1) * ctrls.velocity.initial_guess_values[0][0] 
            else:
                segment.state.unknowns.mission.velocity = ones_row(1) *  100 
                 
            if ctrls.velocity.bounds !=  None:
                segment.state.unknowns_lower_bounds.mission.velocity = ctrls.velocity.bounds[0][0] * ones_row(1)
                segment.state.unknowns_upper_bounds.mission.velocity = ctrls.velocity.bounds[0][1] * ones_row(1)
            else:
                segment.state.unknowns_lower_bounds.mission.velocity =  -np.inf * ones_row(1) 
                segment.state.unknowns_upper_bounds.mission.velocity =   np.inf * ones_row(1)
                
        # Ground Velocity 
        if ctrls.ground_velocity.active:  
            segment.state.number_of_mission_unknowns  += 1 
            if  ctrls.ground_velocity.initial_guess_values !=  None:
                segment.state.unknowns.mission.ground_velocity = ones_row(1) * ctrls.ground_velocity.initial_guess_values[0][0] 
            else:
                segment.state.unknowns.mission.ground_velocity = ones_row(1) *  100 
    
            if ctrls.ground_velocity.bounds !=  None:
                segment.state.unknowns_lower_bounds.mission.ground_velocity = ctrls.ground_velocity.bounds[0][0] * ones_row_m1(1)
                segment.state.unknowns_upper_bounds.mission.ground_velocity = ctrls.ground_velocity.bounds[0][1] * ones_row_m1(1)
            else:
                segment.state.unknowns_lower_bounds.mission.ground_velocity =  -np.inf * ones_row_m1(1) 
                segment.state.unknowns_upper_bounds.mission.ground_velocity =   np.inf * ones_row_m1(1)
                        
        
        # Altitude
        if ctrls.altitude.active:  
            segment.state.number_of_mission_unknowns  += 1 
            if ctrls.altitude.initial_guess_values != None: 
                segment.state.unknowns.mission.altitude = ctrls.altitude.initial_guess_values[0][0] 
            else:
                segment.state.unknowns.mission.altitude = ones_row(1) * 0.0 
        
            if ctrls.altitude.bounds !=  None:
                segment.state.unknowns_lower_bounds.mission.altitude = ctrls.altitude.bounds[0][0] * ones_row(1) 
                segment.state.unknowns_upper_bounds.mission.altitude = ctrls.altitude.bounds[0][1] * ones_row(1)  
            else:
                segment.state.unknowns_lower_bounds.mission.altitude = -np.inf * ones_row(1)    
                segment.state.unknowns_upper_bounds.mission.altitude =  np.inf * ones_row(1) 
                        
        # Acceleration 
        if ctrls.acceleration.active:  
            segment.state.number_of_mission_unknowns  += 1 
            if ctrls.acceleration.initial_guess_values !=  None:
                segment.state.unknowns.mission.acceleration = ones_row(1) * ctrls.acceleration.initial_guess_values[0][0] 
            else:
                segment.state.unknowns.mission.acceleration = ones_row(1) *  1.
    
            if ctrls.acceleration.bounds !=  None:
                segment.state.unknowns_lower_bounds.mission.acceleration = ctrls.acceleration.bounds[0][0] * ones_row(1)
                segment.state.unknowns_upper_bounds.mission.acceleration = ctrls.acceleration.bounds[0][1] * ones_row(1)
            else:
                segment.state.unknowns_lower_bounds.mission.acceleration =  -np.inf * ones_row(1) 
                segment.state.unknowns_upper_bounds.mission.acceleration =   np.inf * ones_row(1)                 

        # Time
        if ctrls.elapsed_time.active:  
            segment.state.number_of_mission_unknowns  += 1 
            if ctrls.elapsed_time.initial_guess_values != None: 
                segment.state.unknowns.mission.elapsed_time = ctrls.elapsed_time.initial_guess_values[0][0] 
            else:
                segment.state.unknowns.mission.elapsed_time = 30 
        
            if ctrls.elapsed_time.bounds !=  None:
                segment.state.unknowns_lower_bounds.mission.elapsed_time = ctrls.elapsed_time.bounds[0][0] 
                segment.state.unknowns_upper_bounds.mission.elapsed_time = ctrls.elapsed_time.bounds[0][1]  
            else:
                segment.state.unknowns_lower_bounds.mission.elapsed_time = -np.inf    
                segment.state.unknowns_upper_bounds.mission.elapsed_time =  np.inf    
                                
        # Elevator 
        if ctrls.elevator_deflection.active:      
            segment.state.number_of_mission_unknowns  += 1 
            if ctrls.elevator_deflection.initial_guess_values!= None:  
                segment.state.unknowns.mission["elevator"] = ones_row(1) * ctrls.elevator_deflection.initial_guess_values[0][0]
            else:
                segment.state.unknowns.mission["elevator"] = ones_row(1) * 0.0 * Units.degrees

            if ctrls.elevator_deflection.bounds !=  None:
                segment.state.unknowns_lower_bounds.mission["elevator"] = ctrls.elevator_deflection.bounds[i][0] * ones_row(1)
                segment.state.unknowns_upper_bounds.mission["elevator"] = ctrls.elevator_deflection.bounds[i][1] * ones_row(1)
            else:
                segment.state.unknowns_lower_bounds.mission["elevator"] =  -np.inf * ones_row(1) 
                segment.state.unknowns_upper_bounds.mission["elevator"] =   np.inf * ones_row(1)                   
                
        # Rudder
        if ctrls.rudder_deflection.active:    
            segment.state.number_of_mission_unknowns  += 1 
            if ctrls.rudder_deflection.initial_guess_values !=  None: 
                segment.state.unknowns.mission["rudder"] = ones_row(1) * ctrls.rudder_deflection.initial_guess_values[0][0]
            else:
                segment.state.unknowns.mission["rudder"] = ones_row(1) * 0.0 * Units.degrees
                
    
            if ctrls.rudder_deflection.bounds !=  None:
                segment.state.unknowns_lower_bounds.mission["rudder"] = ctrls.rudder_deflection.bounds[i][0] * ones_row(1)
                segment.state.unknowns_upper_bounds.mission["rudder"] = ctrls.rudder_deflection.bounds[i][1] * ones_row(1)
            else:
                segment.state.unknowns_lower_bounds.mission["rudder"] =  -np.inf * ones_row(1) 
                segment.state.unknowns_upper_bounds.mission["rudder"] =   np.inf * ones_row(1)                   
                  
        # Aileron  
        if ctrls.aileron_deflection.active:   
            segment.state.number_of_mission_unknowns  += 1 
            if ctrls.aileron_deflection.initial_guess_values !=  None:
                segment.state.unknowns.mission["aileron" ] = ones_row(1) * ctrls.aileron_deflection.initial_guess_values[0][0]
            else: 
                segment.state.unknowns.mission["aileron" ] = ones_row(1) * 0.0 * Units.degrees 
        
            if ctrls.aileron_deflection.bounds !=  None:
                segment.state.unknowns_lower_bounds.mission["aileron"] = ctrls.aileron_deflection.bounds[i][0] * ones_row(1)
                segment.state.unknowns_upper_bounds.mission["aileron"] = ctrls.aileron_deflection.bounds[i][1] * ones_row(1)
            else:
                segment.state.unknowns_lower_bounds.mission["aileron"] =  -np.inf * ones_row(1) 
                segment.state.unknowns_upper_bounds.mission["aileron"] =   np.inf * ones_row(1)                 
    return 
                                                                                                                                                                