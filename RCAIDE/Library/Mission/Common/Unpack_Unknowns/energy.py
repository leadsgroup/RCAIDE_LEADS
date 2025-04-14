# RCAIDE/Library/Missions/Common/Unpack_Unknowns/energy.py
# 
# 
# Created:  Jul 2023, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  Unpack Unknowns
# ---------------------------------------------------------------------------------------------------------------------- 
def unknowns(segment):  
    ACV_T      =  segment.assigned_control_variables.throttle
    ACV_TA     =  segment.assigned_control_variables.thrust_vector_angle
    ACV_RBPC   =  segment.assigned_control_variables.rotor_blade_pitch_command
    
    for network in segment.analyses.energy.vehicle.networks: 
        if 'throttle' in segment: 
            for propulsor in network.propulsors: 
                segment.state.conditions.energy.propulsors[propulsor.tag].throttle[:,0] = segment.throttle
            
        if ACV_T.active: 
            for i in range(len(ACV_T.assigned_propulsors)): 
                propulsor_group = ACV_T.assigned_propulsors[i]
                for propulsor_name in propulsor_group:  
                    segment.state.conditions.energy.propulsors[propulsor_name].throttle = segment.state.unknowns["throttle_" + str(i)]  
    
       ## Thrust Vector Control 
        #if ACV_TA.active:                
            #for i in range(len(ACV_TA.assigned_propulsors)): 
                #propulsor_group = ACV_TA.assigned_propulsors[i]
                #for propulsor_name in propulsor_group:  
                    #segment.state.conditions.energy.propulsors[propulsor_name].commanded_thrust_vector_angle = segment.state.unknowns["thrust_vector_" + str(i)]
                    
             
        ## Thrust Vector Control 
        #if ACV_RBPC.active:                
            #for i in range(len(ACV_RBPC.assigned_propulsors)): 
                #propulsor_group = ACV_RBPC.assigned_propulsors[i] 
                #for propulsor_name in propulsor_group:
                    #propulsor = network.propulsors[propulsor_name]
                    #rotor     = propulsor.rotor 
                    #segment.state.conditions.energy.propulsors[propulsor_name][rotor.tag].blade_pitch_command = segment.state.unknowns["blade_pitch_command_" + str(i)]                    
    return 
     
 
    
