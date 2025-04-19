# RCAIDE/Methods/Energy/Propulsors/Rotor_Design/set_optimized_parameters.py
# 
# 
# Created:  Jul 2023, M. Clarke 
    
# ----------------------------------------------------------------------------------------------------------------------  
#  Set Optimized rotor platform paramters
# ----------------------------------------------------------------------------------------------------------------------   
def set_optimized_parameters(rotor,optimization_problem):
    """ Append parameters of optimized prop-rotor to input prop-rotor
          
          Inputs:  
             rotor                - rotor data structure                        [None]
             optimization_problem      - data struction of optimized parameters [None]
             
          Outputs: 
             rotor                - rotor data structure                        [None]
              
          Assumptions: 
             1) Default noise measurements are taken 135 degrees from prop-rotor plane 
        
          Source:
             None
    """    
    results                         = optimization_problem.results 
    optimal_hover_rotor             = optimization_problem.vehicle_configurations.hover.networks.electric.propulsors.electric_rotor.rotor     
    optimal_oei_rotor               = optimization_problem.vehicle_configurations.oei.networks.electric.propulsors.electric_rotor.rotor    
    rotor.chord_distribution        = optimal_hover_rotor.chord_distribution
    rotor.twist_distribution        = optimal_hover_rotor.twist_distribution   
    
    if rotor.hover.design_power == None: 
        rotor.hover.design_power = results.hover.power 
    
    if rotor.hover.design_thrust == None: 
        rotor.hover.design_thrust = results.hover.thrust   
        
    rotor.hover.design_torque              = results.hover.torque  
    rotor.hover.design_angular_velocity    = results.hover.omega  
    rotor.hover.design_Cl                  = results.hover.mean_CL 
    rotor.hover.design_thrust_coefficient  = results.hover.thurst_c
    rotor.hover.design_power_coefficient   = results.hover.power_c 
    rotor.hover.design_SPL_dBA             = results.hover.mean_SPL 
    rotor.hover.design_pitch_command       = optimal_hover_rotor.hover.design_pitch_command
    rotor.oei.design_thrust                = results.oei.thrust    
    rotor.oei.design_power                 = results.oei.power 
    rotor.oei.design_torque                = results.oei.torque  
    rotor.oei.design_angular_velocity      = results.oei.omega 
    rotor.oei.design_pitch_command         = optimal_oei_rotor.oei.design_pitch_command
    
    if optimization_problem.prop_rotor_flag:  
        optimal_cruise_rotor  = optimization_problem.vehicle_configurations.cruise.networks.electric.propulsors.electric_rotor.rotor      
        if rotor.cruise.design_power == None: 
            rotor.cruise.design_power = results.cruise.power 
        
        if rotor.cruise.design_thrust == None: 
            rotor.cruise.design_thrust = results.cruise.thrust            
     
        rotor.cruise.design_torque              = results.cruise.torque  
        rotor.cruise.design_angular_velocity    = results.cruise.omega  
        rotor.cruise.design_Cl                  = results.cruise.mean_CL 
        rotor.cruise.design_thrust_coefficient  = results.cruise.thurst_c
        rotor.cruise.design_power_coefficient   = results.cruise.power_c   
        rotor.cruise.design_SPL_dBA             = results.cruise.mean_SPL
        rotor.cruise.design_pitch_command       = optimal_cruise_rotor.cruise.design_pitch_command
             
    rotor.max_thickness_distribution        = optimal_hover_rotor.max_thickness_distribution  
    rotor.radius_distribution               = optimal_hover_rotor.radius_distribution         
    rotor.number_of_blades                  = optimal_hover_rotor.number_of_blades               
    rotor.mid_chord_alignment               = optimal_hover_rotor.mid_chord_alignment         
    rotor.thickness_to_chord                = optimal_hover_rotor.max_thickness_distribution / optimal_hover_rotor.chord_distribution
    rotor.blade_solidity                    = optimal_hover_rotor.blade_solidity   
    
    return rotor 