# RCAIDE/Methods/Energy/Propulsors/Rotor_Design/set_optimized_parameters.py
# 
# 
# Created:  Jul 2023, M. Clarke 
    
# ----------------------------------------------------------------------------------------------------------------------  
#  Set Optimized rotor platform paramters
# ----------------------------------------------------------------------------------------------------------------------   
def set_optimized_parameters(rotor, optimization_problem):
    """
    Appends parameters of optimized rotor to input rotor data structure.
    
    Parameters
    ----------
    rotor : RCAIDE.Library.Components.Powertrain.Converters.Rotor
        Rotor component to be updated with optimized parameters
    optimization_problem : RCAIDE.Framework.Optimization.Common.Nexus
        Optimization problem with results and optimized rotor configurations
    
    Returns
    -------
    rotor : RCAIDE.Library.Components.Powertrain.Converters.Rotor
        Updated rotor with optimized parameters
    
    Notes
    -----
    This function transfers the optimized rotor parameters from the optimization results
    to the input rotor data structure. It copies geometric properties such as chord and
    twist distributions, as well as performance metrics for different flight conditions.
    
    The function updates the following parameters:
        - Geometric properties:
            - chord_distribution
            - twist_distribution
            - max_thickness_distribution
            - radius_distribution
            - number_of_blades
            - mid_chord_alignment
            - thickness_to_chord
            - blade_solidity
        
        - Hover performance metrics:
            - design_power (if not specified)
            - design_thrust (if not specified)
            - design_torque
            - design_angular_velocity
            - design_Cl
            - design_thrust_coefficient
            - design_power_coefficient
            - design_SPL_dBA
            - design_blade_pitch_command
        
        - OEI (One Engine Inoperative) performance metrics:
            - design_thrust
            - design_power
            - design_torque
            - design_angular_velocity
            - design_blade_pitch_command
        
        - Cruise performance metrics (for prop-rotors only):
            - design_power (if not specified)
            - design_thrust (if not specified)
            - design_torque
            - design_angular_velocity
            - design_Cl
            - design_thrust_coefficient
            - design_power_coefficient
            - design_SPL_dBA
            - design_blade_pitch_command
    
    **Major Assumptions**
        * Default noise measurements are taken 135 degrees from rotor plane
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Design.optimization_setup
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Design.procedure_setup
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
    rotor.hover.design_blade_pitch_command = optimal_hover_rotor.hover.design_blade_pitch_command
    rotor.oei.design_thrust                = results.oei.thrust    
    rotor.oei.design_power                 = results.oei.power 
    rotor.oei.design_torque                = results.oei.torque  
    rotor.oei.design_angular_velocity      = results.oei.omega 
    rotor.oei.design_blade_pitch_command   = optimal_oei_rotor.oei.design_blade_pitch_command
    
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
        rotor.cruise.design_blade_pitch_command = optimal_cruise_rotor.cruise.design_blade_pitch_command
             
    rotor.max_thickness_distribution        = optimal_hover_rotor.max_thickness_distribution  
    rotor.radius_distribution               = optimal_hover_rotor.radius_distribution         
    rotor.number_of_blades                  = optimal_hover_rotor.number_of_blades               
    rotor.mid_chord_alignment               = optimal_hover_rotor.mid_chord_alignment         
    rotor.thickness_to_chord                = optimal_hover_rotor.max_thickness_distribution / optimal_hover_rotor.chord_distribution
    rotor.blade_solidity                    = optimal_hover_rotor.blade_solidity   
    
    return rotor 