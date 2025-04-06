# RCAIDE/Methods/Energy/Propulsors/Rotor_Design/optimization_setup.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
 
# RCAIDE Imports  
import RCAIDE 
from RCAIDE.Framework.Core                                                                    import Units, Data   
from RCAIDE.Framework.Optimization.Common                                                     import Nexus       
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Design.blade_geometry_setup    import blade_geometry_setup
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Design.procedure_setup         import procedure_setup

# Python package imports   
import numpy as np  
    
# ----------------------------------------------------------------------------------------------------------------------  
#  Optimization Setuo 
# ----------------------------------------------------------------------------------------------------------------------    
def optimization_setup(rotor, number_of_stations, print_iterations):
    """
    Sets up rotor optimization problem including design variables, constraints and objective function
    using RCAIDE's Nexus optimization framework.
    
    Parameters
    ----------
    rotor : RCAIDE.Library.Components.Powertrain.Converters.Rotor
        Rotor component (Lift_Rotor or Prop_Rotor) with optimization parameters
    number_of_stations : int
        Number of radial stations for blade discretization
    print_iterations : bool
        Flag to print optimization iterations
    
    Returns
    -------
    nexus : RCAIDE.Framework.Optimization.Common.Nexus
        RCAIDE's optimization framework object with the following attributes:
            - optimization_problem : Data
                Optimization problem definition
                - inputs : numpy.ndarray
                    Design variables
                - objective : numpy.ndarray
                    Objective function
                - constraints : numpy.ndarray
                    Constraints
            - vehicle_configurations : list
                List of vehicle configurations for analysis
            - procedure : Data
                Optimization procedure
            - print_iterations : bool
                Flag to print optimization iterations
    
    Notes
    -----
    This function configures a complete optimization problem for rotor blade design by:
        1. Creating a Nexus optimization framework
        2. Validating the rotor type (must be Lift_Rotor or Prop_Rotor)
        3. Setting up design variables with bounds and scaling
        4. Defining the objective function
        5. Establishing constraints for performance and geometry
        6. Creating aliases to link optimization variables to vehicle properties
        7. Setting up vehicle configurations using blade_geometry_setup
        8. Configuring the optimization procedure
    
    The design variables include:
        - chord_r, chord_p, chord_q, chord_t: Parameters defining the chord distribution
        - twist_r, twist_p, twist_q, twist_t: Parameters defining the twist distribution
        - hover_tip_mach: Tip Mach number in hover
        - OEI_tip_mach: Tip Mach number in one-engine-inoperative condition
        - OEI_collective_pitch: Collective pitch in one-engine-inoperative condition
        - cruise_tip_mach, cruise_collective_pitch: Additional parameters for prop rotors
    
    Constraints ensure:
        - Thrust and power requirements are met
        - Blade taper is within reasonable bounds (0.3 to 0.9)
        - Blade twist is positive
        - Maximum sectional lift coefficient is below 0.8
        - Chord and twist distribution parameters maintain reasonable ratios
    
    **Major Assumptions**
        * Minimum allowable blade taper: 0.3
        * Maximum allowable blade taper: 0.9
        * Maximum sectional lift coefficient: 0.8
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Design.blade_geometry_setup
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Design.procedure_setup
    """    
    nexus                        = Nexus()
    problem                      = Data()
    nexus.optimization_problem   = problem
   
    if type(rotor) != RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor or  type(rotor) != RCAIDE.Library.Components.Powertrain.Converters.Lift_Rotor:
        assert('rotor must be of Lift-Rotor or Prop-Rotor class') 
        
    if type(rotor) == RCAIDE.Library.Components.Powertrain.Converters.Prop_Rotor:
        nexus.prop_rotor_flag = True 
    else:
        nexus.prop_rotor_flag = False 
    # -------------------------------------------------------------------
    # Inputs
    # -------------------------------------------------------------------  
    R         = rotor.tip_radius  
    tm_ll_h   = rotor.optimization_parameters.tip_mach_range[0]
    tm_ul_h   = rotor.optimization_parameters.tip_mach_range[1] 
    tm_0_h    = (tm_ul_h + tm_ll_h)/2 
    
    if nexus.prop_rotor_flag:    
        tm_ll_c          = rotor.optimization_parameters.tip_mach_range[0]
        tm_ul_c          = rotor.optimization_parameters.tip_mach_range[1]    
        
    inputs = []
    inputs.append([ 'chord_r'               ,  0.1*R    , 0.05*R     , 0.2*R     , 1.0     ,  1*Units.less])
    inputs.append([ 'chord_p'               ,  2        , 0.25       , 2.0       , 1.0     ,  1*Units.less])
    inputs.append([ 'chord_q'               ,  1        , 0.25       , 1.5       , 1.0     ,  1*Units.less])
    inputs.append([ 'chord_t'               ,  0.05*R   , 0.02*R     , 0.1*R     , 1.0     ,  1*Units.less])  
    inputs.append([ 'twist_r'               ,  np.pi/6  ,  0         , np.pi/4   , 1.0     ,  1*Units.less])
    inputs.append([ 'twist_p'               ,  1        , 0.25       , 2.0       , 1.0     ,  1*Units.less])
    inputs.append([ 'twist_q'               ,  0.5      , 0.25       , 1.5       , 1.0     ,  1*Units.less])
    inputs.append([ 'twist_t'               ,  np.pi/6  , 0          , np.pi/4   , 1.0     ,  1*Units.less])  
    inputs.append([ 'hover_tip_mach'        , tm_0_h    , tm_ll_h    , tm_ul_h   , 1.0     ,  1*Units.less])
    inputs.append([ 'OEI_tip_mach'          , tm_0_h    , tm_ll_h    , 0.85      , 1.0     ,  1*Units.less])
    inputs.append([ 'OEI_collective_pitch'  , np.pi/6   , -np.pi/5   , np.pi/5   , 1.0      ,  1*Units.less])
    if nexus.prop_rotor_flag: 
        inputs.append([ 'cruise_tip_mach'         , tm_ll_c , tm_ll_c    , tm_ul_c  , 1.0     ,  1*Units.less]) 
        inputs.append([ 'cuise_collective_pitch'  , np.pi/8 , -np.pi/5   , np.pi/5  , 1.0     ,  1*Units.less]) 
    problem.inputs = np.array(inputs,dtype=object)   

    # -------------------------------------------------------------------
    # Objective
    # ------------------------------------------------------------------- 
    problem.objective = np.array([  
                                 [  'objective'  ,  1.0   ,    1*Units.less] 
    ],dtype=object)
    
    # -------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------  
    constraints = [] 
    constraints.append([ 'hover_thrust_pow_res'      ,  '<'  ,  1E-3 ,   1.0   , 1*Units.less])  
    constraints.append([ 'blade_taper_constraint_1'  ,  '>'  ,  0.3 ,   1.0   , 1*Units.less])  
    constraints.append([ 'blade_taper_constraint_2'  ,  '<'  ,  0.9 ,   1.0   , 1*Units.less])  
    constraints.append([ 'blade_twist_constraint'    ,  '>'  ,  0.0 ,   1.0   , 1*Units.less])
    constraints.append([ 'max_sectional_cl_hov'      ,  '<'  ,  0.8 ,   1.0   , 1*Units.less])
    constraints.append([ 'chord_p_to_q_ratio'        ,  '>'  ,  0.5 ,   1.0   , 1*Units.less])    
    constraints.append([ 'twist_p_to_q_ratio'        ,  '>'  ,  0.5 ,   1.0   , 1*Units.less])  
    constraints.append([ 'OEI_hov_thrust_pow_res'    ,  '<'  ,  1E-3 ,   1.0   , 1*Units.less]) 
    if nexus.prop_rotor_flag:
        constraints.append([ 'cruise_thrust_pow_res'     ,  '<'  ,  1E-3 ,   1.0   , 1*Units.less]) 
        constraints.append([ 'max_sectional_cl_cruise'   ,  '<'  ,  0.8 ,   1.0   , 1*Units.less])   
    problem.constraints =  np.array(constraints,dtype=object)                
    
    # -------------------------------------------------------------------
    #  Aliases
    # ------------------------------------------------------------------- 
    aliases = [] 
    aliases.append([ 'chord_r'                    , 'vehicle_configurations.*.networks.electric.propulsors.electric_rotor.rotor.chord_r' ])
    aliases.append([ 'chord_p'                    , 'vehicle_configurations.*.networks.electric.propulsors.electric_rotor.rotor.chord_p' ])
    aliases.append([ 'chord_q'                    , 'vehicle_configurations.*.networks.electric.propulsors.electric_rotor.rotor.chord_q' ])
    aliases.append([ 'chord_t'                    , 'vehicle_configurations.*.networks.electric.propulsors.electric_rotor.rotor.chord_t' ]) 
    aliases.append([ 'twist_r'                    , 'vehicle_configurations.*.networks.electric.propulsors.electric_rotor.rotor.twist_r' ])
    aliases.append([ 'twist_p'                    , 'vehicle_configurations.*.networks.electric.propulsors.electric_rotor.rotor.twist_p' ])
    aliases.append([ 'twist_q'                    , 'vehicle_configurations.*.networks.electric.propulsors.electric_rotor.rotor.twist_q' ])
    aliases.append([ 'twist_t'                    , 'vehicle_configurations.*.networks.electric.propulsors.electric_rotor.rotor.twist_t' ])     
    aliases.append([ 'hover_tip_mach'             , 'vehicle_configurations.hover.networks.electric.propulsors.electric_rotor.rotor.hover.design_tip_mach' ]) 
    aliases.append([ 'objective'                  , 'summary.objective'       ])  
    aliases.append([ 'hover_thrust_pow_res'       , 'summary.hover_thrust_power_residual'   ]) 
    aliases.append([ 'blade_taper_constraint_1'   , 'summary.blade_taper_constraint_1'])   
    aliases.append([ 'blade_taper_constraint_2'   , 'summary.blade_taper_constraint_2'])   
    aliases.append([ 'blade_twist_constraint'     , 'summary.blade_twist_constraint'])    
    aliases.append([ 'max_sectional_cl_hov'       , 'summary.max_sectional_cl_hover'])   
    aliases.append([ 'chord_p_to_q_ratio'         , 'summary.chord_p_to_q_ratio'    ])  
    aliases.append([ 'twist_p_to_q_ratio'         , 'summary.twist_p_to_q_ratio'    ])   
    aliases.append([ 'OEI_hov_thrust_pow_res'     , 'summary.oei_thrust_power_residual'   ]) 
    aliases.append([ 'OEI_collective_pitch'       , 'vehicle_configurations.oei.networks.electric.propulsors.electric_rotor.rotor.oei.design_blade_pitch_command' ]) 
    aliases.append([ 'OEI_tip_mach'               , 'vehicle_configurations.oei.networks.electric.propulsors.electric_rotor.rotor.oei.design_tip_mach' ]) 
    if nexus.prop_rotor_flag: 
        aliases.append([ 'cruise_tip_mach'        , 'vehicle_configurations.cruise.networks.electric.propulsors.electric_rotor.rotor.cruise.design_tip_mach' ])  
        aliases.append([ 'cuise_collective_pitch' , 'vehicle_configurations.cruise.networks.electric.propulsors.electric_rotor.rotor.cruise.design_blade_pitch_command' ])  
        aliases.append([ 'cruise_thrust_pow_res'  , 'summary.cruise_thrust_power_residual'   ]) 
        aliases.append([ 'max_sectional_cl_cruise', 'summary.max_sectional_cl_cruise'])  
         
    problem.aliases = aliases
    
    # -------------------------------------------------------------------
    #  Vehicles
    # -------------------------------------------------------------------
    nexus.vehicle_configurations = blade_geometry_setup(rotor,number_of_stations)
    
    # -------------------------------------------------------------------
    #  Analyses
    # -------------------------------------------------------------------
    nexus.analyses = None 
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.missions = None
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.print_iterations  = print_iterations 
    nexus.procedure         = procedure_setup()
    
    # -------------------------------------------------------------------
    #  Summary
    # -------------------------------------------------------------------    
    nexus.summary        = Data()     
    nexus.results.hover  = Data() 
    nexus.results.cruise = Data()
    nexus.results.oei    = Data()
    
    return nexus   