# RCAIDE/Methods/Energy/Propulsors/Rotor_Design/procedure_setup.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  

# RCAIDE Imports 
import RCAIDE 
from RCAIDE.Framework.Core                                                        import Units  
from RCAIDE.Library.Methods.Noise.Frequency_Domain_Buildup.Rotor                  import compute_rotor_noise 
from RCAIDE.Framework.Analyses.Process                                            import Process   
from RCAIDE.Framework.Mission.Common                                              import Conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.compute_rotor_performance import compute_rotor_performance 

# Python package imports   
import numpy as np 
import scipy as sp  
    
# ----------------------------------------------------------------------------------------------------------------------  
#  Procedure Setup 
# ----------------------------------------------------------------------------------------------------------------------    
def procedure_setup(): 
    
    # size the base config
    procedure = Process()
    
    # mofify blade geometry 
    procedure.modify_rotor = modify_blade_geometry
    
    # Run the rotor in hover
    procedure.hover       = run_rotor_hover
    
    # Run the rotor in the oei condition
    procedure.oei         = run_rotor_OEI 

     # Run the rotor in cruise
    procedure.ruise       = run_rotor_cruise

    # post process the results
    procedure.post_process = post_process
        
    return procedure


# ----------------------------------------------------------------------
# Update blade geometry 
# ---------------------------------------------------------------------- 
def modify_blade_geometry(nexus): 
    """ Modifies geometry of prop-rotor blade 
          
          Inputs:  
             nexus     - RCAIDE optmization framework with prop-rotor blade data structure [None]
              
          Outputs:   
             procedure - optimization methodology                                         [None]
              
          Assumptions: 
             N/A 
        
          Source:
             None
    """        
 
    # Pull out the vehicles

    vehicle_hover     = nexus.vehicle_configurations.hover
    rotor_hover       = vehicle_hover.networks.electric.propulsors.electric_rotor.rotor
    vehicle_oei       = nexus.vehicle_configurations.oei
    rotor_oei         = vehicle_oei.networks.electric.propulsors.electric_rotor.rotor    
    
    if nexus.prop_rotor_flag:  
        vehicle_cruise    = nexus.vehicle_configurations.cruise 
        rotor_cruise      = vehicle_cruise.networks.electric.propulsors.electric_rotor.rotor 
        
    airfoils = rotor_hover.airfoils      
    a_loc    = rotor_hover.airfoil_polar_stations   
        
    # Update geometry of blade
    R       = rotor_hover.tip_radius     
    B       = rotor_hover.number_of_blades 
    r       = rotor_hover.radius_distribution
    c       = updated_blade_geometry(rotor_hover.radius_distribution/rotor_hover.tip_radius ,rotor_hover.chord_r,rotor_hover.chord_p,rotor_hover.chord_q,rotor_hover.chord_t)     
    beta    = updated_blade_geometry(rotor_hover.radius_distribution/rotor_hover.tip_radius ,rotor_hover.twist_r,rotor_hover.twist_p,rotor_hover.twist_q,rotor_hover.twist_t)   
    
    # compute max thickness distribution   
    blade_area    = sp.integrate.cumulative_trapezoid(B*c, r-r[0])
    sigma         = blade_area[-1]/(np.pi*R**2)      
    t_max         = np.zeros(len(c))    
    t_c           = np.zeros(len(c))       
    t_max  = np.zeros(len(c))     
    if len(airfoils.keys())>0:
        for j,airfoil in enumerate(airfoils): 
            a_geo         = airfoil.geometry
            locs          = np.where(np.array(a_loc) == j )
            t_max[locs]   = a_geo.max_thickness*c[locs]   
     
    rotor_hover.chord_distribution          = c
    rotor_hover.twist_distribution          = beta  
    rotor_hover.mid_chord_alignment         = c/4. - c[0]/4.
    rotor_hover.max_thickness_distribution  = t_max 
    rotor_hover.thickness_to_chord          = t_c
    rotor_hover.blade_solidity              = sigma     
    vehicle_hover.store_diff() 
      

    rotor_oei.chord_distribution         = rotor_hover.chord_distribution
    rotor_oei.twist_distribution         = rotor_hover.twist_distribution
    rotor_oei.mid_chord_alignment        = rotor_hover.mid_chord_alignment  
    rotor_oei.max_thickness_distribution = rotor_hover.max_thickness_distribution
    rotor_oei.thickness_to_chord         = rotor_hover.thickness_to_chord 
    rotor_oei.blade_solidity             = rotor_hover.blade_solidity    
    vehicle_oei.store_diff()     
     
    if nexus.prop_rotor_flag: 
        rotor_cruise.chord_distribution         = rotor_hover.chord_distribution
        rotor_cruise.twist_distribution         = rotor_hover.twist_distribution
        rotor_cruise.mid_chord_alignment        = rotor_hover.mid_chord_alignment  
        rotor_cruise.max_thickness_distribution = rotor_hover.max_thickness_distribution
        rotor_cruise.thickness_to_chord         = rotor_hover.thickness_to_chord 
        rotor_cruise.blade_solidity             = rotor_hover.blade_solidity     
        vehicle_cruise.store_diff()  
    
    return nexus    


# ----------------------------------------------------------------------
#   Update blade geometry 
# ---------------------------------------------------------------------- 
def updated_blade_geometry(chi,c_r,p,q,c_t):
    """ Computes planform function of twist and chord distributron using hyperparameters  
          
          Inputs:  
             chi - prop-rotor radius distribution [None]
             c_r - hyperparameter no. 1           [None]
             p   - hyperparameter no. 2           [None]
             q   - hyperparameter no. 3           [None]
             c_t - hyperparameter no. 4           [None] 
                   
          Outputs:       
             x_lin  - function distribution       [None]
              
          Assumptions: 
             N/A 
        
          Source:
              Traub, Lance W., et al. "Effect of taper ratio at low reynolds number."
              Journal of Aircraft 52.3 (2015): 734-747.
              
    """           

    n       = np.linspace(len(chi)-1,0,len(chi))          
    theta_n = n*(np.pi/2)/len(chi)              
    y_n     = chi[-1]*np.cos(theta_n)          
    eta_n   = np.abs(y_n/chi[-1])            
    x_cos   = c_r*(1 - eta_n**p)**q + c_t*eta_n  
    x_lin   = np.interp(chi,eta_n, x_cos)  
    return x_lin 


# ----------------------------------------------------------------------
#   Run the Rotor Hover
# ----------------------------------------------------------------------  
def run_rotor_hover(nexus):
     
    # Unpack    
    network               = nexus.vehicle_configurations.hover.networks.electric 
    bus                   = network.busses.bus 
    electric_rotor        = network.propulsors.electric_rotor
    rotor                 = electric_rotor.rotor
    
    # Setup Test conditions
    alpha                   = rotor.optimization_parameters.multiobjective_aeroacoustic_weight 
    speed                   = rotor.hover.design_freestream_velocity 
    altitude                = np.array([rotor.hover.design_altitude])  
    atmosphere              = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere_conditions   = atmosphere.compute_values(altitude)  
  
    segment                                             = RCAIDE.Framework.Mission.Segments.Segment()  
    conditions                                          = RCAIDE.Framework.Mission.Common.Results()
    conditions.freestream.update(atmosphere_conditions)  
    conditions.frames.inertial.velocity_vector          = np.array([[0.,0.,speed]])   
    conditions.frames.body.transform_to_inertial        = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., -1.]]]) 
    conditions.frames.wind.transform_to_inertial        = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0.,  1.]]]) 
    conditions.frames.planet.true_course                = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0.,  1.]]])  
    segment.state.conditions                            = conditions
    
    segment.state.conditions.energy[bus.tag] = Conditions()
    segment.state.conditions.noise[bus.tag]  = Conditions() 
    electric_rotor.append_operating_conditions(segment) 
    for tag, item in  electric_rotor.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,electric_rotor)
    
    rotor_conditions                      =  segment.state.conditions.energy[electric_rotor.tag][rotor.tag]     
    rotor_conditions.omega                = (atmosphere_conditions.speed_of_sound*rotor.hover.design_tip_mach)/rotor.tip_radius
    rotor_conditions.pitch_command[:,0]   = rotor.hover.design_pitch_command
    
    compute_rotor_performance(electric_rotor,segment.state)   
     
    # Pack the results
    nexus.results.hover.thrust                       = -segment.state.conditions.energy[electric_rotor.tag][rotor.tag].thrust[0,2]  
    nexus.results.hover.torque                       = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].torque[0][0]
    nexus.results.hover.power                        = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].power[0][0]
    nexus.results.hover.power_c                      = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].power_coefficient[0][0]
    nexus.results.hover.thurst_c                     = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].thrust_coefficient[0][0]
    nexus.results.hover.omega                        = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].omega[0][0]
    nexus.results.hover.max_sectional_cl             = np.max(segment.state.conditions.energy[electric_rotor.tag][rotor.tag].lift_coefficient[0]) 
    nexus.results.hover.mean_CL                      = np.mean(segment.state.conditions.energy[electric_rotor.tag][rotor.tag].lift_coefficient[0]) 
    nexus.results.hover.figure_of_merit              = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].figure_of_merit[0][0]  
    nexus.results.hover.efficiency                   = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].efficiency[0][0] 
    nexus.results.hover.conditions                   = conditions  
                
    # microphone locations             
    ctrl_pts                                         = 1 
    theta                                            = rotor.optimization_parameters.noise_evaluation_angle 
    S_hover                                          = np.maximum(altitude[0],20*Units.feet)  
    mic_positions_hover                              = np.array([[0.0 , S_hover*np.sin(theta)  ,S_hover*np.cos(theta)]])      
    
    # Run noise model  
    conditions.noise[bus.tag]                        = RCAIDE.Framework.Mission.Common.Conditions()      
    conditions.noise[electric_rotor.tag]             = RCAIDE.Framework.Mission.Common.Conditions()  
    conditions.noise.relative_microphone_locations   = np.repeat(mic_positions_hover[ np.newaxis,:,: ],1,axis=0)
    conditions.aerodynamics.angles.alpha             = np.ones((ctrl_pts,1))* 0. * Units.degrees 
    segment                                          = RCAIDE.Framework.Mission.Segments.Segment() 
    segment.state.conditions                         = conditions
    segment.state.conditions.expand_rows(ctrl_pts)  
    noise                                            = RCAIDE.Framework.Analyses.Noise.Frequency_Domain_Buildup() 
    settings                                         = noise.settings   
    num_mic                                          = len(conditions.noise.relative_microphone_locations[0])  
    conditions.noise.number_of_microphones           = num_mic   
    
    if alpha != 1: 
        compute_rotor_noise(bus,electric_rotor,segment,settings)    
        nexus.results.hover.mean_SPL   = np.mean(conditions.noise[electric_rotor.tag][rotor.tag].SPL_dBA) 
    else: 
        nexus.results.hover.mean_SPL   = 0  

    return nexus

# ----------------------------------------------------------------------
#   One Engine Inoperative 
# ---------------------------------------------------------------------- 
def run_rotor_OEI(nexus):
    
    # Unpack   
    network               = nexus.vehicle_configurations.oei.networks.electric 
    bus                   = network.busses.bus 
    electric_rotor        = network.propulsors.electric_rotor    
    rotor                 = electric_rotor.rotor
    
    # Setup Test conditions
    speed                 = rotor.oei.design_freestream_velocity 
    altitude              = np.array([rotor.oei.design_altitude]) 
    atmosphere            = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere_conditions = atmosphere.compute_values(altitude)  
  
  
    segment                                             = RCAIDE.Framework.Mission.Segments.Segment()  
    conditions                                          = RCAIDE.Framework.Mission.Common.Results()
    conditions.freestream.update(atmosphere_conditions)  
    conditions.frames.inertial.velocity_vector          = np.array([[0.,0.,speed]])   
    conditions.frames.body.transform_to_inertial        = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., -1.]]]) 
    conditions.frames.wind.transform_to_inertial        = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0.,  1.]]]) 
    conditions.frames.planet.true_course                = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0.,  1.]]]) 
    segment.state.conditions                            = conditions
    
    segment.state.conditions.energy[bus.tag] = Conditions()
    segment.state.conditions.noise[bus.tag]  = Conditions()
    electric_rotor.append_operating_conditions(segment) 
    for tag, item in  electric_rotor.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,electric_rotor) 
                
    rotor_conditions                      =  segment.state.conditions.energy[electric_rotor.tag][rotor.tag]     
    rotor_conditions.omega                = (atmosphere_conditions.speed_of_sound*rotor.oei.design_tip_mach)/rotor.tip_radius
    rotor_conditions.pitch_command[:,0]   = rotor.oei.design_pitch_command
    
    compute_rotor_performance(electric_rotor,segment.state)   
            
    # Pack the results
    nexus.results.oei.thrust       = -segment.state.conditions.energy[electric_rotor.tag][rotor.tag].thrust[0,2]  
    nexus.results.oei.torque       = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].torque[0][0]
    nexus.results.oei.power        = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].power[0][0]
    nexus.results.oei.power_c      = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].power_coefficient[0][0]
    nexus.results.oei.omega        = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].omega[0][0]
    nexus.results.oei.thurst_c     = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].thrust_coefficient[0][0]
    nexus.results.oei.efficiency   = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].efficiency[0][0]
    nexus.results.oei.conditions   = conditions 
    
    return nexus

# ----------------------------------------------------------------------
#   Run the Rotor Cruise
# ----------------------------------------------------------------------  
def run_rotor_cruise(nexus):
 
    if nexus.prop_rotor_flag:     
        network         = nexus.vehicle_configurations.cruise.networks.electric 
        bus             = network.busses.bus 
        electric_rotor  = network.propulsors.electric_rotor
        rotor           = electric_rotor.rotor 
        alpha           = rotor.optimization_parameters.multiobjective_aeroacoustic_weight       
        
        # Setup Test conditions
        speed                 = rotor.cruise.design_freestream_velocity 
        altitude              = np.array([rotor.cruise.design_altitude])  
        atmosphere            = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
        atmosphere_conditions = atmosphere.compute_values(altitude) 
      
        segment                                             = RCAIDE.Framework.Mission.Segments.Segment()  
        conditions                                          = RCAIDE.Framework.Mission.Common.Results()
        conditions.freestream.update(atmosphere_conditions)  
        conditions.frames.inertial.velocity_vector          = np.array([[0.,0.,speed]])   
        conditions.frames.body.transform_to_inertial        = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0., -1.]]]) 
        conditions.frames.wind.transform_to_inertial        = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0.,  1.]]]) 
        conditions.frames.planet.true_course                = np.array([[[1., 0., 0.],[0., 1., 0.],[0., 0.,  1.]]]) 
        segment.state.conditions                            = conditions
        
        segment.state.conditions.energy[bus.tag] = Conditions()
        segment.state.conditions.noise[bus.tag]  = Conditions()
        electric_rotor.append_operating_conditions(segment) 
        for tag, item in  electric_rotor.items(): 
            if issubclass(type(item), RCAIDE.Library.Components.Component):
                item.append_operating_conditions(segment,electric_rotor) 
            
        rotor_conditions                      =  segment.state.conditions.energy[electric_rotor.tag][rotor.tag]     
        rotor_conditions.omega                = (atmosphere_conditions.speed_of_sound*rotor.cruise.design_tip_mach)/rotor.tip_radius
        rotor_conditions.pitch_command[:,0]   = rotor.cruise.design_pitch_command
        
        compute_rotor_performance(electric_rotor,segment.state)   
        
        # Pack the results
        nexus.results.cruise.thrust                      = -segment.state.conditions.energy[electric_rotor.tag][rotor.tag].thrust[0,2] 
        nexus.results.cruise.torque                      = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].torque[0][0]
        nexus.results.cruise.power                       = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].power[0][0]
        nexus.results.cruise.power_c                     = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].power_coefficient[0][0]
        nexus.results.cruise.omega                       = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].omega[0][0]
        nexus.results.cruise.thurst_c                    = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].thrust_coefficient[0][0]
        nexus.results.cruise.max_sectional_cl            = np.max(segment.state.conditions.energy[electric_rotor.tag][rotor.tag].lift_coefficient[0]) 
        nexus.results.cruise.mean_CL                     = np.mean(segment.state.conditions.energy[electric_rotor.tag][rotor.tag].lift_coefficient[0])  
        nexus.results.cruise.efficiency                  = segment.state.conditions.energy[electric_rotor.tag][rotor.tag].efficiency[0][0]
        nexus.results.cruise.conditions                  = conditions  
                    
        # microphone locations            
        ctrl_pts                                         = 1 
        theta                                            = rotor.optimization_parameters.noise_evaluation_angle 
        S_cruise                                         = np.maximum(altitude[0],20*Units.feet)  
        mic_positions_cruise                             = np.array([[0.0 ,S_cruise*np.sin(theta)  ,S_cruise*np.cos(theta)]])      
        
        # Run noise model  
        conditions.noise[bus.tag]                        = RCAIDE.Framework.Mission.Common.Conditions()      
        conditions.noise[electric_rotor.tag]    = RCAIDE.Framework.Mission.Common.Conditions()  
        conditions.noise.relative_microphone_locations   = np.repeat(mic_positions_cruise[ np.newaxis,:,: ],1,axis=0)
        conditions.aerodynamics.angles.alpha             = np.ones((ctrl_pts,1))* 0. * Units.degrees 
        segment                                          = RCAIDE.Framework.Mission.Segments.Segment() 
        segment.state.conditions                         = conditions
        segment.state.conditions.expand_rows(ctrl_pts)  
        noise                                            = RCAIDE.Framework.Analyses.Noise.Frequency_Domain_Buildup() 
        settings                                         = noise.settings   
        num_mic                                          = len(conditions.noise.relative_microphone_locations[0])  
        conditions.noise.number_of_microphones           = num_mic    
        
        if alpha != 1: 
            compute_rotor_noise(bus,electric_rotor,segment,settings)      
            nexus.results.cruise.mean_SPL   = np.mean(conditions.noise[electric_rotor.tag][rotor.tag].SPL_dBA)   
        else:
            nexus.results.cruise.mean_SPL   = 0  
            
    else:     
        nexus.results.cruise.thrust           = 0.0
        nexus.results.cruise.torque           = 0.0
        nexus.results.cruise.power            = 0.0
        nexus.results.cruise.power_c          = 0.0
        nexus.results.cruise.thurst_c         = 0.0
        nexus.results.cruise.omega            = 0.0
        nexus.results.cruise.max_sectional_cl = 0.0
        nexus.results.cruise.mean_CL          = 0.0
        nexus.results.cruise.efficiency       = 0.0  
        nexus.results.cruise.mean_SPL         = 0.0
        nexus.results.cruise.noise_data       = None   

    return nexus
   

# ----------------------------------------------------------------------
#   Post Process Results to give back to the optimizer
# ----------------------------------------------------------------------   
def post_process(nexus):
     
    rotor                           = nexus.vehicle_configurations.hover.networks.electric.propulsors.electric_rotor.rotor  
    rotor_oei                       = nexus.vehicle_configurations.oei.networks.electric.propulsors.electric_rotor.rotor
    alpha                           = rotor.optimization_parameters.multiobjective_aeroacoustic_weight
    beta                            = rotor.optimization_parameters.multiobjective_performance_weight
    gamma                           = rotor.optimization_parameters.multiobjective_acoustic_weight
    ideal_SPL                       = rotor.optimization_parameters.ideal_SPL_dBA  
    ideal_efficiency                = rotor.optimization_parameters.ideal_efficiency      
    ideal_FoM                       = rotor.optimization_parameters.ideal_figure_of_merit  
    print_iter                      = nexus.print_iterations  
    mean_CL_hover                   = nexus.results.hover.mean_CL
    omega_hover                     = nexus.results.hover.omega
    FM_hover                        = nexus.results.hover.figure_of_merit  
    
    # q to p ratios 
    summary                                 = nexus.summary 
    summary.max_sectional_cl_hover          = nexus.results.hover.max_sectional_cl
    summary.chord_p_to_q_ratio              = rotor.chord_p/rotor.chord_q
    summary.twist_p_to_q_ratio              = rotor.twist_p/rotor.twist_q      
    summary.blade_taper_constraint_1        = rotor.chord_distribution[-1]/rotor.chord_distribution[0]
    summary.blade_taper_constraint_2        = rotor.chord_distribution[-1]/rotor.chord_distribution[0]
    summary.blade_twist_constraint          = rotor.twist_distribution [0] - rotor.twist_distribution [-1] 
    summary.OEI_hover_thrust_power_residual = abs(nexus.results.oei.thrust - rotor.oei.design_thrust) / nexus.results.oei.thrust
            
    # thrust/power residuals  
    if rotor.hover.design_thrust == None:
        summary.hover_thrust_power_residual = abs(nexus.results.hover.power - rotor.hover.design_power) / rotor.hover.design_power
    else: 
        summary.hover_thrust_power_residual = abs(nexus.results.hover.thrust - rotor.hover.design_thrust) /rotor.hover.design_thrust 

    # oei
    if rotor.oei.design_thrust == None:
        summary.oei_thrust_power_residual =  abs(nexus.results.oei.power - rotor.oei.design_power) / rotor.oei.design_power
    else: 
        summary.oei_thrust_power_residual = abs(nexus.results.oei.thrust - rotor.oei.design_thrust) /rotor.oei.design_thrust 
    
        
    if nexus.prop_rotor_flag: 
        if rotor.cruise.design_thrust == None:
            summary.cruise_thrust_power_residual = abs(nexus.results.cruise.power - rotor.cruise.design_power) /rotor.cruise.design_power 
        else: 
            summary.cruise_thrust_power_residual =  abs(nexus.results.cruise.thrust - rotor.cruise.design_thrust) /rotor.cruise.design_thrust    
            
    # -------------------------------------------------------
    # OBJECTIVE FUNCTION
    # -------------------------------------------------------   
    performance_objective  = ((ideal_FoM - FM_hover)/ideal_FoM)*beta +  ((ideal_efficiency - nexus.results.cruise.efficiency)/ideal_efficiency)*(1-beta) 
    
    acoustic_objective     = ((nexus.results.hover.mean_SPL  - ideal_SPL)/ideal_SPL)*gamma  + ((nexus.results.cruise.mean_SPL - ideal_SPL)/ideal_SPL)*(1-gamma) 
 
    summary.objective      = (performance_objective*alpha + acoustic_objective*(1-alpha))  
    

    if nexus.prop_rotor_flag:  
        rotor_cru  = nexus.vehicle_configurations.cruise.networks.electric.propulsors.electric_rotor.rotor         
        summary.max_sectional_cl_cruise = nexus.results.cruise.max_sectional_cl   
        
    # -------------------------------------------------------
    # PRINT ITERATION PERFOMRMANCE
    # -------------------------------------------------------      
    if print_iter:
        print("Aeroacoustic Weight          : " + str(alpha))  
        print("Multiobj. Performance Weight : " + str(beta))  
        print("Multiobj. Acoustic Weight    : " + str(gamma)) 
        print("Performance Obj              : " + str(performance_objective))   
        print("Acoustic Obj                 : " + str(acoustic_objective))  
        print("Aeroacoustic Obj             : " + str(summary.objective))    
        print("Blade Taper                  : " + str(summary.blade_taper_constraint_1))
        print("Hover RPM                    : " + str(omega_hover/Units.rpm))     
        if rotor.hover.design_thrust == None: 
            print("Hover Power                  : " + str(nexus.results.hover.power))  
        if rotor.hover.design_power == None: 
            print("Hover Thrust                 : " + str(nexus.results.hover.thrust))  
        print("Hover Average SPL            : " + str(nexus.results.hover.mean_SPL))    
        print("Hover Tip Mach               : " + str(rotor.hover.design_tip_mach)) 
        print("Hover Thrust/Power Residual  : " + str(summary.hover_thrust_power_residual)) 
        print("Hover Figure of Merit        : " + str(FM_hover))  
        print("Hover Max Sectional Cl       : " + str(summary.max_sectional_cl_hover)) 
        print("Hover Blade CL               : " + str(mean_CL_hover))    
        print("OEI Thrust                   : " + str(nexus.results.oei.thrust)) 
        print("OEI Thrust/Power Residual    : " + str(summary.OEI_hover_thrust_power_residual)) 
        print("OEI Tip Mach                 : " + str(rotor_oei.oei.design_tip_mach))  
        print("OEI Collective (deg)         : " + str(rotor_oei.design_pitch_command/Units.degrees)) 
        if nexus.prop_rotor_flag:    
            print("Cruise RPM                   : " + str(nexus.results.cruise.omega/Units.rpm))    
            print("Cruise Collective (deg)      : " + str(rotor_cru.design_pitch_command/Units.degrees)) 
            if rotor_cru.cruise.design_thrust == None:  
                print("Cruise Power                 : " + str(nexus.results.cruise.power)) 
            if rotor_cru.cruise.design_power == None:  
                print("Cruise Thrust                : " + str(nexus.results.cruise.thrust))   
            print("Cruise Tip Mach              : " + str(rotor_cru.cruise.design_tip_mach))  
            print("Cruise Thrust/Power Residual : " + str(summary.cruise_thrust_power_residual))
            print("Cruise Efficiency            : " + str(nexus.results.cruise.efficiency)) 
            print("Cruise Max Sectional Cl      : " + str(summary.max_sectional_cl_cruise))  
            print("Cruise Blade CL              : " + str(nexus.results.cruise.mean_CL))  
        print("\n\n") 

   
    return nexus    
