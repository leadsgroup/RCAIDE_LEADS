# RCAIDE Imports 

# Imports    
import RCAIDE
from RCAIDE.Framework.Core import Units, Data  
from RCAIDE.Library.Methods.Noise.Frequency_Domain_Buildup.Rotor                  import compute_rotor_noise
from RCAIDE.Framework.Mission.Common                                              import Results  
from RCAIDE.Framework.Mission.Segments.Segment                                    import Segment 
from RCAIDE.Framework.Mission.Common                                              import Conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.compute_rotor_performance import compute_rotor_performance 
from RCAIDE.Library.Plots import * 

# Python Imports  
import sys 
import numpy as np  
import matplotlib.pyplot as plt 
import time   
import os

sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles' + os.path.sep + 'Rotors')) 
# the analysis functions

from F8745_D4_Propeller  import F8745_D4_Propeller
from APC_11x4_Propeller  import APC_11x4_Propeller 
# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main(): 
    '''This regression script is for validation and verification of the mid-fidelity acoustics
    analysis routine. "Experimental Data is obtained from Comparisons of predicted propeller
    noise with windtunnel ..." by Weir, D and Powers, J.
    '''  
    # define plotting parameters 
    PP = plot_parameters() 
       
    # harmonic noise test 
    Harmonic_Noise_Validation(PP)

    # broadband nosie test function 
    Broadband_Noise_Validation(PP)
    
    return  
    
    
# ------------------------------------------------------------------ 
# Harmonic Noise Validation
# ------------------------------------------------------------------  
def Harmonic_Noise_Validation(PP):
    
    F8745_D4_verification_values_60deg = [156.92733365790193,100.86943888034477,109.60611659108622 ]
    F8745_D4_verification_values_90deg = [159.07711862067382,107.37402397630274,117.14527113998096 ]
    fidelities                         = ['point_source', 'line_source', 'plane_source'] 
    electric_rotor                     = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor() 
    rotor                              = F8745_D4_Propeller() 
    electric_rotor.rotor               = rotor
    
    plot_rotor(rotor, save_figure=False, show_figure=False)
    
    # Operating Conditions                                             
    a                       = 343.376
    T                       = 288.16889478  
    density                 = 1.2250	
    dynamic_viscosity       = 1.81E-5  
    theta                   = np.array([1,10,20,30.1,40,50,59.9,70,80,89.9,100,110,120.1,130,140,150.1,160,170,179])  
    S                       = 4.
    test_omega              = np.array([2390,2710,2630]) * Units.rpm    
    ctrl_pts                = len(test_omega)
    AoA                     = np.zeros(ctrl_pts)
    true_course             = 0
    fligth_path_angle       = 0 
    three_quarter_twist     = 21 * Units.degrees 
    n                       = len(rotor.twist_distribution)
    beta                    = rotor.twist_distribution
    beta_75                 = beta[round(n*0.75)] 
    delta_beta              = three_quarter_twist-beta_75
    rotor.twist_distribution = beta + delta_beta 

    # microphone locations
    mic_positions = np.zeros(( len(theta),3))
    for i in range(len(theta)):
        if theta[i]*Units.degrees < np.pi/2:
            mic_positions[i][:] = [-S*np.cos(theta[i]*Units.degrees)  ,S*np.sin(theta[i]*Units.degrees), 0.0]
        else: 
            mic_positions[i][:] = [S*np.sin(theta[i]*Units.degrees- np.pi/2)  ,S*np.cos(theta[i]*Units.degrees - np.pi/2), 0.0]   
 
    segment                                                = Segment()  
    conditions                                             = Results() 
    conditions.noise.relative_microphone_locations         = np.repeat(mic_positions[ np.newaxis,:,: ],1,axis=0)   
    conditions.aerodynamics.angles.alpha                   = np.atleast_2d(AoA).T 
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * density
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T  
    conditions.frames.inertial.velocity_vector             = np.array([[77.2, 0. ,0.],[ 77.0,0.,0.], [ 77.2, 0. ,0.]]) 
    conditions.freestream.mach_number                      = np.atleast_2d(np.linalg.norm(conditions.frames.inertial.velocity_vector,axis = 1)).T/ a
    conditions.frames.planet.true_course                   = np.zeros((ctrl_pts,3,3)) 
    conditions.frames.planet.true_course[:,0,0]            = np.cos(true_course),
    conditions.frames.planet.true_course[:,0,1]            = - np.sin(true_course)
    conditions.frames.planet.true_course[:,1,0]            = np.sin(true_course)
    conditions.frames.planet.true_course[:,1,1]            = np.cos(true_course) 
    conditions.frames.planet.true_course[:,2,2]            = 1 
    conditions.frames.wind.transform_to_inertial           = np.zeros((ctrl_pts,3,3))   
    conditions.frames.wind.transform_to_inertial[:,0,0]    = np.cos(fligth_path_angle) 
    conditions.frames.wind.transform_to_inertial[:,0,2]    = np.sin(fligth_path_angle) 
    conditions.frames.wind.transform_to_inertial[:,1,1]    = 1 
    conditions.frames.wind.transform_to_inertial[:,2,0]    = -np.sin(fligth_path_angle) 
    conditions.frames.wind.transform_to_inertial[:,2,2]    = np.cos(fligth_path_angle)  
    conditions.frames.body.transform_to_inertial           = np.zeros((ctrl_pts,3,3))
    conditions.frames.body.transform_to_inertial[:,0,0]    = np.cos(AoA)
    conditions.frames.body.transform_to_inertial[:,0,2]    = np.sin(AoA)
    conditions.frames.body.transform_to_inertial[:,1,1]    = 1
    conditions.frames.body.transform_to_inertial[:,2,0]    = -np.sin(AoA)
    conditions.frames.body.transform_to_inertial[:,2,2]    = np.cos(AoA)     
    segment.state.conditions                               = conditions 
    
    # Run BEMT for Unsteady loading
    rotor.number_azimuthal_stations                        = 16
    rotor.use_2d_analysis                                  = True
    
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Plot Validation Results
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
    validation_data, axes_1_1, axes_1_2, axes_1_3, axes_1_4, axes_1_5, axes_1_6, axes_2_1 = Hararmonic_Noise_Validation_Data(PP)    
    
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Run simulation using different fidelities 
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    for fid in  range(len(fidelities)): 
        electric_rotor.append_operating_conditions(segment) 
        for tag, item in  electric_rotor.items(): 
            if issubclass(type(item), RCAIDE.Library.Components.Component):
                item.append_operating_conditions(segment,electric_rotor)  
        # Run BEMT
        segment.state.conditions.expand_rows(ctrl_pts)
        rotor_conditions             =  segment.state.conditions.energy[electric_rotor.tag][rotor.tag]     
        rotor_conditions.omega[:,0]  = test_omega
        compute_rotor_performance(electric_rotor,segment.state)      
        
        noise                                                  = RCAIDE.Framework.Analyses.Noise.Frequency_Domain_Buildup() 
        settings                                               = noise.settings
        settings.fidelity                                      = fidelities[fid]  
        conditions.noise.number_of_microphones                 = len(theta)
        
        # time
        ti  = time.time()
        # Run Frequency Domain Rotor Noise Model           
        compute_rotor_noise(mic_positions,electric_rotor,rotor,segment,settings)
        tf  = time.time()
        
        F8745D4_SPL                                            = conditions.noise[electric_rotor.tag][rotor.tag].SPL     
        F8745D4_SPL_harmonic                                   = conditions.noise[electric_rotor.tag][rotor.tag].SPL_harmonic 
        F8745D4_SPL_broadband                                  = conditions.noise[electric_rotor.tag][rotor.tag].SPL_broadband  
        F8745D4_SPL_harmonic_bpf_spectrum                      = conditions.noise[electric_rotor.tag][rotor.tag].SPL_harmonic_bpf_spectrum     
        
        Cp =  segment.state.conditions.energy[electric_rotor.tag][rotor.tag].power_coefficient

        
        # plot results 
        axes_1_1.plot(validation_data.harmonics, F8745D4_SPL_harmonic_bpf_spectrum[0,6,:][:len(validation_data.harmonics)]   , color = PP.Slc[fid] , linestyle = PP.Sls, marker = PP.Slm[0] , markersize = PP.m , linewidth = PP.lw, label = fidelities[fid])     
        axes_1_1.set_title('60 deg. Case 1, $C_P$ = ' + str(round(Cp[0,0],3))) 
         
        axes_1_2.plot(validation_data.harmonics, F8745D4_SPL_harmonic_bpf_spectrum[1,6,:][:len(validation_data.harmonics)] , color = PP.Slc[fid] , linestyle = PP.Sls, marker = PP.Slm[0]  , markersize = PP.m , linewidth = PP.lw,   label = fidelities[fid])    
        axes_1_2.set_title('60 deg. Case 2, $C_P$ = ' +  str(round(Cp[1,0],3)))   
  
        axes_1_3.plot(validation_data.harmonics, F8745D4_SPL_harmonic_bpf_spectrum[2,6,:][:len(validation_data.harmonics)] , color = PP.Slc[fid] , linestyle = PP.Sls, marker = PP.Slm[0]  , markersize = PP.m , linewidth = PP.lw,   label = fidelities[fid])        
        axes_1_3.set_title('60 deg. Case 3, $C_P$ = ' +  str(round(Cp[2,0],3))) 
          
        axes_1_4.plot(validation_data.harmonics, F8745D4_SPL_harmonic_bpf_spectrum[0,9,:][:len(validation_data.harmonics)] , color = PP.Slc[fid] , linestyle = PP.Sls, marker = PP.Slm[0]  , markersize = PP.m , linewidth = PP.lw,  label = fidelities[fid])        
        axes_1_4.set_title('90 deg. Case 1, $C_P$ = ' + str(round(Cp[0,0],3))) 
            
        axes_1_5.plot(validation_data.harmonics, F8745D4_SPL_harmonic_bpf_spectrum[1,9,:][:len(validation_data.harmonics)]  , color = PP.Slc[fid] , linestyle = PP.Sls, marker = PP.Slm[0]  , markersize = PP.m , linewidth = PP.lw, label = fidelities[fid])     
        axes_1_5.set_title('90 deg. Case 2, $C_P$ = ' +  str(round(Cp[1,0],3)))    
       
        axes_1_6.plot(validation_data.harmonics, F8745D4_SPL_harmonic_bpf_spectrum[2,9,:][:len(validation_data.harmonics)] , color = PP.Slc[fid] , linestyle = PP.Sls, marker = PP.Slm[0]  , markersize = PP.m , linewidth = PP.lw,   label = fidelities[fid])       
        axes_1_6.set_title('90 deg. Case 3, $C_P$ = ' +  str(round(Cp[2,0],3)))    
        
        
        # Polar plot of noise    
        axes_2_1.plot(theta*Units.degrees,F8745D4_SPL[0,:] , color = PP.Slc[0] , linestyle =PP.Sls, marker = PP.Slm[0] , markersize = PP.m*2 , linewidth = PP.lw, label = 'Total'  )  
        axes_2_1.plot(-theta*Units.degrees,F8745D4_SPL[0,:] , color = PP.Slc[0] , linestyle =PP.Sls , marker = PP.Slm[0] , markersize = PP.m*2 , linewidth = PP.lw  )  
        axes_2_1.plot(theta*Units.degrees,F8745D4_SPL_harmonic[0,:] , color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm[0] , markersize = PP.m , linewidth = PP.lw   )  
        axes_2_1.plot(-theta*Units.degrees,F8745D4_SPL_harmonic[0,:] , color = PP.Slc[1] , linestyle = PP.Sls, marker = PP.Slm[0] , markersize = PP.m , linewidth = PP.lw, label = 'Harmonic'  )  
        axes_2_1.plot(theta*Units.degrees,F8745D4_SPL_broadband[0,:] , color = PP.Slc[2] , linestyle = PP.Sls, marker = PP.Slm[0] , markersize = PP.m , linewidth = PP.lw   )  
        axes_2_1.plot(-theta*Units.degrees,F8745D4_SPL_broadband[0,:] , color = PP.Slc[2] , linestyle = PP.Sls, marker = PP.Slm[0] , markersize = PP.m , linewidth = PP.lw, label = 'Broadband' )  

        # Store errors 
        error = Data()
        print(fidelities[fid] + ' Noise Modeling: Compute Time (per mic)  = '+  str((tf - ti)/len(theta)))
        print(fidelities[fid] + ' Noise Modeling: SPL at 60 deg = ' + str(F8745D4_SPL[0,6]))
        print(fidelities[fid] + ' Noise Modeling: SPL at 90 deg = ' + str(F8745D4_SPL[0,9]))
        error.SPL_Case_1_60deg  = np.max(np.abs(F8745D4_SPL[0,6] - F8745_D4_verification_values_60deg[fid])/F8745_D4_verification_values_60deg[fid])  
        error.SPL_Case_1_90deg  = np.max(np.abs(F8745D4_SPL[0,9] - F8745_D4_verification_values_90deg[fid])/F8745_D4_verification_values_90deg[fid])
         
        print( fidelities[fid] + ' Harmonic Noise Errors:')
        print(error)
        
        for k,v in list(error.items()):
            assert(np.abs(v)<1E0)
        
    axes_1_5.legend(loc='upper center', prop={'size': PP.lf} , bbox_to_anchor=(0.5, -0.4), ncol= 3 )  
    axes_2_1.legend(loc='upper right', prop={'size': PP.lf} , bbox_to_anchor=(1.2,1.5))    

    return  
 

# ------------------------------------------------------------------ 
# Broadband Noise Validation
# ------------------------------------------------------------------     
def Broadband_Noise_Validation(PP):
    APC_broadboand_verification_60deg = np.array([15.12527918, 17.05865653, 19.35486775, 21.70880183, 24.34934545,
                                                  27.34543027, 30.7553328 , 33.87619342, 36.61322209, 39.03247278,
                                                  40.96849047, 42.53019394, 43.7858881 , 44.59572055, 46.58461962,
                                                  49.15271767, 49.61433894, 50.79835398, 50.95238605, 52.37793939,
                                                  54.40820869])
    APC_broadboand_verification_90deg = np.array([21.28334833, 23.21667437, 25.51424316, 27.87113539, 30.51433249,
                                                  33.51177464, 36.92264228, 40.04222718, 42.77577719, 45.1930248 ,
                                                  47.12976239, 48.69430141, 49.95308047, 50.76434641, 52.72373368,
                                                  55.2634795 , 55.73659471, 56.92545406, 57.11246906, 58.55137706,
                                                  60.58095738])

    
    
    electric_rotor                 = RCAIDE.Library.Components.Powertrain.Propulsors.Electric_Rotor() 
    rotor                          = APC_11x4_Propeller()  
    electric_rotor.rotor           = rotor     
    
    # Operating Conditions       
    APC_SF_inflow_ratio           = 0.08  
    a                             = 343   
    density                       = 1.225
    dynamic_viscosity             = 1.78899787e-05   
    T                             = 286.16889478                       
    APC_SF_RPM                    = np.array([3600,4200,4800])
    APC_SF_omega_vector           = APC_SF_RPM * Units.rpm 
    ctrl_pts                      = len(APC_SF_omega_vector)   
    velocity                      = APC_SF_inflow_ratio*APC_SF_omega_vector*rotor.tip_radius 
    theta                         = np.array([45., 67.5, 90.001, 112.5 , 135.])  # np.linspace(0.1,180,100)  
    S                             = 1.905
    AoA                           = np.zeros(ctrl_pts)
    true_course                   = 0
    fligth_path_angle             = 0    

    # Microphone Locations 
    mic_positions = np.zeros((len(theta),3))
    for i in range(len(theta)):
        if theta[i]*Units.degrees < np.pi/2:
            mic_positions[i][:] = [-S*np.cos(theta[i]*Units.degrees),-S*np.sin(theta[i]*Units.degrees), 0.0]
        else: 
            mic_positions[i][:] = [S*np.sin(theta[i]*Units.degrees- np.pi/2),-S*np.cos(theta[i]*Units.degrees - np.pi/2), 0.0] 

    # Define conditions    
    segment                                                = Segment()  
    conditions                                             = Results() 
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * density
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T
    v_mat                                                  = np.zeros((ctrl_pts,3))
    v_mat[:,0]                                             = velocity 
    conditions.frames.inertial.velocity_vector             = v_mat 
    conditions.energy.throttle                             = np.ones((ctrl_pts,1)) * 1.0    
    conditions.aerodynamics.angle_of_attack                = np.atleast_2d(AoA).T  
    conditions.freestream.mach_number                      = np.atleast_2d(np.linalg.norm(conditions.frames.inertial.velocity_vector,axis = 1)).T/ a
    conditions.frames.planet.true_course                   = np.zeros((ctrl_pts,3,3)) 
    conditions.frames.planet.true_course[:,0,0]            = np.cos(true_course),
    conditions.frames.planet.true_course[:,0,1]            = - np.sin(true_course)
    conditions.frames.planet.true_course[:,1,0]            = np.sin(true_course)
    conditions.frames.planet.true_course[:,1,1]            = np.cos(true_course) 
    conditions.frames.planet.true_course[:,2,2]            = 1 
    conditions.frames.wind.transform_to_inertial           = np.zeros((ctrl_pts,3,3))   
    conditions.frames.wind.transform_to_inertial[:,0,0]    = np.cos(fligth_path_angle) 
    conditions.frames.wind.transform_to_inertial[:,0,2]    = np.sin(fligth_path_angle) 
    conditions.frames.wind.transform_to_inertial[:,1,1]    = 1 
    conditions.frames.wind.transform_to_inertial[:,2,0]    = -np.sin(fligth_path_angle) 
    conditions.frames.wind.transform_to_inertial[:,2,2]    = np.cos(fligth_path_angle)  
    conditions.frames.body.transform_to_inertial           = np.zeros((ctrl_pts,3,3))
    conditions.frames.body.transform_to_inertial[:,0,0]    = np.cos(AoA)
    conditions.frames.body.transform_to_inertial[:,0,2]    = np.sin(AoA)
    conditions.frames.body.transform_to_inertial[:,1,1]    = 1
    conditions.frames.body.transform_to_inertial[:,2,0]    = -np.sin(AoA)
    conditions.frames.body.transform_to_inertial[:,2,2]    = np.cos(AoA)  
    segment.state.conditions                               = conditions
    
    # Run BEMT for Unsteady loading
    rotor.number_azimuthal_stations                        = 16
    rotor.use_2d_analysis                                  = True
     
    electric_rotor.append_operating_conditions(segment) 
    for tag, item in  electric_rotor.items(): 
        if issubclass(type(item), RCAIDE.Library.Components.Component):
            item.append_operating_conditions(segment,electric_rotor)  
    # Run BEMT
    segment.state.conditions.expand_rows(ctrl_pts)
    rotor_conditions             =  segment.state.conditions.energy[electric_rotor.tag][rotor.tag]     
    rotor_conditions.omega[:,0]  = APC_SF_omega_vector
    compute_rotor_performance(electric_rotor,segment.state)

    noise                                                  = RCAIDE.Framework.Analyses.Noise.Frequency_Domain_Buildup() 
    settings                                               = noise.settings
    settings.fidelity                                      = 'plane_source' 
    conditions.noise.number_of_microphones                 = len(theta)
    
    # time
    ti = time.time()             
    # Run Frequency Domain Rotor Noise Model           
    compute_rotor_noise(mic_positions,electric_rotor,rotor,segment,settings)
    tf = time.time()
    
    APC_SF_1_3_Spectrum                                     = conditions.noise[electric_rotor.tag][rotor.tag].SPL_1_3_spectrum 
    APC_SF_SPL_broadband_1_3_spectrum                       = conditions.noise[electric_rotor.tag][rotor.tag].SPL_broadband_1_3_spectrum  

    axes_3_1,axes_3_2, axes_3_3, axes_3_4, validation_data = Broadband_Noise_Validation_Data(PP)


    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Plots  
    # ----------------------------------------------------------------------------------------------------------------------------------------    
    axes_3_1.plot(validation_data.Exp_APC_SF_freqency_spectrum ,     APC_SF_1_3_Spectrum[0,0,8:]  , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm[0] , markersize = PP.m , linewidth = PP.lw, label =' RCAIDE 3600 RPM')    
    axes_3_1.legend(loc='lower right')
    
    axes_3_2.plot(validation_data.Exp_APC_SF_freqency_spectrum ,     APC_SF_1_3_Spectrum[2,0,8:]  , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm[0] , markersize = PP.m , linewidth = PP.lw,label = ' RCAIDE 4800 RPM')   
    axes_3_2.legend(loc='lower right')
    
    axes_3_3.plot(validation_data.Exp_APC_SF_freqency_spectrum , APC_SF_SPL_broadband_1_3_spectrum[1,4,8:] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm[0] , markersize = PP.m , linewidth = PP.lw,  label = 'RCAIDE 45 $\degree$ mic')     
    axes_3_3.legend(loc='lower right')   

    axes_3_4.plot(validation_data.Exp_APC_SF_freqency_spectrum , APC_SF_SPL_broadband_1_3_spectrum[1,3,8:] , color = PP.Slc[0] , linestyle = PP.Sls, marker = PP.Slm[0] , markersize = PP.m , linewidth = PP.lw,  label = 'RCAIDE 22.5 $\degree$ mic.')   
    axes_3_4.legend(loc='lower right')  

    # Store errors 
    error = Data()
    print('Compute Time  ='+  str(tf - ti))
    error.SPL_Case_1_60deg  = np.max(np.abs(APC_SF_SPL_broadband_1_3_spectrum[1,3,8:]  - APC_broadboand_verification_60deg)/APC_broadboand_verification_60deg)
    error.SPL_Case_1_90deg  = np.max(np.abs(APC_SF_SPL_broadband_1_3_spectrum[1,4,8:] - APC_broadboand_verification_90deg)/APC_broadboand_verification_90deg)
    
    print('Broadband Noise Errors:')
    print(error)
    
    for k,v in list(error.items()):
        assert(np.abs(v)<1E0)
        
    return 

def Hararmonic_Noise_Validation_Data(PP):  
    validation_data = Data()

    validation_data.harmonics              = np.arange(1,19)
    validation_data.ANOPP_PAS_Case_1_60deg = np.array([105.82,101.08,100.13,97.581,94.035,89.095,82.957,
                                       80.609,81.052,72.718,70.772,68.023,67.072,53.949,
                                       np.inf,np.inf,np.inf,np.inf]) 

    validation_data.Exp_Test_Case_1_60deg  = np.array([103.23,100.08,98.733,94.990,91.045,87.500,82.161,79.012,
                                       74.469,70.128,65.784,61.241,56.699,51.958,47.013,42.673,
                                       37.927,32.989,]) 

    validation_data.ANOPP_PAS_Case_1_90deg = np.array([107.671,104.755,105.829,103.307,101.385,100.465,99.1399,
                                       96.8208,93.6988,91.7765,89.6573,86.5323,85.2098,83.4874,
                                       78.1692,75.4503,73.7248,72.0024]) 

    validation_data.Exp_Test_Case_1_90deg  = np.array([108.077,107.554,105.626,103.307,100.988,100.068,98.9430,
                                       96.8208,93.6988,91.7796,89.6542,85.5295,85.0099,81.8879,
                                       77.9724,74.8566,73.1250,71.2057  ])    

    validation_data.Exp_Test_Case_2_60deg = np.array([111.951,108.175,108.789,106.352,105.059,100.140,100.945,
                                      99.8430,93.9683,93.8203,91.1914,85.3167,85.3626,82.1580,
                                       78.1933,75.7552,80.1887,72.2133])

    validation_data.ANOPP_PAS_Case_2_60deg = np.array([111.760,111.421,108.984,106.352,104.487,101.856,99.0369,
                                       95.4522,93.2050,89.4327,86.2313,82.6498,79.2559,75.4804,
                                       71.1356,68.1219,63.9663,60.0000]) 


    validation_data.Exp_Test_Case_2_90deg  = np.array([115.587,113.363,115.520,113.868,113.365,111.331,
                                       113.491,110.507,109.999,109.873,107.649,106.949,
                                       106.822,103.079,103.715,102.633,99.6502,97.8095])

    validation_data.ANOPP_PAS_Case_2_90deg = np.array([115.397,115.273,114.377,113.870,113.362,111.143,
                                       110.631,109.752,108.859,107.585,109.175,105.234,
                                       103.782,102.127,101.236,99.7790,98.7002,98.9523  ])  

    validation_data.Exp_Test_Case_3_60deg  = np.array([110.93,107.28,108.60,106.28,104.17,99.377,
                                       100.69,100.28,95.688,95.094,92.975,84.365,
                                       84.533,82.224,77.622,77.411,78.152,74.312])
    validation_data.ANOPP_PAS_Case_3_60deg = np.array([110.93,110.53,108.41,107.43,104.55,101.47,
                                       98.592,95.328,92.635,88.987,86.103,83.028,
                                       79.573,76.114,73.040,69.775,65.554,61.908 ])

    validation_data.Exp_Test_Case_3_90deg  = np.array([114.499,112.135,114.674,112.898,112.299,111.308,
                                       112.473,110.894,109.510,109.303,107.724,107.124,
                                       106.133,102.790,103.758,101.983,99.2279,98.0404])

    validation_data.ANOPP_PAS_Case_3_90deg = np.array([114.499,114.291,113.889,113.879,111.122,110.523,
                                       109.924,109.129,108.725,107.342,106.743,105.164,
                                       104.369,102.593,101.210,100.021,98.6401,96.6674])
    
    fig_1 = plt.figure('Harmonic Test')
    fig_1.set_size_inches(PP.fig_size_width,PP.fig_size_height)
    fig_1.tight_layout()
    axes_1_1 = fig_1.add_subplot(2,3,1)     
    axes_1_1.plot(validation_data.harmonics, validation_data.ANOPP_PAS_Case_1_60deg                                      , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm[0]  , markersize = PP.m , linewidth = PP.lw, label = 'ANOPP PAS')       
    axes_1_1.plot(validation_data.harmonics, validation_data.Exp_Test_Case_1_60deg                                       , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm[0]  , markersize = PP.m , linewidth = PP.lw,  label = 'Exp.')    
    axes_1_1.set_ylabel('SPL (dB)') 
    axes_1_1.minorticks_on() 
     
    # Test Case 2
    axes_1_2 = fig_1.add_subplot(2,3,2)  
    axes_1_2.plot(validation_data.harmonics, validation_data.ANOPP_PAS_Case_2_60deg                                    , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm[0]  , markersize = PP.m , linewidth = PP.lw,   label = 'ANOPP PAS')       
    axes_1_2.plot(validation_data.harmonics, validation_data.Exp_Test_Case_2_60deg                                     , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm[0]  , markersize = PP.m , linewidth = PP.lw,   label = 'Exp.')  
    axes_1_2.minorticks_on()   

    # Test Case 3
    axes_1_3 = fig_1.add_subplot(2,3,3)   
    axes_1_3.plot(validation_data.harmonics, validation_data.ANOPP_PAS_Case_3_60deg                                    , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm[0]  , markersize = PP.m , linewidth = PP.lw,   label = 'ANOPP PAS')       
    axes_1_3.plot(validation_data.harmonics, validation_data.Exp_Test_Case_3_60deg                                     , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm[0]  , markersize = PP.m , linewidth = PP.lw,  label = 'Exp.')        
    axes_1_3.minorticks_on() 
 
    axes_1_4 = fig_1.add_subplot(2,3,4)       
    axes_1_4.plot(validation_data.harmonics, validation_data.ANOPP_PAS_Case_1_90deg                                    , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm[0]  , markersize = PP.m , linewidth = PP.lw,   label = 'ANOPP PAS')       
    axes_1_4.plot(validation_data.harmonics, validation_data.Exp_Test_Case_1_90deg                                     , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm[0]  , markersize = PP.m , linewidth = PP.lw,  label = 'Exp.')       
    axes_1_4.set_ylabel('SPL (dB)')
    axes_1_4.set_xlabel('Harmonic #')   
    axes_1_4.minorticks_on() 

    axes_1_5 = fig_1.add_subplot(2,3,5)                  
    axes_1_5.plot(validation_data.harmonics, validation_data.ANOPP_PAS_Case_2_90deg                                     , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm[0]  , markersize = PP.m , linewidth = PP.lw, label = 'ANOPP PAS')       
    axes_1_5.plot(validation_data.harmonics, validation_data.Exp_Test_Case_2_90deg                                      , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm[0]  , markersize = PP.m , linewidth = PP.lw, label = 'Exp.')   
    axes_1_5.set_xlabel('Harmonic #')  
    axes_1_5.minorticks_on() 

    axes_1_6 = fig_1.add_subplot(2,3,6)         
    axes_1_6.plot(validation_data.harmonics, validation_data.ANOPP_PAS_Case_3_90deg                                    , color = PP.Rlc[0] , linestyle = PP.Rls, marker = PP.Rlm[0]  , markersize = PP.m , linewidth = PP.lw,   label = 'ANOPP PAS')       
    axes_1_6.plot(validation_data.harmonics, validation_data.Exp_Test_Case_3_90deg                                     , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm[0]  , markersize = PP.m , linewidth = PP.lw,  label = 'Exp.')     
    axes_1_6.set_xlabel('Harmonic #')  
    axes_1_6.minorticks_on()
    

    # Polar plot of noise   
    fig_2 = plt.figure('Polar')
    axes_2_1 = fig_2.add_subplot(111, projection='polar')         
    axes_2_1.set_yticks(np.arange(50,150,25))     
    axes_2_1.grid(True)  


    return validation_data,  axes_1_1, axes_1_2, axes_1_3, axes_1_4, axes_1_5, axes_1_6, axes_2_1


def Broadband_Noise_Validation_Data(PP):  
    validation_data = Data()      
    validation_data.Exp_APC_SF_freqency_spectrum =  np.array([100, 125, 160, 200, 250, 315, 400, 500, 630, 800, 1000, 1250, 1600, 
                                              2000, 2500, 3150,4000, 5000, 6300, 8000, 10000])   

    validation_data.Exp_APC_SF_1_3_Spectrum      = np.array([[22.149, 42.242, 24.252, 22.616, 26.121, 24.953, 28.925, 29.158, 39.205
                                              , 42.943, 39.205, 38.971, 47.149, 45.280, 40.373, 38.738, 38.037, 39.906
                                              , 41.308, 45.981, 42.710, 39.205, 41.775, 37.570, 34.065, 33.598 ],
                                             [17.943,46.214,46.214,22.850,27.056,27.990,31.495,31.261,37.336,
                                              42.242,50.186,40.373,45.280,42.476,45.747,43.878,43.878,48.084,
                                              48.317,49.252,49.018,49.018,46.214,42.242,40.140,39.205 ],
                                             [ 19.345, 18.411, 54.859, 24.018, 26.355, 34.065, 33.130, 33.130, 36.635
                                               , 45.981, 45.046, 40.841, 42.710, 43.411, 44.813, 51.588, 45.981, 46.915
                                              , 52.289, 48.551, 50.186, 48.551, 48.551, 48.317, 43.177, 41.308]])   

    validation_data.Exp_broadband_APC = np.array([[24.8571428,27.7142857,28.8571428,26.5714285,27.1428571,27.7142857,30.2857142,32,35.4285714,
                                   39.1428571,40.5714285,39.4285714,40.5714285,40,41.1428571,40.2857142,41.7142857,44,44.5714285,
                                   44.8571428,45.1428571],
                                  [23.42857,26,27.14285,25.14285,25.71428,26.57142,28.28571,29.14285,34.28571,37.14285,
                                   37.99999,34.57142,39.14285,34,35.71428,34.85714,35.99999,43.42857,39.14285,41.14285,
                                   42.57142]]) 

    # ----------------------------------------------------------------------------------------------------------------------------------------
    #  Plots  
    # ----------------------------------------------------------------------------------------------------------------------------------------    
    fig_3_1 = plt.figure('Noise_Validation_Total_1_3_Spectrum_3600')    
    fig_3_1.set_size_inches(PP.fig_size_width,PP.fig_size_height)
    fig_3_1.tight_layout()
    axes_3_1 = fig_3_1.add_subplot(1,1,1)      
    axes_3_1.plot(validation_data.Exp_APC_SF_freqency_spectrum , validation_data.Exp_APC_SF_1_3_Spectrum[0,:-5]  , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm[0] , markersize = PP.m , linewidth = PP.lw, label = 'Exp. 3600 RPM')       
    axes_3_1.set_xscale('log') 
    axes_3_1.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes_3_1.set_xlabel('Frequency (Hz)')  
    axes_3_1.set_ylim([15,60])    

    fig_3_2 = plt.figure('Noise_Validation_1_3_Spectrum_4800')    
    fig_3_2.set_size_inches(PP.fig_size_width,PP.fig_size_height)
    fig_3_2.tight_layout()
    axes_3_2 = fig_3_2.add_subplot(1,1,1)           
    axes_3_2.plot(validation_data.Exp_APC_SF_freqency_spectrum , validation_data.Exp_APC_SF_1_3_Spectrum[2,:-5]  , color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm[0] , markersize = PP.m , linewidth = PP.lw,  label = 'Exp. 4800 RPM')     
    axes_3_2.set_xscale('log') 
    axes_3_2.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes_3_2.set_xlabel('Frequency (Hz)')  
    axes_3_2.set_ylim([15,60])   


    fig_3_3 = plt.figure('Noise_Validation_Broadband_1_3_Spectrum_45_deg')    
    fig_3_3.set_size_inches(PP.fig_size_width,PP.fig_size_height)
    fig_3_3.tight_layout()
    axes_3_3 = fig_3_3.add_subplot(1,1,1)           
    axes_3_3.plot(validation_data.Exp_APC_SF_freqency_spectrum , validation_data.Exp_broadband_APC[0,:], color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm[0] , markersize = PP.m , linewidth = PP.lw,    label = 'Exp. 45 $\degree$ mic.')       
    axes_3_3.set_xscale('log') 
    axes_3_3.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes_3_3.set_xlabel('Frequency (Hz)')  
    axes_3_3.set_ylim([15,50])   


    fig_3_4 = plt.figure('Noise_Validation_Broadband_1_3_Spectrum_22_deg')    
    fig_3_4.set_size_inches(PP.fig_size_width,PP.fig_size_height)
    fig_3_4.tight_layout()
    axes_3_4 = fig_3_4.add_subplot(1,1,1)            
    axes_3_4.plot(validation_data.Exp_APC_SF_freqency_spectrum , validation_data.Exp_broadband_APC[1,:], color = PP.Elc[0] , linestyle = PP.Els, marker = PP.Elm[0] , markersize = PP.m , linewidth = PP.lw,   label = 'Exp. 22.5 $\degree$ mic.')       
    axes_3_4.set_xscale('log') 
    axes_3_4.set_ylabel(r'SPL$_{1/3}$ (dB)')
    axes_3_4.set_xlabel('Frequency (Hz)')  
    axes_3_4.set_ylim([15,50])
    
    return axes_3_1,axes_3_2, axes_3_3, axes_3_4, validation_data

def plot_parameters():
     
    plt.rcParams.update({'font.size': 12})
    plt.rcParams['axes.linewidth'] = 1. 
 
    PP = Data(  
        fig_size_width  = 14 ,
        fig_size_height = 9 ,       
        lw  = 1,                             # line_width               
        m   = 5,                             # markersize               
        lf  = 10,                            # legend_font_size         
        Slc = ['black','green','yellow'],       # line_colors        
        Slm = ['^','o','s'],                 # line_markers       
        Sls = '-',                           # line_styles        
        Elc = ['darkred','red','tomato'],    # Experimental_line_colors 
        Elm = ['s'],                         # Experimental_line_markers
        Els = '',                            # Experimental_line_styles 
        Rlc = ['darkblue','blue','cyan'],    # Ref_Code_line_colors     
        Rlm = ['o'],                         # Ref_Code_line_markers    
        Rls = ':',                           # Ref_Code_line_styles     
    )   
    
    return PP  

def setup_noise_settings(sts): 

    sts.ground_microphone_phi_angles   = np.array([30.,45.,60.,75.,89.9,90.1,105.,120.,135.,150.])*Units.degrees
    sts.ground_microphone_theta_angles = np.array([89.9,89.9,89.9,89.9,89.9,89.9,89.9,89.9, 89.9,89.9 ])*Units.degrees
    sts.center_frequencies             = np.array([16,20,25,31.5,40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, \
                                                   500, 630, 800, 1000, 1250, 1600, 2000, 2500, 3150,
                                                   4000, 5000, 6300, 8000, 10000])        
    sts.lower_frequencies              = np.array([14,18,22.4,28,35.5,45,56,71,90,112,140,180,224,280,355,450,560,710,\
                                                   900,1120,1400,1800,2240,2800,3550,4500,5600,7100,9000 ])
    sts.upper_frequencies              = np.array([18,22.4,28,35.5,45,56,71,90,112,140,180,224,280,355,450,560,710,900,1120,\
                                                   1400,1800,2240,2800,3550,4500,5600,7100,9000,11200 ])
    sts.harmonics                      = np.arange(1,30)


    sts.broadband_spectrum_resolution        = 301
    sts.floating_point_precision             = np.float32
    sts.urban_canyon_microphone_z_resolution = 16 
    sts.sideline_x_position                  = 0     
    sts.number_of_multiprocessing_workers    = 8
    sts.noise_hemisphere                     = False
    sts.lateral_ground_distance              = 1000 * Units.feet  
    sts.level_ground_microphone_min_x        = -50
    sts.level_ground_microphone_max_x        = 1000
    sts.level_ground_microphone_min_y        = -1000 * Units.feet 
    sts.level_ground_microphone_max_y        = 1000 * Units.feet 
    sts.level_ground_microphone_x_resolution = 16 
    sts.level_ground_microphone_y_resolution = 4      
    return sts 

if __name__ == '__main__': 
    main()  
    plt.show()   
    
