# digital_elevation_and_noise_hemispheres_test.py
#
# Created: Dec 2023 M. Clarke  
 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE Imports 
import RCAIDE
from RCAIDE.Framework.Core import Units , Data 
from RCAIDE.Library.Plots import *

# Python imports
import matplotlib.pyplot as plt  
import sys 
import os
import numpy as np     

sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from NASA_X57    import vehicle_setup, configs_setup     

# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main(): 

    current_dir = os.path.dirname(__file__)
    data_file = os.path.join(current_dir, 'LA_Metropolitan_Area.txt')


    plot_elevation_contours(topography_file   =data_file,use_lat_long_coordinates = True, save_filename = "Elevation_Contours_Lat_Long")

    plot_elevation_contours(topography_file   =data_file,use_lat_long_coordinates = False, save_filename = "Elevation_Contours_XY")
    
    vehicle  = vehicle_setup('Blade_Element_Momentum_Theory_Helmholtz')          
    configs  = configs_setup(vehicle) 
    analyses = analyses_setup(configs)  
    mission  = mission_setup(analyses)
    missions = missions_setup(mission)  
    results  = missions.base_mission.evaluate()  
    results.segments[0].analyses.noise.settings.topography_file = data_file
    
    regression_plotting_flag = False 
    flight_times = np.array(['06:00:00','06:05:00','06:10:00','06:15:00',
                             '06:20:00','06:25:00','06:30:00','06:35:00',
                             '06:40:00','06:45:00','06:50:00','06:55:00',
                             '07:00:00','07:05:00','07:10:00','07:15:00',
                             '07:20:00','07:25:00','07:30:00','07:35:00',
                             '07:40:00','07:45:00','07:50:00','07:55:00', 
                             '08:00:00'])

    noise_data   = post_process_noise_data(results,
                                           flight_times = flight_times,  
                                           time_period  = ['06:00:00','09:00:00'] )  

    
    plot_results(results,noise_data,regression_plotting_flag) 

    X57_SPL        = np.max(results.segments.cruise.conditions.noise.hemisphere_SPL_dBA) 
    X57_SPL_true   = 96.64761095865973
    X57_diff_SPL   = np.abs(X57_SPL - X57_SPL_true)
    print('Error: ',X57_diff_SPL)
    assert np.abs((X57_SPL - X57_SPL_true)/X57_SPL_true) < 1e-3 
     
    return      

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ---------------------------------------------------------------------- 
def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis      = base_analysis(config) 
        analyses[tag] = analysis

    return analyses  


def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
 
    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Electric()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle  = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)   
 
    #  Noise Analysis   
    noise = RCAIDE.Framework.Analyses.Noise.Frequency_Domain_Buildup()   
    noise.vehicle = vehicle
    noise.settings.mean_sea_level_altitude          = False         
    noise.settings.aircraft_origin_coordinates      = [33.94067953101678, -118.40513722978149]# Los Angeles International Airport
    noise.settings.aircraft_destination_coordinates = [33.8146, -118.1459]  # Ontario International airport 
    noise.settings.microphone_x_resolution          = 800  
    noise.settings.microphone_y_resolution          = 1200  
    noise.settings.noise_times_steps                = 50  
    noise.settings.number_of_microphone_in_stencil  = 25
    noise.settings.topography_file                  = 'LA_Metropolitan_Area.txt' 
    analyses.append(noise)

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle  = vehicle 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    # done!
    return analyses    

# ----------------------------------------------------------------------
#  Set Up Mission 
# ---------------------------------------------------------------------- 
def mission_setup(analyses):      
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission       = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag   = 'mission' 
    Segments      = RCAIDE.Framework.Mission.Segments  
    base_segment  = Segments.Segment()   
    base_segment.state.numerics.number_of_control_points  = 3
    base_segment.state.numerics.discretization_method     = RCAIDE.Library.Methods.Utilities.Chebyshev.linear_data
    
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment) 
    segment.tag = "cruise"   
    segment.analyses.extend( analyses.base ) 
    segment.initial_battery_state_of_charge              = 1.0       
    segment.altitude                                     = 30
    segment.air_speed                                    = 100
    segment.distance                                     = 1 * Units.miles
    segment.true_course                                  = 130 *Units.degrees 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
       
    mission.append_segment(segment)      
     
    return mission

# ----------------------------------------------------------------------
#  Set Up Missions 
# ---------------------------------------------------------------------- 
def missions_setup(mission): 
 
    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions  


# ----------------------------------------------------------------------
#  Plot Resuls 
# ---------------------------------------------------------------------- 
def plot_results(results,noise_data,regression_plotting_flag):  
    plot_noise_level(noise_data,
                    noise_level  = noise_data.SPL_dBA[0], 
                    save_filename="Sideline_Noise_Levels")  
    
    # Maximum Sound Pressure Level   
    plot_3D_noise_contour(noise_data,
                          noise_level      = np.max(noise_data.SPL_dBA,axis=0), 
                          min_noise_level  = 20,  
                          max_noise_level  = 90, 
                          noise_scale_label= 'SPL [dBA]',
                          save_filename    = "SPL_max_Noise_3D_Contour",
                          show_figure      = regression_plotting_flag)   
                        

    # Day Night Average Noise Level 
    plot_3D_noise_contour(noise_data,
                        noise_level      = noise_data.L_dn,
                        min_noise_level  = 0,  
                        max_noise_level  = 90, 
                        noise_scale_label= 'L_dn',
                        show_microphones = True, 
                        save_filename    = "L_dn_Noise_3D_Contour",
                        show_figure      = regression_plotting_flag) 
    

    # Equivalent Noise Level
    plot_3D_noise_contour(noise_data,
                        noise_level      = noise_data.L_eq,
                        min_noise_level  = 0,  
                        max_noise_level  = 90, 
                        noise_scale_label= 'L_eq',
                        show_trajectory  = True,
                        save_filename    = "L_eq_Noise_3D_Contour",
                        show_figure      = regression_plotting_flag)    
    

    # 24-hr Equivalent Noise Level
    plot_3D_noise_contour(noise_data,
                       noise_level      = noise_data.L_eq,
                       min_noise_level  = 0,  
                       max_noise_level  = 90, 
                       noise_scale_label= '24hr-L_eq',
                       save_filename    = "24hr_L_eq_Noise_3D_Contour", 
                       use_lat_long_coordinates = False,                         
                       show_figure      = regression_plotting_flag)      
    

    # Single Event Noise Exposure Level
    plot_3D_noise_contour(noise_data,
                       noise_level      = noise_data.SENEL,
                       min_noise_level  = 0,  
                       max_noise_level  = 90, 
                       noise_scale_label= 'SENEL',
                       save_filename    = "SENEL_Noise_3D_Contour",
                       show_figure      = regression_plotting_flag)
    
    # Maximum Sound Pressure Level   
    plot_2D_noise_contour(noise_data,
                        noise_level      = np.max(noise_data.SPL_dBA,axis=0), 
                        min_noise_level  = 0,  
                        max_noise_level  = 90, 
                        noise_scale_label= 'SPL [dBA]',
                        save_filename    = "SPL_max_Noise_2D_Contour",
                        show_elevation   = True,
                        use_lat_long_coordinates= False,
                        show_figure      = regression_plotting_flag)   
                        

    # Day Night Average Noise Level 
    plot_2D_noise_contour(noise_data,
                        noise_level      = noise_data.L_dn,
                        min_noise_level  = 0,  
                        max_noise_level  = 90, 
                        noise_scale_label= 'L_dn',
                        save_filename    = "L_dn_Noise_2D_Contour",
                        show_figure      = regression_plotting_flag) 
    

    # Equivalent Noise Level
    plot_2D_noise_contour(noise_data,
                        noise_level      = noise_data.L_eq,
                        min_noise_level  = 0,  
                        max_noise_level  = 90, 
                        noise_scale_label= 'L_eq',
                        save_filename    = "L_eq_Noise_2D_Contour",
                        show_figure      = regression_plotting_flag)    
    

    # 24-hr Equivalent Noise Level
    plot_2D_noise_contour(noise_data,
                       noise_level      = noise_data.L_eq,
                       min_noise_level  = 20,  
                       max_noise_level  = 90, 
                       noise_scale_label= '24hr-L_eq',
                       save_filename    = "24hr_L_eq_Noise_2D_Contour",
                       show_figure      = regression_plotting_flag)      
    

    # Single Event Noise Exposure Level
    plot_2D_noise_contour(noise_data,
                       noise_level      = noise_data.SENEL,
                       min_noise_level  = 20,  
                       max_noise_level  = 90, 
                       noise_scale_label= 'SENEL',
                       save_filename    = "SENEL_Noise_2D_Contour",
                       show_figure      = regression_plotting_flag)      
    return  

if __name__ == '__main__': 
    main()    
    plt.show()
