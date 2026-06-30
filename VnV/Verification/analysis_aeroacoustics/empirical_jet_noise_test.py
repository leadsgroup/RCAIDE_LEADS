# empirical_jet_noise_test.py
#
# Created: Jan 2024, M. Clarke 

""" setup file for empirical jet noise base on SAE standards 
"""
 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from RCAIDE.Framework.Core import Units, Data   
from RCAIDE.Library.Plots import *  
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Airframe.landing_gear_noise_model import compute_landing_gear_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Airframe.flap_noise_model import flap_noise_model
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Airframe.slat_noise_model import slat_noise
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.compute_fan_noise import compute_fan_noise
from RCAIDE.Framework.Mission.Common                                              import Results  
from RCAIDE.Framework.Mission.Segments.Segment                                    import Segment 
from RCAIDE.Framework.Mission.Common                                              import Conditions 
from RCAIDE.Library.Plots import * 
 
# Python Imports  
import sys
import matplotlib.pyplot as plt 
import numpy as np     
from copy import deepcopy
import os

# local imports 
base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
 
# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():  
    # define plotting parameters 
    PP = plot_parameters()  

    # landing gear noise validation
    Landing_Gear_Validation(PP) 
    
    return  
    
    
# ------------------------------------------------------------------ 
# Harmonic Noise Validation
# ------------------------------------------------------------------  
def Landing_Gear_Validation(PP): 
    
    # define aircraft properties 
    '''gear_params: dict
    - num_wheels (Nw): Number of wheels
    - wheel_diam (d): Wheel diameter [inches]
    - wheel_width (w): Wheel width [inches]
    - strut_lengths (L_j): List of lengths of struts [inches]
    - strut_dims (dim_j): List of diameters/widths of struts [inches]
    - aircraft_weight (W_ac): Max Takeoff Weight [lbs]
    - track_angle (gamma): Wheel track alignment angle [degrees]
    
flight_params: dict
    - M_flight: Flight Mach number
    - theta: Emission angle [degrees] (90 is overhead)
    - R: Distance to observer [ft]
    - c0: Speed of sound [ft/s] (default 1116)
    - rho0: Air density [slugs/ft^3] (default 0.00237)'''

    #define params for landing gear model
    D = 1.016 #m
    H = 8.0518 #m
    wheels= 2
    Weight = 68038.8555 #kg
    strut_diameter=0.11811#m
    theta =(np.pi)/2 #deg 
    frequency = np.logspace(1.5, 4, 100)
    W = 0.3556#m
    
    #define params for flap model
    thickness = 0.1#m
    cf = 0.9#m
    deltaf = np.radians(37.5)

    #define params for slat model
    phi= 0
    Ls = 0.08128
    gamma_s = np.radians(20)
    sigma_s =  np.radians(25)
    alpha =  np.radians(30)

 # define operating conditions                                            
    a                       = 343.376
    T                       = 288.16889478  
    density                 = 1.2250	
    dynamic_viscosity       = 1.81E-5   
    ctrl_pts                = 1
    AoA                     = 4
    U = 103 #aircraft velocity
    M = 0.2 #mach number
    frequency_flp = np.logspace(1, 4, 100)

    #define params for core noise model
    #------------------------------------------------------------------------------------------------------------------------------------
    # Propulsor: Starboard Propulsor
    #------------------------------------------------------------------------------------------------------------------------------------
    turbofan                                    = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()
    turbofan.tag                                = 'starboard_propulsor'
    turbofan.bypass_ratio                       = 5.4
    turbofan.design_altitude                    = 35000.0*Units.ft
    turbofan.design_mach_number                 = 0.78
    turbofan.design_thrust                      = 35000.0* Units.N             

    # fan
    fan                                         = RCAIDE.Library.Components.Powertrain.Converters.Fan()
    fan.tag                                     = 'fan'
    fan.polytropic_efficiency                   = 0.93
    fan.pressure_ratio                          = 1.7
    turbofan.fan                                = fan

    # working fluid
    turbofan.working_fluid                      = RCAIDE.Library.Attributes.Gases.Air()
    ram                                         = RCAIDE.Library.Components.Powertrain.Converters.Ram()
    ram.tag                                     = 'ram'
    turbofan.ram                                = ram

    m = None #mass flow rate


    # define microphone locations
    microphone_locations = np.zeros((1,3))   

    # define segment 
    segment                                                = Segment()  
    conditions                                             = Results() 
    conditions.aeroacoustics.relative_microphone_locations = np.repeat(microphone_locations[ np.newaxis,:,: ],1,axis=0)   
    conditions.aerodynamics.angles.alpha                   = np.atleast_2d(AoA).T 
    conditions.freestream.density                          = np.ones((ctrl_pts,1)) * density
    conditions.freestream.dynamic_viscosity                = np.ones((ctrl_pts,1)) * dynamic_viscosity   
    conditions.freestream.speed_of_sound                   = np.ones((ctrl_pts,1)) * a 
    conditions.freestream.temperature                      = np.ones((ctrl_pts,1)) * T
    conditions.freestream.pressure                         = 97717 #pa at 300m
    conditions.freestream.velocity                         = 85 
    conditions.frames.inertial.velocity_vector             = np.array([[U, 0. ,0.]]) 
    conditions.freestream.mach_number                      = np.atleast_2d(np.linalg.norm(conditions.frames.inertial.velocity_vector,axis = 1)).T/ a
    conditions.frames.planet.true_course                   = np.zeros((ctrl_pts,3,3)) 
    conditions.frames.planet.true_course[:,2,2]            = 1 
    conditions.frames.wind.transform_to_inertial           = np.zeros((ctrl_pts,3,3))    
    conditions.frames.body.transform_to_inertial           = np.zeros((ctrl_pts,3,3))
    conditions.frames.body.transform_to_inertial[:,0,0]    = np.cos(AoA)
    conditions.frames.body.transform_to_inertial[:,0,2]    = np.sin(AoA)
    conditions.frames.body.transform_to_inertial[:,1,1]    = 1
    conditions.frames.body.transform_to_inertial[:,2,0]    = -np.sin(AoA)
    conditions.frames.body.transform_to_inertial[:,2,2]    = np.cos(AoA)     

    segment.state.conditions                                 = conditions 


    turbofan.append_operating_conditions(segment, segment.state.conditions.energy,segment.state.conditions.aeroacoustics)
 
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.angular_velocity = 4200
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_velocity = 416
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_stagnation_temperature = T+80
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_stagnation_pressure = 152*1000
    
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.number_of_blades = 54
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.diameter = 70 / Units.inches

    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.static_temperature_output = T + 80
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.static_temperature_input  = T
    
      
    segment.state.conditions.expand_rows(ctrl_pts)   
           
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Get Raw Data
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
   
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Run simulation  
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
     

    results = compute_landing_gear_noise(microphone_locations, D, H, W, wheels, M, Weight, strut_diameter, theta, frequency, segment)

    # plt.figure(figsize=(10, 6))
    # plt.semilogx(results['Freq'], results['Total'], 'k-', linewidth=2, label='Total Noise')
    # plt.semilogx(results['Freq'], results['Low'], 'r--', label='Low Freq (Wheels)')
    # plt.semilogx(results['Freq'], results['Mid'], 'g--', label='Mid Freq (Struts)')
    # plt.semilogx(results['Freq'], results['High'], 'b--', label='High Freq (Details)')
    # plt.xlabel('Frequency (Hz)')
    # plt.ylabel('SPL (dB)')
    # plt.title(f'Landing Gear Noise Prediction (M={0.2}, $\\theta=90^\circ$)')
    # plt.legend()
    # plt.grid(True, which="both", alpha=0.5)
    # plt.show()
    
    comp_li = flap_noise_model(microphone_locations,cf,thickness, deltaf, theta, frequency_flp,segment)
    # print(comp_li)
    # fig, ax = plt.subplots(figsize=(8, 5))
    # ax.plot(frequency_flp,comp_li[0],label='HF Curve')
    # # Logarithmic X-axis
    # ax.set_xscale('log')

    # plt.ylabel('SPL dB')
    # plt.xlabel('Freq. Hz')
    # plt.ylim([-85,0])
    # plt.tight_layout()
    # plt.legend()
    # plt.show()

    slat_noise1 = slat_noise(microphone_locations, phi, theta, Ls, gamma_s, sigma_s, alpha, segment, frequency, A=1e-5)
    # print(slat_noise1)
    # fig, ax = plt.subplots(figsize=(8, 5))
    # ax.plot(frequency_flp,slat_noise1[0],label='HF Curve')
    # # # Logarithmic X-axis
    # ax.set_xscale('log')
    # plt.xlim([10**2,10**4])

    # plt.legend()
    # plt.show()

    core_noise1 = compute_fan_noise(microphone_locations, turbofan,m, segment.state.conditions.aeroacoustics, segment, frequency)
    print(core_noise1)
    
    return  

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
 
if __name__ == '__main__': 
    main()  
    plt.show()   
    
   

#     # vehicle data
#     vehicle                           = vehicle_setup()
#     vehicle.wings.main_wing.control_surfaces.flap.configuration_type = 'triple_slotted'  
#     vehicle.wings.main_wing.high_lift = True
    
#     # Set up configs
#     configs           = configs_setup(vehicle) 
#     analyses          = analyses_setup(configs)  
#     mission           = baseline_mission_setup(analyses)
#     basline_missions  = baseline_missions_setup(mission)     
#     baseline_results  = basline_missions.base_mission.evaluate()
     
#     _   = post_process_noise_data(baseline_results,compute_PNL=True )      
     
#     # SPL of rotor check during hover 
#     E190_SPL        = np.max(baseline_results.segments.takeoff.conditions.aeroacoustics.hemisphere_SPL_dBA)
#     E190_SPL_true   = 124.19217253485145 # this value is high because its of a hemisphere of radius 20
#     E190_diff_SPL   = np.abs(E190_SPL - E190_SPL_true)
#     print('SPL difference: ',E190_diff_SPL)
#     assert np.abs((E190_SPL - E190_SPL_true)/E190_SPL_true) < 1e-3 
#     return

# def base_analysis(vehicle):

#     # ------------------------------------------------------------------
#     #   Initialize the Analyses
#     # ------------------------------------------------------------------     
#     analyses = RCAIDE.Framework.Analyses.Vehicle() 
#     analyses.vehicle = vehicle

#     #  Geometry
#     geometry = RCAIDE.Framework.Analyses.Geometry.Geometry() 
#     analyses.append(geometry) 

#     #  Geometry
#     weights = RCAIDE.Framework.Analyses.Weights.Conventional_Transport() 
#     analyses.append(weights)    
 
#     #  Aerodynamics Analysis 
#     aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()     
#     analyses.append(aerodynamics)

#     # ------------------------------------------------------------------
#     #  Noise Analysis
#     # ------------------------------------------------------------------
#     aeroacoustics = RCAIDE.Framework.Analyses.Aeroacoustics.Semi_Empirical()   
#     analyses.append(aeroacoustics)

#     # ------------------------------------------------------------------
#     #  Energy
#     # ------------------------------------------------------------------
#     energy= RCAIDE.Framework.Analyses.Energy.Energy() 
#     analyses.append(energy)

#     # ------------------------------------------------------------------
#     #  Planet Analysis
#     # ------------------------------------------------------------------
#     planet = RCAIDE.Framework.Analyses.Planets.Earth()
#     analyses.append(planet)

#     # ------------------------------------------------------------------
#     #  Atmosphere Analysis
#     # ------------------------------------------------------------------
#     atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
#     analyses.append(atmosphere)   
 
#     return analyses   
# # ----------------------------------------------------------------------
# #   Define the Vehicle Analyses
# # ----------------------------------------------------------------------

# def analyses_setup(configs):

#     analyses = RCAIDE.Framework.Analyses.Analysis.Container()

#     # build a base analysis for each config
#     for tag,config in configs.items():
#         analysis = base_analysis(config)
#         analyses[tag] = analysis

#     return analyses 
 

# def baseline_mission_setup(analyses): 
#     # ------------------------------------------------------------------
#     #   Initialize the Mission
#     # ------------------------------------------------------------------ 
#     mission      = RCAIDE.Framework.Mission.Sequential_Segments()
#     mission.tag  = 'base_mission' 
#     Segments     = RCAIDE.Framework.Mission.Segments 
#     base_segment = Segments.Segment() 
#     base_segment.state.numerics.number_of_control_points    = 3

#     # -------------------   -----------------------------------------------
#     #   Mission for Landing Noise
#     # ------------------------------------------------------------------     
#     segment                                               = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
#     segment.tag                                           = "descent"
#     segment.analyses.extend(analyses.base )   
#     segment.altitude_start                                = 120.5
#     segment.altitude_end                                  = 0.
#     segment.air_speed                                     = 67. * Units['m/s']
#     segment.descent_angle                                 = 3.0   * Units.degrees   
    
#     # define flight dynamics to model 
#     segment.flight_dynamics.force_x                       = True  
#     segment.flight_dynamics.force_z                       = True     
    
#     # define flight controls 
#     segment.assigned_control_variables.throttle.active               = True           
#     segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
#     segment.assigned_control_variables.pitch_angle.active             = True                
    
#     mission.append_segment(segment) 

#     # ------------------------------------------------------------------
#     #   First Climb Segment: constant Mach, constant segment angle 
#     # ------------------------------------------------------------------  
#     segment                                                   = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
#     segment.tag                                               = "takeoff"    
#     segment.analyses.extend(analyses.takeoff )     
#     segment.altitude_start                                    = 0 *  Units.meter
#     segment.altitude_end                                      = 304.8 * Units.meter
#     segment.air_speed                                         = 100* Units['m/s']
#     segment.throttle                                          = 1.
    
#     # define flight dynamics to model 
#     segment.flight_dynamics.force_x                           = True  
#     segment.flight_dynamics.force_z                           = True     
    
#     # define flight controls 
#     segment.assigned_control_variables.angle_of_attack.active                 = True     
#     segment.assigned_control_variables.angle_of_attack.initial_guess_values   = [[ 1.0 * Units.deg]] 
#     segment.assigned_control_variables.pitch_angle.active                 = True        
#     segment.assigned_control_variables.pitch_angle.initial_guess_values   = [[ 5.0 * Units.deg]]
     
#     mission.append_segment(segment)

#     # ------------------------------------------------------------------
#     # Cutback Segment: Constant speed, constant segment angle
#     # ------------------------------------------------------------------
#     segment                                              = Segments.Climb.Constant_Speed_Constant_Angle(base_segment)
#     segment.tag                                          = "cutback"
#     segment.analyses.extend(analyses.cutback )
#     segment.air_speed                                    = 100 * Units['m/s']
#     segment.altitude_end                                 = 1. * Units.km
#     segment.climb_angle                                  = 5  * Units.degrees

#     # define flight dynamics to model
#     segment.flight_dynamics.force_x                      = True
#     segment.flight_dynamics.force_z                      = True

#     # define flight controls
#     segment.assigned_control_variables.throttle.active               = True
#     segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]
#     segment.assigned_control_variables.pitch_angle.active             = True

#     mission.append_segment(segment)

#     # ------------------------------------------------------------------
#     #   First Climb Segment: constant Mach, constant segment angle 
#     # ------------------------------------------------------------------      
#     segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
#     segment.tag = "climb_1" 
#     segment.analyses.extend( analyses.cruise )   
#     segment.altitude_start                                = 1. * Units.km    
#     segment.altitude_end                                  = 2.0   * Units.km
#     segment.air_speed                                     = 125.0 * Units['m/s']
#     segment.climb_rate                                    = 6.0   * Units['m/s']  
    
#     # define flight dynamics to model 
#     segment.flight_dynamics.force_x                       = True  
#     segment.flight_dynamics.force_z                       = True     
    
#     # define flight controls 
#     segment.assigned_control_variables.throttle.active               = True           
#     segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]  
#     segment.assigned_control_variables.pitch_angle.active             = True                 
       
#     mission.append_segment(segment) 

#     return mission 
  

# def baseline_missions_setup(base_mission):

#     # the mission container
#     missions     = RCAIDE.Framework.Mission.Missions() 

#     # ------------------------------------------------------------------
#     #   Base Mission
#     # ------------------------------------------------------------------ 
#     base_mission.tag  = 'base_mission'
#     missions.append(base_mission)
 
#     return missions   

# if __name__ == '__main__': 
#     main()
#     plt.show()
