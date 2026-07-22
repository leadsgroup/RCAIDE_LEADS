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
from RCAIDE.Library.Methods.Aeroacoustics.Semi_Empirical.Propulsion.Engine_Noise.compute_core_noise import compute_core_noise
from RCAIDE.Framework.Mission.Common                                              import Results  
from RCAIDE.Framework.Mission.Segments.Segment                                    import Segment 
from RCAIDE.Framework.Mission.Common                                              import Conditions 
from RCAIDE.Library.Methods.Aeroacoustics.Common   import SPL_arithmetic 
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
    frequency = np.logspace(1.5, 4.5, 100)
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
    alpha =  np.radians(10)

    #define param for core noise model
    pr = 13.1
    


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

    #combustor
    combustor                                         = RCAIDE.Library.Components.Powertrain.Converters.Combustor()
    combustor.tag                                     = 'combustor'
    combustor.number_of_fuel_nozzle                   = 18
    combustor.diameter                                = 0.6858
    turbofan.combustor                                = combustor

    # core nozzle
    core_nozzle                                    = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    core_nozzle.tag                                = 'core nozzle'
    core_nozzle.polytropic_efficiency              = 0.98                    
    core_nozzle.pressure_ratio                     = 0.995 
    core_nozzle.diameter                           = 0.38118288
    turbofan.core_nozzle                           = core_nozzle
             
    # fan nozzle             
    fan_nozzle                                     = RCAIDE.Library.Components.Powertrain.Converters.Expansion_Nozzle()   
    fan_nozzle.tag                                 = 'fan nozzle'
    fan_nozzle.polytropic_efficiency               = 0.98                    
    fan_nozzle.pressure_ratio                      = 0.995
    turbofan.fan_nozzle                            = fan_nozzle 

    m = None #mass flow rate


    # define microphone locations
    microphone_locations = np.zeros((1,3))   
    microphone_locations = np.array([[80,80,80]])
    print('setting distance:', np.linalg.norm(microphone_locations, axis=1))

    # define segment 
    segment                                                = Segment()  
    conditions                                             = Results()  
    conditions.aerodynamics.angles.alpha                   = alpha
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
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_velocity = 350 * Units.mph
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_stagnation_temperature = 440
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.exit_stagnation_pressure = 152*1000
    
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.number_of_blades = 22
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.diameter = 70*Units.inches

    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.static_temperature_output = T + (80/1.8)
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan.static_temperature_input  = T

    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan_nozzle.exit_velocity = 280.0
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan_nozzle.exit_stagnation_temperature = 340.0
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].fan_nozzle.exit_stagnation_pressure = 2611.8 

    # Core Nozzle (Primary) Parameters - Realistic for CFM56
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].core_nozzle.exit_velocity = 400.0
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].core_nozzle.exit_stagnation_temperature = 800.0
    segment.state.conditions.aeroacoustics.propulsors[turbofan.tag].core_nozzle.exit_stagnation_pressure = 165000.0
        
    segment.state.conditions.energy.converters['combustor'].inputs.static_temperature = 622.7
    segment.state.conditions.energy.converters['combustor'].outputs.static_temperature = 1000
    
      
    segment.state.conditions.expand_rows(ctrl_pts)   
           
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Get Raw Validation Data
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- 
    import csv
    def read_noise_data(file_path):
        frequency_hz = []
        total_SPL = []
        
        with open(file_path, mode='r') as file:
            reader = csv.reader(file)
            
            # Skip the header row ("Frequency (Hz),Total SPL (dB)")
            next(reader)
            
            # Extract and convert the data
            for row in reader:
                if row: # Check to ensure the row isn't empty
                    frequency_hz.append(float(row[0]))
                    total_SPL.append(float(row[1]))
                    
        return frequency_hz, total_SPL
    # -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    # Run simulation  
    # ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
     

    lg_noise1 = compute_landing_gear_noise(microphone_locations, D, H, W, wheels, M, Weight, strut_diameter, theta, frequency, segment)
    validation_lg = read_noise_data('/Users/siripunn/Desktop/LEADS_WORK/RESEARCH/05_Aeroacoustics/Boeing_Method/LG_Noise/b737_gear_noise_data.csv')
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
    
    flap_noise1 = flap_noise_model(microphone_locations,cf,thickness, deltaf, theta, frequency,segment)
    validation_flap = read_noise_data('/Users/siripunn/Desktop/LEADS_WORK/RESEARCH/05_Aeroacoustics/Boeing_Method/Flap_Side_Edge_Noise/b737_flap_noise_data.csv')
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
    #validation_slat = read_noise_data('/Users/siripunn/Desktop/LEADS_WORK/RESEARCH/05_Aeroacoustics/Boeing_Method/Core Noise/b737_core_noise_data.csv')
    # print(slat_noise1)
    # fig, ax = plt.subplots(figsize=(8, 5))
    # ax.plot(frequency_flp,slat_noise1[0],label='HF Curve')
    # # # Logarithmic X-axis
    # ax.set_xscale('log')
    # plt.xlim([10**2,10**4])

    # plt.legend()
    # plt.show()


    fan_noise1 = compute_fan_noise(microphone_locations, turbofan,m, segment.state.conditions.aeroacoustics, segment, frequency)
    validation_fan = read_noise_data('/Users/siripunn/Desktop/LEADS_WORK/RESEARCH/05_Aeroacoustics/Boeing_Method/Fan Noise/b737_fan_noise_data.csv/b737_fan_noise_data.csv')
    core_noise1 = compute_core_noise(microphone_locations, turbofan, pr, segment.state.conditions.aeroacoustics, segment, frequency)
    validation_core = read_noise_data('/Users/siripunn/Desktop/LEADS_WORK/RESEARCH/05_Aeroacoustics/Boeing_Method/Core Noise/b737_core_noise_data.csv')
    # print(core_noise1.SPL_1_3_spectrum)
    # fig, ax = plt.subplots(figsize=(8, 5))
    # ax.plot(frequency,core_noise1.SPL_1_3_spectrum[0][0],label='HF Curve')
    # ax.set_xscale('log')

    #flap slat noise is current neg. db
    SPL_total = np.concatenate((lg_noise1.Total,flap_noise1[0],slat_noise1[0],fan_noise1.SPL_1_3_spectrum[0][0],core_noise1.SPL_1_3_spectrum[0][0]), axis=0)
    twodim = np.atleast_2d(SPL_total)
    total_SPL_classic = SPL_arithmetic(np.array([lg_noise1.Total,flap_noise1[0],slat_noise1[0],fan_noise1.SPL_1_3_spectrum[0][0],core_noise1.SPL_1_3_spectrum[0][0]]),sum_axis = 0)
    fig, ax = plt.subplots(figsize=(8, 5))
    #ax.plot(validation_lg[0],validation_lg[1], 'bo')
    #ax.plot(validation_flap[0],validation_flap[1],'bo')
    #ax.plot(validation_fan[0],validation_fan[1],'bo')
    #ax.plot(validation_core[0],validation_core[1],'bo')
    ax.plot(frequency,total_SPL_classic,label='Total Noise')
    ax.plot(frequency,core_noise1.SPL_1_3_spectrum[0][0],label='Core Noise')
    ax.plot(frequency,fan_noise1.SPL_1_3_spectrum[0][0],label='Fan Noise')
    ax.plot(frequency,slat_noise1[0],label='Slat Noise')
    ax.plot(frequency,flap_noise1[0],label='Flap Noise')
    ax.plot(frequency,lg_noise1.Total,label='Landing Gear Noise')
    ax.set_xscale('log')
    plt.legend()
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
