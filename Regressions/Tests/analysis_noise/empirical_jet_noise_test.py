# empirical_jet_noise_test.py
#
# Created: Jan 2024, M. Clarke 

""" setup file for empirical jet noise base on SAE standards 
"""
 
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

import RCAIDE
from RCAIDE.Framework.Core import Units 
from RCAIDE.Library.Plots import *      
from RCAIDE.Library.Methods.Geometry.Planform import wing_planform 

import sys
import matplotlib.pyplot as plt 
import numpy as np     
from copy import deepcopy
import os

# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Embraer_190    import vehicle_setup as vehicle_setup
from Embraer_190    import configs_setup as configs_setup 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():     

    # vehicle data
    vehicle                           = vehicle_setup()
    vehicle.wings.main_wing.control_surfaces.flap.configuration_type = 'triple_slotted'  
    vehicle.wings.main_wing.high_lift = True
    vehicle.wings.main_wing           = wing_planform(vehicle.wings.main_wing)
    
    # Set up configs
    configs           = configs_setup(vehicle) 
    analyses          = analyses_setup(configs)  
    mission           = baseline_mission_setup(analyses)
    basline_missions  = baseline_missions_setup(mission)     
    baseline_results  = basline_missions.base_mission.evaluate()
     
    _   = post_process_noise_data(baseline_results)      
     
    # SPL of rotor check during hover 
    B737_SPL        = np.max(baseline_results.segments.takeoff.conditions.noise.hemisphere_SPL_dBA) 
    B737_SPL_true   = 163.14985051328844 # this value is high because its of a hemisphere of radius 20
    B737_diff_SPL   = np.abs(B737_SPL - B737_SPL_true)
    print('SPL difference: ',B737_diff_SPL)
    assert np.abs((B737_SPL - B737_SPL_true)/B737_SPL_true) < 1e-3
    
    # plot aircraft
    plot_3d_vehicle(vehicle, show_figure=False)
    return

def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 

    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------
    weights = RCAIDE.Framework.Analyses.Weights.Conventional()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    # ------------------------------------------------------------------
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle = vehicle
    aerodynamics.settings.number_of_spanwise_vortices   = 5
    aerodynamics.settings.number_of_chordwise_vortices  = 2     
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Noise Analysis
    # ------------------------------------------------------------------
    noise = RCAIDE.Framework.Analyses.Noise.Correlation_Buildup()   
    noise.vehicle = vehicle          
    analyses.append(noise)

    # ------------------------------------------------------------------
    #  Energy
    # ------------------------------------------------------------------
    energy= RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle  = vehicle 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    # ------------------------------------------------------------------
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    # ------------------------------------------------------------------
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   
 
    return analyses   
# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses 
 

def baseline_mission_setup(analyses): 
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------ 
    mission      = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag  = 'base_mission' 
    Segments     = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 
    base_segment.state.numerics.number_of_control_points    = 3

    # -------------------   -----------------------------------------------
    #   Mission for Landing Noise
    # ------------------------------------------------------------------     
    segment                                               = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag                                           = "descent"
    segment.analyses.extend(analyses.base )   
    segment.altitude_start                                = 120.5
    segment.altitude_end                                  = 0.
    segment.air_speed                                     = 67. * Units['m/s']
    segment.descent_angle                                 = 3.0   * Units.degrees   
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment) 

    # ------------------------------------------------------------------
    #   First Climb Segment: constant Mach, constant segment angle 
    # ------------------------------------------------------------------  
    segment                                                   = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag                                               = "takeoff"    
    segment.analyses.extend(analyses.takeoff )     
    segment.altitude_start                                    = 0 *  Units.meter
    segment.altitude_end                                      = 304.8 * Units.meter
    segment.air_speed                                         = 100* Units['m/s']
    segment.throttle                                          = 1.    
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                           = True  
    segment.flight_dynamics.force_z                           = True     
    
    # define flight controls 
    segment.assigned_control_variables.wind_angle.active                 = True     
    segment.assigned_control_variables.wind_angle.initial_guess_values   = [[ 1.0 * Units.deg]] 
    segment.assigned_control_variables.body_angle.active                 = True        
    segment.assigned_control_variables.body_angle.initial_guess_values   = [[ 5.0 * Units.deg]]
     
    mission.append_segment(segment)

    # ------------------------------------------------------------------  
    # Cutback Segment: Constant speed, constant segment angle
    # ------------------------------------------------------------------   
    segment                                              = Segments.Climb.Constant_Speed_Constant_Angle(base_segment)
    segment.tag                                          = "cutback"     
    segment.analyses.extend(analyses.cutback )
    segment.air_speed                                    = 100 * Units['m/s'] 
    segment.altitude_end                                 = 1. * Units.km    
    segment.climb_angle                                  = 5  * Units.degrees
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
       
    mission.append_segment(segment)   

    # ------------------------------------------------------------------
    #   First Climb Segment: constant Mach, constant segment angle 
    # ------------------------------------------------------------------      
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1" 
    segment.analyses.extend( analyses.cruise )   
    segment.altitude_start                                = 1. * Units.km    
    segment.altitude_end                                  = 2.0   * Units.km
    segment.air_speed                                     = 125.0 * Units['m/s']
    segment.climb_rate                                    = 6.0   * Units['m/s']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]  
    segment.assigned_control_variables.body_angle.active             = True                 
       
    mission.append_segment(segment) 

    return mission 
  

def baseline_missions_setup(base_mission):

    # the mission container
    missions     = RCAIDE.Framework.Mission.Missions() 

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------ 
    base_mission.tag  = 'base_mission'
    missions.append(base_mission)
 
    return missions   

if __name__ == '__main__': 
    main()
    plt.show()
