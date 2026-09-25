# noise_certification_test.py
#
# Created: Apr 2025, M. Clarke  
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units , Data
from RCAIDE.Library.Plots import *
from RCAIDE.Library.Methods.Performance.compute_noise_certification_metrics import  compute_noise_certification_metrics

import sys
import matplotlib.pyplot as plt 
import numpy as np      
import os

# local imports 
base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Embraer_190    import vehicle_setup as vehicle_setup
from Embraer_190    import configs_setup as configs_setup 
import time

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(): 
    ti = time.time()
    vehicle           = vehicle_setup() 
    configs           = configs_setup(vehicle) 
    analyses          = noise_analyses_setup(configs)  
    approach_mission  = approach_mission_setup(analyses)
    takeoff_mission   = takeoff_mission_setup(analyses)  
     
    results = compute_noise_certification_metrics(approach_mission = approach_mission, takeoff_mission=takeoff_mission)
    plot_noise_certification_contour(results)

    # truth_approach_noise_2000m  = 100.2084876866911
    # truth_flyover_noise_6000m   = 92.57084988685932
    # truth_sideline_noise_450m   = 108.8302490908947

    # # Check the errors
    # error = Data()
    # error.approach_noise_2000m   = abs( truth_approach_noise_2000m - results.approach_noise_2000m)/results.approach_noise_2000m
    # error.flyover_noise_6000m    = abs( truth_flyover_noise_6000m  - results.flyover_noise_6000m )/results.flyover_noise_6000m 
    # error.sideline_noise_450m    = abs( truth_sideline_noise_450m  - results.sideline_noise_450m )/results.sideline_noise_450m  

    # print('Errors:')
    # print(error)

    # for k,v in list(error.items()):
    #     assert(np.abs(v)<1e-2)


    # elapsed_time = time.time() - ti
    # elapsed_time_min = elapsed_time / 60
    # print('Elapsed time (min): ', elapsed_time_min)
    return 
 

# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# ############################################################################################################################################################################
# ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def noise_analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = noise_base_analysis(config)
        analyses[tag] = analysis

    return analyses 


def noise_base_analysis(vehicle):
    """This is the baseline set of analyses to be used with this vehicle. Of these, the most
    commonly changed are the weights and aerodynamics methods."""

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle

    # ------------------------------------------------------------------
    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_Transport()
    analyses.append(weights)
    
    # ------------------------------------------------------------------
    #  Aerodynamics  
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)
    
    # ------------------------------------------------------------------
    #  Acoustics Analysis
    # ------------------------------------------------------------------
    aeroacoustics = RCAIDE.Framework.Analyses.Aeroacoustics.Semi_Empirical()  
    analyses.append(aeroacoustics)
 
    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere) 
     

    return analyses


def approach_mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 
    base_segment.state.numerics.number_of_control_points = 40 
 

    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "final_approach"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_start                                           = 1000.0 * Units.ft
    segment.altitude_end                                             = 10.0   * Units.ft
    segment.air_speed                                                = 135.0  * Units['knots']
    segment.descent_angle                                            = 3.  * Units.deg                               
           
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.pitch_angle.active             = True                

    mission.append_segment(segment)
 
 
    return mission

def takeoff_mission_setup(analyses):
    """Certification-style takeoff: full-thrust ground run to lift-off, climb to the 35 ft screen
    height, accelerating initial climb, then a fixed-throttle cutback climb past the flyover
    microphone. Speeds are continuous between segments."""
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points = 40

    # ------------------------------------------------------------------
    #   Ground Run: full takeoff thrust to lift-off speed
    # ------------------------------------------------------------------
    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff_Ground_Run" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start                                           = 20.0  * Units['knots']
    segment.velocity_end                                             = 140.0 * Units['knots']
    segment.friction_coefficient                                     = 0.03
    segment.altitude                                                 = 0.0   
    segment.throttle                                                 = 1.0
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Screen Climb: constant speed to 35 ft
    # ------------------------------------------------------------------
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Takeoff_Climb" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_start                                           = 0.0   * Units['ft']
    segment.altitude_end                                             = 35.0  * Units['ft']
    segment.air_speed                                                = 140.0 * Units['knots']
    segment.climb_rate                                               = 1500  * Units['fpm']
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.pitch_angle.active            = True                 
    mission.append_segment(segment) 

    # ------------------------------------------------------------------
    #   Initial Climb: accelerate to climb speed while climbing to 1000 ft
    # ------------------------------------------------------------------
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Inital_Climb" 
    segment.analyses.extend( analyses.takeoff )  
    segment.altitude_end                                             = 1000.0 * Units['ft']
    segment.air_speed_start                                          = 140.0  * Units['knots']
    segment.air_speed_end                                            = 160.0  * Units['knots']
    segment.climb_rate                                               = 2000   * Units['fpm']
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.pitch_angle.active            = True                 
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Cutback Climb: fixed reduced throttle past the flyover microphone
    # ------------------------------------------------------------------
    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "Cutback_Climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.altitude_end                                             = 3000.0 * Units['ft']
    segment.air_speed                                                = 160.0  * Units['knots']
    segment.throttle                                                 = 0.85
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    segment.assigned_control_variables.angle_of_attack.active        = True
    segment.assigned_control_variables.pitch_angle.active            = True                 
    mission.append_segment(segment)

    return mission

if __name__ == '__main__': 
    main()    
    plt.show()