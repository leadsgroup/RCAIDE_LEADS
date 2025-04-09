# noise_certification_test.py
#
# Created: Apr 2025, M. Clarke  
# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units , Data
from RCAIDE.Library.Plots import *
from RCAIDE.Library.Methods.Performance.compute_noise_certification_data import  compute_noise_certification_data

import sys
import matplotlib.pyplot as plt 
import numpy as np      
import os

# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Embraer_190    import vehicle_setup as vehicle_setup
from Embraer_190    import configs_setup as configs_setup 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(): 
    vehicle           = vehicle_setup() 
    configs           = configs_setup(vehicle) 
    analyses          = noise_analyses_setup(configs)  
    approach_mission  = approach_mission_setup(analyses)
    takeoff_mission   = takeoff_mission_setup(analyses)  
     
    results =  compute_noise_certification_data(approach_mission = approach_mission, takeoff_mission=takeoff_mission) 

    truth_approach_noise_2000m  = 84.73294804063438
    truth_flyover_noise_6000m   = 83.29878462609882
    truth_sideline_noise_450m   = 95.24007857683979
    truth_area_65_dbA           = 98.58686616791356
    truth_area_85_dbA           = 41.174840676087555

    # Check the errors
    error = Data()
    error.approach_noise_2000m   = abs( truth_approach_noise_2000m - results.approach_noise_2000m)/results.approach_noise_2000m
    error.flyover_noise_6000m    = abs( truth_flyover_noise_6000m  - results.flyover_noise_6000m )/results.flyover_noise_6000m 
    error.sideline_noise_450m    = abs( truth_sideline_noise_450m  - results.sideline_noise_450m )/results.sideline_noise_450m 
    error.area_65_dbA            = abs( truth_area_65_dbA          - results.area_65_dbA         )/results.area_65_dbA         
    error.area_85_dbA            = abs( truth_area_85_dbA          - results.area_85_dbA         )/results.area_85_dbA

    print('Errors:')
    print(error)

    for k,v in list(error.items()):
        assert(np.abs(v)<1e-5)

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

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.vehicle = vehicle
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle = vehicle 
    analyses.append(energy)
    
    # ------------------------------------------------------------------
    #  Noise Analysis
    # ------------------------------------------------------------------
    noise = RCAIDE.Framework.Analyses.Noise.Correlation_Buildup()   
    noise.vehicle = vehicle 
    analyses.append(noise)
 
    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
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
    base_segment.state.numerics.number_of_control_points = 10 
 

    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "final_approach"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_start                                           = 120.5   
    segment.altitude_end                                             = 10.0   * Units.ft
    segment.air_speed                                                = 160  * Units['knots']
    segment.descent_angle                                            = 3.  * Units.deg                               
           
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)
 
 
    return mission

def takeoff_mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points = 10  


    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff_Ground_Run" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start                                           = 20.* Units.knots
    segment.velocity_end                                             = 167.0 * Units['knots']
    segment.friction_coefficient                                     = 0.03
    segment.altitude                                                 = 5.0   
    segment.throttle                                                 = 0.8
    mission.append_segment(segment)

     
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Takeoff_Climb" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_end                                             = 35 * Units['ft']
    segment.air_speed_end                                            = 175.0 * Units['knots']
    segment.climb_rate                                               = 1800 * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment) 

    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Inital_Climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.altitude_end                                             = 1500  * Units['feet']
    segment.air_speed                                                = 200.0 * Units['knots']
    segment.climb_rate                                               = 1800   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)
 

    return mission

if __name__ == '__main__': 
    main()    
    plt.show()