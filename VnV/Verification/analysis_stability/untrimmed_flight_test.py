# vlm_pertubation_test.py
# 
# Created: May 2024, M. Clarke
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE 
from RCAIDE.Framework.Core import Units     
from RCAIDE.Library.Plots       import *  

# python imports  
import pylab as plt
import numpy as np 


# local imports 
import sys 
import os

base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Navion    import vehicle_setup, configs_setup     
import time
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(): 
    ti = time.time()
    
    # vehicle data
    vehicle  = vehicle_setup() 

    # append slat to exercise the Slat branch in evaluate_no_surrogate
    slat                     = RCAIDE.Library.Components.Wings.Control_Surfaces.Slat()
    slat.tag                 = 'slat'
    slat.span_fraction_start = 0.1
    slat.span_fraction_end   = 0.7
    slat.deflection          = 0.0 * Units.degrees
    slat.chord_fraction      = 0.1
    vehicle.wings.main_wing.append_control_surface(slat)

    # Set up vehicle configs
    configs  = configs_setup(vehicle) 

    # create analyses
    analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(analyses) 

    # create mission instances (for multiple types of missions)
    missions = missions_setup(mission) 

    # mission analysis 
    results = missions.base_mission.evaluate() 

    CL        = results.segments.cruise.conditions.aerodynamics.coefficients.lift.total[0][0]
    CL_true   = 0.7130354497601528
    CL_diff   = np.abs(CL - CL_true)
    print('Error: ',CL_diff)
    assert np.abs(CL_diff/CL_true) < 1e-6

    # regression guard for a former copy-paste bug in evaluate_no_surrogate() where the Slat branch
    # wrote dCdrag_ddelta_s into the Clift_delta_s key instead of Cdrag_delta_s, leaving Cdrag_delta_s
    # at its zero default. Confirm Cdrag_delta_s is actually populated and distinct from Clift_delta_s.
    Cdrag_delta_s = results.segments.cruise.conditions.static_stability.derivatives.Cdrag_delta_s[0][0]
    Clift_delta_s = results.segments.cruise.conditions.static_stability.derivatives.Clift_delta_s[0][0]
    print('Cdrag_delta_s:', Cdrag_delta_s, 'Clift_delta_s:', Clift_delta_s)
    assert Cdrag_delta_s != 0.0
    assert Cdrag_delta_s != Clift_delta_s


    elapsed_time = time.time() - ti
    elapsed_time_min = elapsed_time / 60
    print('Elapsed time (min): ', elapsed_time_min)
    return  
# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config, configs)
        analyses[tag] = analysis

    return analyses


def base_analysis(vehicle, configs):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle =  vehicle

    # ------------------------------------------------------------------
    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry() 
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()  
    aerodynamics.settings.use_surrogate                 = False 
    analyses.append(aerodynamics) 

    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Electric_General_Aviation() 
    analyses.append(weights) 

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method()  
    stability.settings.use_surrogate                = False 
    analyses.append(stability)
    
    # ------------------------------------------------------------------
    #  Energy
    energy= RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    # done!
    return analyses  

def plot_mission(results): 

    # Plot Aircraft Stability 
    plot_longitudinal_stability(results)  
    
    plot_lateral_stability(results) 
    
    plot_flight_forces_and_moments(results) 
      
    return
 
# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------ 
 
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'
  
    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments

    #   Cruise Segment: constant Speed, constant altitude 
    segment                           = Segments.Untrimmed.Untrimmed()
    segment.analyses.extend( analyses.base )   
    segment.tag                       = "cruise"
    segment.angle_of_attack           = 5 * Units.degrees
    segment.altitude                  = 5000 * Units.feet
    segment.air_speed                 = 150 * Units.mph

    segment.flight_dynamics.force_x   = True    
    segment.flight_dynamics.force_z   = True    
    segment.flight_dynamics.force_y   = True     
    segment.flight_dynamics.moment_y  = True 
    segment.flight_dynamics.moment_x  = True
    segment.flight_dynamics.moment_z  = True

    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.5]]
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['ice_propeller']] 
        
    mission.append_segment(segment)      

    return mission 

def missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions  
 

if __name__ == '__main__': 
    main()    
    plt.show()