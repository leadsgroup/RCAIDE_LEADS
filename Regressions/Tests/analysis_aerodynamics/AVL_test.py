# AVL_test.py
# 
# Created:  Dec 2023, M. Clarke 

""" setup file for segment test regression with a Boeing 737"""

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units ,  Data
from RCAIDE.Library.Plots             import *       

# python imports 
import numpy as np
import pylab as plt 
import sys
import os

sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Navion    import vehicle_setup, configs_setup 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    
    new_regression_results = False  # Keep False, Only True when getting new results for regression 

    
    use_surrogate          = False  
    trim_aircraft          = True  
    keep_regression_files  = True
    folder_name            = '_single_point'
    AVL_Single_Point_Trim_Mission(use_surrogate,trim_aircraft,keep_regression_files,new_regression_results,folder_name)

    use_surrogate          = True
    trim_aircraft          = False
    keep_regression_files  = True 
    folder_name            = '_surrogate'
    AVL_Surrogate_Mission(use_surrogate,trim_aircraft,keep_regression_files,new_regression_results,folder_name)    
 
    return 
    
def AVL_Surrogate_Mission(use_surrogate,trim_aircraft,keep_regression_files,new_regression_results,folder_name):
    # vehicle data
    vehicle  = vehicle_setup()  
    
    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = analyses_setup(configs,use_surrogate,trim_aircraft,keep_regression_files,new_regression_results,folder_name)

    # mission analyses 
    mission = AVL_Surrogate_mission_setup(analyses)
    
    # create mission instances (for multiple types of missions)
    missions = missions_setup(mission) 
     
    # mission analysis 
    results = missions.base_mission.evaluate()   
 
    # Extract sample values from computation   
    cruise_CL        = results.segments.cruise.conditions.aerodynamics.coefficients.lift.total[2][0] 
    cruise_CL_thruth = 0.46795223576170475
    # Truth values  
    error = Data()  
    error.cruise_CL   = np.max(np.abs(cruise_CL     - cruise_CL_thruth))   
    print('Errors:')
    print(error)
     
    for k,v in list(error.items()): 
        assert(np.abs(v)<1e-3)
         
    return


def AVL_Single_Point_Trim_Mission(use_surrogate,trim_aircraft,keep_regression_files,new_regression_results,folder_name):
    # vehicle data
    vehicle  = vehicle_setup()   
    
    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = analyses_setup(configs,use_surrogate,trim_aircraft,keep_regression_files,new_regression_results,folder_name)

    # mission analyses 
    mission = AVL_Single_Point_mission_setup(analyses)
    
    # create mission instances (for multiple types of missions)
    missions = missions_setup(mission) 
     
    # mission analysis 
    results = missions.base_mission.evaluate()   
 
    # Extract sample values from computation   
    cruise_CL        = results.segments.cruise.conditions.aerodynamics.coefficients.lift.total[0][0]
    cruise_CL_thruth = 0.45599999999999996
    
    # Truth values  
    error = Data()  
    error.cruise_CL   = np.max(np.abs(cruise_CL     - cruise_CL_thruth))   
    print('Errors:')
    print(error)
     
    for k,v in list(error.items()): 
        assert(np.abs(v)<1e-3)
         
    return


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs,use_surrogate,trim_aircraft,keep_regression_files,new_regression_results,folder_name):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in list(configs.items()):
        analysis = base_analysis(config,use_surrogate,trim_aircraft,keep_regression_files,new_regression_results,folder_name)
        analyses[tag] = analysis
 
    return analyses


def base_analysis(vehicle,use_surrogate,trim_aircraft,keep_regression_files,new_regression_results,folder_name):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 

    # ------------------------------------------------------------------
    #  Weights
    weights                                          = RCAIDE.Framework.Analyses.Weights.Conventional()
    weights.vehicle                                  = vehicle
    analyses.append(weights)
 
    #  Aerodynamics Analysis
    aerodynamics                                     = RCAIDE.Framework.Analyses.Aerodynamics.Athena_Vortex_Lattice()
    aerodynamics.vehicle                             = vehicle 
    aerodynamics.settings.filenames.avl_bin_name     = '/Users/matthewclarke/Documents/LEADS/CODES/AVL/avl3.35'
    aerodynamics.settings.filenames.run_folder       = 'avl_files' +  folder_name
    aerodynamics.settings.use_surrogate              = use_surrogate
    aerodynamics.settings.trim_aircraft              = trim_aircraft 
    aerodynamics.settings.model_fuselage             = False 
    aerodynamics.settings.print_output               = False 
    aerodynamics.settings.keep_files                 = keep_regression_files          
    aerodynamics.settings.new_regression_results     = new_regression_results
    analyses.append(aerodynamics)
    

    # Stability Analysis
    stability                                        = RCAIDE.Framework.Analyses.Stability.Athena_Vortex_Lattice()
    stability.vehicle                                = vehicle 
    stability.settings.filenames.avl_bin_name        = '/Users/matthewclarke/Documents/LEADS/CODES/AVL/avl3.35'
    stability.settings.filenames.run_folder          = 'avl_files' +  folder_name
    stability.settings.use_surrogate                 = use_surrogate
    stability.settings.trim_aircraft                 = trim_aircraft 
    stability.settings.model_fuselage                = False 
    stability.settings.print_output                  = False 
    stability.settings.keep_files                    = keep_regression_files        
    stability.settings.new_regression_results        = new_regression_results
    analyses.append(stability)    
  
    #  Energy
    energy                                           = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle                                   = vehicle 
    analyses.append(energy)
 
    #  Planet Analysis
    planet                                           = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere                                       = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet                       = planet.features
    analyses.append(atmosphere)   

    # done!
    return analyses 
   
# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def AVL_Surrogate_mission_setup(analyses): 

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points = 4  
 
 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Cruise Segment 3 : Constant Pitch Rate Constant Altltude
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                                = 2500 * Units.feet
    segment.air_speed                                               = 120 * Units['mph']
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                 = True  
    segment.flight_dynamics.force_z                                 = True     
    
    # define flight controls 
    segment.assigned_control_variables.body_angle.active             = True     
    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.5]]
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['ice_propeller']]    
    mission.append_segment(segment)    
    
    
    return mission

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def AVL_Single_Point_mission_setup(analyses): 

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 
 
 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Cruise Segment 3 : Constant Pitch Rate Constant Altltude
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Single_Point.Set_Speed_Set_Altitude_AVL_Trimmed(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                                = 2500 * Units.feet
    segment.air_speed                                               = 120 * Units['mph']
    segment.trim_lift_coefficient                                   = 0.4
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                 = True  
    segment.flight_dynamics.force_z                                 = True     
    
    # define flight controls 
    segment.assigned_control_variables.body_angle.active             = True     
    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.5]]
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['ice_propeller']]   
    mission.append_segment(segment)    
    
    
    return mission

def missions_setup(mission): 
 
    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions  


if __name__ == '__main__': 
    main()    
