# SU2_test.py
# 
# Created:  Aug 2025, M. Clarke

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
import shutil 

# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Navion    import vehicle_setup as vehicle_setup
from Navion    import configs_setup as configs_setup 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():
    new_regression             = True # should be false
    gmsh_installation          = False 
    vsp_installation           = False 
    generate_new_training_data = False

    try: 
        import gmsh as gmsh
        gmsh_installation = True   
    except:
        pass

    try:  
        try:
            import vsp as vsp
        except:
            import openvsp as vsp
        vsp_installation  = True 
    except:
        pass 


    su2_path = shutil.which("SU2_CFD")
    if su2_path == None:
        SU2_installation = False
    else:
        SU2_installation = True
        
    # if new regression and all flags are true, run SU2
    if new_regression:
        if (gmsh_installation == False) or (vsp_installation  == False) or (SU2_installation  == False):
            return
        generate_new_training_data = True 
    else:
        generate_new_training_data = False
    
    # vehicle data
    vehicle  = vehicle_setup()  
    
    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = analyses_setup(configs,generate_new_training_data)

    # mission analyses 
    mission = mission_setup(analyses)
    
    # create mission instances (for multiple types of missions)
    missions = missions_setup(mission) 
     
    # mission analysis 
    results = missions.base_mission.evaluate()   
 
    # Extract sample values from computation   
    cruise_CL       = results.segments.cruise.conditions.aerodynamics.coefficients.lift.total[2][0] 
    
    #print values for resetting regression
    show_vals = True
    if show_vals:
        data = [cruise_CL]
        for val in data:
            print(val)
    
    # Truth values 
    cruise_CL_truth       = 0.2753927270500611
    
    # Store errors 
    error = Data() 
    error.cruise_1        = np.max((np.abs(cruise_CL          - cruise_CL_truth ))/cruise_CL_truth)       
     
    print('Errors:')
    print(error)
     
    for k,v in list(error.items()): 
        assert(np.abs(v)<1e-3)
        
    return 
# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs, generate_new_training_data):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in list(configs.items()):
        analysis = base_analysis(config,generate_new_training_data)
        analyses[tag] = analysis
 
    return analyses


def base_analysis(vehicle, generate_new_training_data):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    
    # ------------------------------------------------------------------
    #  geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.vehicle = vehicle 
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Weights
    weights                                          = RCAIDE.Framework.Analyses.Weights.Conventional_Transport() 
    weights.vehicle                                  = vehicle
    analyses.append(weights)
 
    #  Aerodynamics Analysis
    aerodynamics                                        = RCAIDE.Framework.Analyses.Aerodynamics.SU2_Euler()
    aerodynamics.vehicle                                = vehicle
    aerodynamics.settings.run_new_SU2_sim               = True
    aerodynamics.settings.store_training_data           = True
    aerodynamics.training.angle_of_attack               = np.array([-1, 3, 8]) * Units.deg 
    aerodynamics.training.Mach                          = np.array([0.1, 0.4]) 
    analyses.append(aerodynamics)

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

def mission_setup(analyses): 

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points = 8  
    base_segment.state.numerics.solver.type              = 'root_finder' 
  
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Cruise Segment: constant Speed, constant altitude
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                                 = 1000 * Units.feet 
    segment.mach_number                                              = 0.2 
    segment.distance                                                 = 50 * Units.km  
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['ice_propeller']] 
    segment.assigned_control_variables.throttle.bounds             = [[-90.0*Units.deg, 90.0*Units.deg]]    
    segment.assigned_control_variables.body_angle.active             = True     
    segment.assigned_control_variables.body_angle.bounds             = [[-0.2, 5.0]]         
    
    mission.append_segment(segment)   
    return mission


def missions_setup(mission): 
 
    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions  


if __name__ == '__main__': 
    main() 