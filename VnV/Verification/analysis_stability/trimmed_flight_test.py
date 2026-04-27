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
# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(): 
    
    # vehicle data
    vehicle  = vehicle_setup()

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
    
    '''Values are different from trimmed stability derivative test because stability derivatives are different.'''
    elevator_deflection        = results.segments.cruise.conditions.control_surfaces.elevator.deflection[0,0] / Units.deg
    print('Elevator Defection',elevator_deflection)
    elevator_deflection_true   = 1.4877866750722837
    elevator_deflection_diff   = np.abs(elevator_deflection - elevator_deflection_true)
    print('Elevator Error 1: ',elevator_deflection_diff)
    assert np.abs(elevator_deflection_diff/elevator_deflection_true) < 5e-3

    aileron_deflection        = results.segments.cruise.conditions.control_surfaces.aileron.deflection[0,0] / Units.deg
    print('Aileron Defection',aileron_deflection)
    aileron_deflection_true   = 3.043403012867304
    aileron_deflection_diff   = np.abs(aileron_deflection - aileron_deflection_true)
    print('Aileron Error 2: ',aileron_deflection_diff)
    assert np.abs(aileron_deflection_diff/aileron_deflection_true) < 5e-3

    rudder_deflection        = results.segments.cruise.conditions.control_surfaces.rudder.deflection[0,0] / Units.deg
    print('Rudder Defection',rudder_deflection)
    rudder_deflection_true   = -23.62037548016846
    rudder_deflection_diff   = np.abs(rudder_deflection - rudder_deflection_true)
    print('Rudder Error 3: ',rudder_deflection_diff)
    assert np.abs(rudder_deflection_diff/rudder_deflection_true) < 5e-3  

    throttle        = results.segments.cruise_2.conditions.energy.propulsors['ice_propeller'].throttle[0,0]
    throttle_true   = 0.37537152645136324
    throttle_diff   = np.abs(throttle - throttle_true)
    print('Throttle Error 1: ',throttle_diff)
    assert np.abs(throttle_diff/throttle_true) < 5e-3    

    throttle3        = results.segments.cruise_3.conditions.energy.propulsors['ice_propeller'].throttle[0,0]
    throttle3_true   = 0.4845433366671214
    throttle3_diff   = np.abs(throttle3 - throttle3_true)
    print('Throttle Error 2: ',throttle3_diff)
    assert np.abs(throttle3_diff/throttle3_true) < 5e-3   

    # plt results
    plot_mission(results)
    
    return  
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


def base_analysis(vehicle):

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
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_General_Aviation() 
    weights.settings.run_weights_analysis = True
    weights.settings.run_moments_of_inertia_analysis = True
    weights.settings.run_center_of_gravity_analysis = True
    analyses.append(weights) 

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()   
    analyses.append(aerodynamics) 

    # ------------------------------------------------------------------
    #  Stability Analysis      
    stability   = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method()   
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
 
    plot_longitudinal_stability(results)  
    
    plot_lateral_stability(results) 
    
    plot_flight_forces_and_moments(results)
    
    plot_center_of_gravity_drift(results)
    
    plot_moment_of_intertia_drift(results)
      
    return
 
# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------

def mission_setup(analyses):


    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments
    
    base_segment = Segments.Segment() 
    base_segment.state.numerics.number_of_control_points = 3
 
    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------      
    segment     = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.base )   
    segment.altitude                                                            = 1000. * Units.feet
    segment.air_speed                                                           = 50.00
    segment.sideslip_angle                                                      = 10.0 * Units.deg  
    
    # equations of motion
    segment.flight_dynamics.force_x                                             = True    
    segment.flight_dynamics.force_z                                             = True
    segment.flight_dynamics.force_y                                             = True        
    segment.flight_dynamics.moment_x                                            = True
    segment.flight_dynamics.moment_z                                            = True
    segment.flight_dynamics.moment_y                                            = True  
    
    # flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['ice_propeller']]   
    segment.assigned_control_variables.body_angle.active                        = True       
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']] 
    segment.assigned_control_variables.aileron_deflection.active                = True    
    segment.assigned_control_variables.aileron_deflection.assigned_surfaces     = [['aileron']] 
    segment.assigned_control_variables.rudder_deflection.active                 = True    
    segment.assigned_control_variables.rudder_deflection.assigned_surfaces      = [['rudder']] 
    segment.assigned_control_variables.bank_angle.active                        = True         
    mission.append_segment(segment)

     # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------      
    segment     = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise_2" 
    segment.analyses.extend( analyses.base )   
    segment.altitude                                                            = 1000. * Units.feet
    segment.air_speed                                                           = 50.00
    segment.sideslip_angle                                                      = 10.0 * Units.deg    
    
    # equations of motion
    segment.flight_dynamics.force_x                                             = True    
    segment.flight_dynamics.force_z                                             = True
    
    # flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['ice_propeller']]   
    segment.assigned_control_variables.body_angle.active                        = True     
    mission.append_segment(segment)
 
    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------      
    segment     = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise_3" 
    segment.analyses.extend( analyses.base )   
    segment.altitude                                                            = 1000. * Units.feet
    segment.air_speed                                                           = 50.00
    segment.sideslip_angle                                                      = 10.0 * Units.deg    

    # equations of motion
    segment.flight_dynamics.force_x                                             = True    
    segment.flight_dynamics.force_z                                             = True
    segment.flight_dynamics.force_y                                             = True        
    segment.flight_dynamics.moment_x                                            = True
    segment.flight_dynamics.moment_z                                            = True
    segment.flight_dynamics.moment_y                                            = True  
    
    # flight controls              
    segment.assigned_control_variables.throttle.active                          = True           
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['ice_propeller']]   
    segment.assigned_control_variables.body_angle.active                        = True       
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']] 
    segment.assigned_control_variables.aileron_deflection.active                = True    
    segment.assigned_control_variables.aileron_deflection.assigned_surfaces     = [['aileron']] 
    segment.assigned_control_variables.rudder_deflection.active                 = True    
    segment.assigned_control_variables.rudder_deflection.assigned_surfaces      = [['rudder']] 
    segment.assigned_control_variables.bank_angle.active                        = True                 
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