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
    elevator_deflection_true   = -3.529563174967033
    elevator_deflection_diff   = np.abs(elevator_deflection - elevator_deflection_true)
    print('Elevator Error 1: ',elevator_deflection_diff)
    assert np.abs(elevator_deflection_diff/elevator_deflection_true) < 5e-3

    aileron_deflection        = results.segments.cruise.conditions.control_surfaces.aileron.deflection[0,0] / Units.deg
    print('Aileron Defection',aileron_deflection)
    aileron_deflection_true   = -8.020687738428768
    aileron_deflection_diff   = np.abs(aileron_deflection - aileron_deflection_true)
    print('Aileron Error 2: ',aileron_deflection_diff)
    assert np.abs(aileron_deflection_diff/aileron_deflection_true) < 5e-3

    rudder_deflection        = results.segments.cruise.conditions.control_surfaces.rudder.deflection[0,0] / Units.deg
    print('Rudder Defection',rudder_deflection)
    rudder_deflection_true   = 13.982585893837207
    rudder_deflection_diff   = np.abs(rudder_deflection - rudder_deflection_true)
    print('Rudder Error 3: ',rudder_deflection_diff)
    assert np.abs(rudder_deflection_diff/rudder_deflection_true) < 5e-3  

    throttle        = results.segments.cruise_2.conditions.energy.propulsors['ice_propeller'].throttle[0,0]
    throttle_true   = 0.5196768049345599
    throttle_diff   = np.abs(throttle - throttle_true)
    print('Throttle Error 1: ',throttle_diff)
    assert np.abs(throttle_diff/throttle_true) < 5e-3    

    throttle3        = results.segments.cruise_3.conditions.energy.propulsors['ice_propeller'].throttle[0,0]
    throttle3_true   = 0.3534245712851439
    throttle3_diff   = np.abs(throttle3 - throttle3_true)
    print('Throttle Error 2: ',throttle3_diff)
    assert np.abs(throttle3_diff/throttle3_true) < 5e-3

    # ------------------------------------------------------------------
    # Mode 1: crosswind_speed → β computed kinematically
    # crosswind = 50 * sin(10 deg) gives same effective β as cruise segment
    # results must match cruise within tolerance
    # ------------------------------------------------------------------
    elevator_cw      = results.segments.cruise_crosswind.conditions.control_surfaces.elevator.deflection[0,0] / Units.deg
    aileron_cw       = results.segments.cruise_crosswind.conditions.control_surfaces.aileron.deflection[0,0]  / Units.deg
    rudder_cw        = results.segments.cruise_crosswind.conditions.control_surfaces.rudder.deflection[0,0]   / Units.deg
    print('Crosswind elevator:', elevator_cw, 'aileron:', aileron_cw, 'rudder:', rudder_cw)
    assert np.abs((elevator_cw - elevator_deflection_true) / elevator_deflection_true) < 5e-3
    assert np.abs((aileron_cw  - aileron_deflection_true)  / aileron_deflection_true)  < 5e-3
    assert np.abs((rudder_cw   - rudder_deflection_true)   / rudder_deflection_true)   < 5e-3

    # ------------------------------------------------------------------
    # Mode 2: sideslip_angle as solver unknown
    # symmetric aircraft, no crosswind → solver must find β ≈ 0
    # bank_angle fixed at 0 to keep system well-determined (6×6)
    # ------------------------------------------------------------------
    sideslip_free    = results.segments.cruise_free_sideslip.conditions.frames.wind.body_rotations[0,2] / Units.deg
    rudder_free      = results.segments.cruise_free_sideslip.conditions.control_surfaces.rudder.deflection[0,0]  / Units.deg
    aileron_free     = results.segments.cruise_free_sideslip.conditions.control_surfaces.aileron.deflection[0,0] / Units.deg
    print('Free sideslip beta:', sideslip_free, 'rudder:', rudder_free, 'aileron:', aileron_free)
    assert np.abs(sideslip_free) < 1e-1
    assert np.abs(rudder_free)   < 1e-1
    assert np.abs(aileron_free)  < 1e-1

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
    segment.assigned_control_variables.pitch_angle.active                        = True       
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
    segment.assigned_control_variables.pitch_angle.active                        = True     
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
    segment.assigned_control_variables.pitch_angle.active                        = True       
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']] 
    segment.assigned_control_variables.aileron_deflection.active                = True    
    segment.assigned_control_variables.aileron_deflection.assigned_surfaces     = [['aileron']] 
    segment.assigned_control_variables.rudder_deflection.active                 = True    
    segment.assigned_control_variables.rudder_deflection.assigned_surfaces      = [['rudder']] 
    segment.assigned_control_variables.bank_angle.active                        = True
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Mode 1: crosswind_speed → β computed kinematically
    #   crosswind = air_speed * sin(10 deg), identical physics to cruise
    # ------------------------------------------------------------------
    segment     = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise_crosswind"
    segment.analyses.extend( analyses.base )
    segment.altitude                                                            = 1000. * Units.feet
    segment.air_speed                                                           = 50.00
    segment.crosswind_speed                                                     = 50.00 * np.sin(10.0 * Units.deg)

    segment.flight_dynamics.force_x                                             = True
    segment.flight_dynamics.force_z                                             = True
    segment.flight_dynamics.force_y                                             = True
    segment.flight_dynamics.moment_x                                            = True
    segment.flight_dynamics.moment_z                                            = True
    segment.flight_dynamics.moment_y                                            = True

    segment.assigned_control_variables.throttle.active                          = True
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['ice_propeller']]
    segment.assigned_control_variables.pitch_angle.active                       = True
    segment.assigned_control_variables.elevator_deflection.active               = True
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]
    segment.assigned_control_variables.aileron_deflection.active                = True
    segment.assigned_control_variables.aileron_deflection.assigned_surfaces     = [['aileron']]
    segment.assigned_control_variables.rudder_deflection.active                 = True
    segment.assigned_control_variables.rudder_deflection.assigned_surfaces      = [['rudder']]
    segment.assigned_control_variables.bank_angle.active                        = True
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Mode 2: sideslip_angle as free solver unknown
    #   symmetric aircraft, no crosswind → solver finds β ≈ 0
    #   bank_angle fixed at 0 to keep the system well-determined (6×6)
    # ------------------------------------------------------------------
    segment     = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise_free_sideslip"
    segment.analyses.extend( analyses.base )
    segment.altitude                                                            = 1000. * Units.feet
    segment.air_speed                                                           = 50.00

    segment.flight_dynamics.force_x                                             = True
    segment.flight_dynamics.force_z                                             = True
    segment.flight_dynamics.force_y                                             = True
    segment.flight_dynamics.moment_x                                            = True
    segment.flight_dynamics.moment_z                                            = True
    segment.flight_dynamics.moment_y                                            = True

    segment.assigned_control_variables.throttle.active                          = True
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['ice_propeller']]
    segment.assigned_control_variables.pitch_angle.active                       = True
    segment.assigned_control_variables.elevator_deflection.active               = True
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']]
    segment.assigned_control_variables.aileron_deflection.active                = True
    segment.assigned_control_variables.aileron_deflection.assigned_surfaces     = [['aileron']]
    segment.assigned_control_variables.rudder_deflection.active                 = True
    segment.assigned_control_variables.rudder_deflection.assigned_surfaces      = [['rudder']]
    segment.assigned_control_variables.sideslip_angle.active                    = True
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