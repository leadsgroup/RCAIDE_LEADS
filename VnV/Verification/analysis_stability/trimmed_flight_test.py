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
    
    # ------------------------------------------------------------------
    # Cruise segment (6-DOF with sideslip = 10 deg)
    # ------------------------------------------------------------------
    cruise_elevator       = results.segments.cruise.conditions.control_surfaces.elevator.deflection[0,0] / Units.deg
    cruise_aileron        = results.segments.cruise.conditions.control_surfaces.aileron.deflection[0,0] / Units.deg
    cruise_rudder         = results.segments.cruise.conditions.control_surfaces.rudder.deflection[0,0] / Units.deg
    cruise_elevator_true  = -1.2899769221036859
    cruise_aileron_true   = -7.660066661814855
    cruise_rudder_true    = 14.06335960790088
    print('Cruise elevator:', cruise_elevator, 'aileron:', cruise_aileron, 'rudder:', cruise_rudder)
    assert np.abs((cruise_elevator - cruise_elevator_true) / cruise_elevator_true) < 5e-3
    assert np.abs((cruise_aileron  - cruise_aileron_true)  / cruise_aileron_true)  < 5e-3
    assert np.abs((cruise_rudder   - cruise_rudder_true)   / cruise_rudder_true)   < 5e-3

    # ------------------------------------------------------------------
    # Cruise 2 segment (2-DOF longitudinal only)
    # ------------------------------------------------------------------
    cruise2_throttle       = results.segments.cruise_2.conditions.energy.propulsors['ice_propeller'].throttle[0,0]
    cruise2_throttle_true  = 0.491661137864161
    print('Cruise 2 throttle:', cruise2_throttle)
    assert np.abs((cruise2_throttle - cruise2_throttle_true) / cruise2_throttle_true) < 5e-3

    # ------------------------------------------------------------------
    # Cruise 3 segment (6-DOF with sideslip = 10 deg)
    # ------------------------------------------------------------------
    cruise3_throttle       = results.segments.cruise_3.conditions.energy.propulsors['ice_propeller'].throttle[0,0]
    cruise3_throttle_true  = 0.6729500171889758
    print('Cruise 3 throttle:', cruise3_throttle)
    assert np.abs((cruise3_throttle - cruise3_throttle_true) / cruise3_throttle_true) < 5e-3

    # ------------------------------------------------------------------
    # Crosswind segment (6-DOF, crosswind_speed → β computed kinematically)
    # ------------------------------------------------------------------
    cw_elevator       = results.segments.cruise_crosswind.conditions.control_surfaces.elevator.deflection[0,0] / Units.deg
    cw_aileron        = results.segments.cruise_crosswind.conditions.control_surfaces.aileron.deflection[0,0]  / Units.deg
    cw_rudder         = results.segments.cruise_crosswind.conditions.control_surfaces.rudder.deflection[0,0]   / Units.deg
    cw_elevator_true  = -1.3019677531930047
    cw_aileron_true   = -7.655548544445317
    cw_rudder_true    = 14.064149405090152
    print('Crosswind elevator:', cw_elevator, 'aileron:', cw_aileron, 'rudder:', cw_rudder)
    assert np.abs((cw_elevator - cw_elevator_true) / cw_elevator_true) < 5e-3
    assert np.abs((cw_aileron  - cw_aileron_true)  / cw_aileron_true)  < 5e-3
    assert np.abs((cw_rudder   - cw_rudder_true)   / cw_rudder_true)   < 5e-3

    # ------------------------------------------------------------------
    # Free sideslip segment (symmetric, no crosswind → β ≈ 0)
    # ------------------------------------------------------------------
    fs_sideslip       = results.segments.cruise_free_sideslip.conditions.frames.wind.body_rotations[0,2] / Units.deg
    fs_elevator       = results.segments.cruise_free_sideslip.conditions.control_surfaces.elevator.deflection[0,0] / Units.deg
    fs_aileron        = results.segments.cruise_free_sideslip.conditions.control_surfaces.aileron.deflection[0,0] / Units.deg
    fs_rudder         = results.segments.cruise_free_sideslip.conditions.control_surfaces.rudder.deflection[0,0]  / Units.deg
    fs_elevator_true  = -1.296188721309234
    print('Free sideslip beta:', fs_sideslip, 'elevator:', fs_elevator, 'rudder:', fs_rudder, 'aileron:', fs_aileron)
    assert np.abs(fs_sideslip) < 1e-6
    assert np.abs(fs_aileron)  < 1e-6
    assert np.abs(fs_rudder)   < 1e-6
    assert np.abs((fs_elevator - fs_elevator_true) / fs_elevator_true) < 5e-3

    # ------------------------------------------------------------------
    # Dynamic stability modes (cruise segment)
    # ------------------------------------------------------------------
    DS                             = results.segments.cruise.conditions.dynamic_stability
    phugoid_freq                   = DS.LongModes.phugoidFreqHz[0,0]
    phugoid_damping                = DS.LongModes.phugoidDamping[0,0]
    short_period_freq              = DS.LongModes.shortPeriodFreqHz[0,0]
    short_period_damping           = DS.LongModes.shortPeriodDamping[0,0]
    dutch_roll_freq                = DS.LatModes.dutchRollFreqHz[0,0]
    dutch_roll_damping             = DS.LatModes.dutchRollDamping[0,0]
    roll_subsistence_time_constant = DS.LatModes.rollSubsistenceTimeConstant[0,0]
    spiral_time_double_half        = DS.LatModes.spiralTimeDoubleHalf[0,0]

    phugoid_freq_true                   = 0.0498980620834614
    phugoid_damping_true                = 0.14598132422045618
    short_period_freq_true              = 0.2875274425477587
    short_period_damping_true           = 0.7832908110862572
    dutch_roll_freq_true                = 0.486458976924637
    dutch_roll_damping_true             = 0.13883619187347213
    roll_subsistence_time_constant_true = 0.028073156923046644
    spiral_time_double_half_true        = 423.073887387459

    print('Phugoid freq (Hz):', phugoid_freq, 'damping:', phugoid_damping)
    print('Short period freq (Hz):', short_period_freq, 'damping:', short_period_damping)
    print('Dutch roll freq (Hz):', dutch_roll_freq, 'damping:', dutch_roll_damping)
    print('Roll subsistence time constant (s):', roll_subsistence_time_constant)
    print('Spiral time to double/half (s):', spiral_time_double_half)
    assert np.abs((phugoid_freq                   - phugoid_freq_true)                   / phugoid_freq_true)                   < 5e-3
    assert np.abs((phugoid_damping                - phugoid_damping_true)                / phugoid_damping_true)                < 5e-3
    assert np.abs((short_period_freq               - short_period_freq_true)              / short_period_freq_true)              < 5e-3
    assert np.abs((short_period_damping            - short_period_damping_true)           / short_period_damping_true)           < 5e-3
    assert np.abs((dutch_roll_freq                 - dutch_roll_freq_true)                / dutch_roll_freq_true)                < 5e-3
    assert np.abs((dutch_roll_damping              - dutch_roll_damping_true)             / dutch_roll_damping_true)             < 5e-3
    assert np.abs((roll_subsistence_time_constant  - roll_subsistence_time_constant_true) / roll_subsistence_time_constant_true) < 5e-3
    assert np.abs((spiral_time_double_half         - spiral_time_double_half_true)        / spiral_time_double_half_true)        < 5e-3

    # plt results
    plot_mission(results)
    

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