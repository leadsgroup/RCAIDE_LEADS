''' 
# transition_segment_test.py
# 
# Created: May 2019, M Clarke
#          Sep 2020, M. Clarke 

'''
#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units, Data     
from RCAIDE.Library.Plots  import *       
from RCAIDE.Library.Methods.Performance.estimate_stall_speed    import estimate_stall_speed 
  
# python imports     
import numpy as np  
import sys
import matplotlib.pyplot as plt    
import os
import time

# local imports 
base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Tiltrotor_EVTOL        import vehicle_setup as  TR_vehicle_setup 
from Tiltrotor_EVTOL        import configs_setup as  TR_configs_setup 
from Tiltwing_EVTOL         import vehicle_setup as  TW_vehicle_setup 
from Tiltwing_EVTOL         import configs_setup as  TW_configs_setup 
from Stopped_Rotor_EVTOL    import vehicle_setup as  SR_vehicle_setup 
from Stopped_Rotor_EVTOL    import configs_setup as  SR_configs_setup


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    ti = time.time()
    # make true only when resizing aircraft. should be left false for regression
    update_regression_values = False

    # TEST 1
    tiltrotor_transition_test(update_regression_values)

    # TEST 2
    tiltwing_transition_test(update_regression_values)

    # TEST 3
    stopped_rotor_transition_test(update_regression_values)

    elapsed_time = time.time() - ti
    elapsed_time_min = elapsed_time / 60
    print('Elapsed time (min): ', elapsed_time_min)
    return

def tiltrotor_transition_test(update_regression_values): 
         
    vehicle  = TR_vehicle_setup(redesign_rotors=update_regression_values)

    # ------------------------------------------------------------------
    #  FIDELITY / COUPLING TOGGLE -- uncomment exactly ONE block below
    # ------------------------------------------------------------------
    # --- BEMT ---
    #for propulsor in vehicle.networks.electric.propulsors:
    #    propulsor.rotor.fidelity = 'Blade_Element_Momentum_Theory_Helmholtz_Wake'

    # --- LL uncoupled ---
    #for propulsor in vehicle.networks.electric.propulsors:
    #    propulsor.rotor.fidelity = 'Lifting_Line_Theory'
    #vehicle.networks.electric.wing_on_rotor_coupling  = False
    #vehicle.networks.electric.rotor_on_rotor_coupling = False

    # --- LL coupled ---
    for propulsor in vehicle.networks.electric.propulsors:
        propulsor.rotor.fidelity = 'Lifting_Line_Theory'
    vehicle.networks.electric.wing_on_rotor_coupling  = True
    vehicle.networks.electric.rotor_on_rotor_coupling = True

    # Set up configs
    configs  = TR_configs_setup(vehicle)
    
    # vehicle analyses
    analyses = TR_analyses_setup(configs)
    
    # mission analyses
    mission  = TR_mission_setup(analyses)
    missions = missions_setup(mission) 
    
    ti                   = time.time()       
    TR_results = missions.base_mission.evaluate()  
    
    # Extract sample values from computation
    hover_throttle          = TR_results.segments.vertical_climb.conditions.energy.propulsors['front_port_propulsor'].throttle[1][0]
    cruise_rpm              = TR_results.segments.cruise.conditions.energy.converters.front_port_rotor.rpm[0][0]
    transition_throttle    = TR_results.segments.departure_transition_1.conditions.energy.propulsors['front_port_propulsor'].throttle[-1][0]
    transition_tva          = TR_results.segments.departure_transition_1.conditions.energy.propulsors['front_port_propulsor'].commanded_thrust_vector_angle[-1][0]

    print('Aircraft: Tiltrotor EVTOL')
    print('Computed values:')
    print('Hover throttle: ', hover_throttle)
    print('departure_transition_1 throttle: ', transition_throttle)
    print('departure_transition_1 thrust_vector_angle (rad): ', transition_tva)
    print('Cruise RPM: ', cruise_rpm)

    tf                   = time.time()
    elapsed_time         = round((tf-ti)/60,2)
    print('Simulation Time: ' + str(elapsed_time) + ' mins')

    # Truth values -- baselined against the full 3-segment mission
    # (Vertical_Climb -> departure_transition_1 -> cruise), BEMT fidelity
    hover_throttle_truth       = 0.4652824222207641
    transition_throttle_truth = 0.449939078418756
    transition_tva_truth      = 0.03841861419222897
    cruise_rpm_truth          = 601.0023312618267

    # Store errors
    error = Data()
    error.hover_throttle       = np.max(np.abs( hover_throttle_truth       - hover_throttle       )/ hover_throttle_truth )
    error.transition_throttle = np.max(np.abs( transition_throttle_truth  - transition_throttle  )/ transition_throttle_truth )
    error.transition_tva      = np.max(np.abs( transition_tva_truth       - transition_tva        )/ transition_tva_truth )
    error.cruise_rpm           = np.max(np.abs( cruise_rpm_truth           - cruise_rpm            )/ cruise_rpm_truth )

    print('Errors:')
    print(error)

    for k,v in list(error.items()):
        assert(np.abs(v)<1e-1)
    return
 

def tiltwing_transition_test(update_regression_values):    
    TW_vehicle  = TW_vehicle_setup(update_regression_values)  
        
    # Set up configs
    TW_configs  = TW_configs_setup(TW_vehicle)

    # vehicle analyses
    TW_analyses = TW_analyses_setup(TW_configs)

    # mission analyses
    TW_mission  = TW_mission_setup(TW_analyses)
    TW_missions = missions_setup(TW_mission) 
     
    TW_results = TW_missions.base_mission.evaluate()  
    
    # Extract sample values from computation    
    hover_throttle            = TW_results.segments.hover.conditions.energy.propulsors['prop_rotor_propulsor_1'].throttle[1][0]
    vertical_climb_1_throttle = TW_results.segments.vertical_climb.conditions.energy.propulsors['prop_rotor_propulsor_1'].throttle[1][0] 
    vertical_descent_throttle = TW_results.segments.vertical_descent.conditions.energy.propulsors['prop_rotor_propulsor_1'].throttle[1][0] 

    print('Aircraft: Tiltwing EVTOL')
    print('Computed values:')
    print('Hover throttle: ', hover_throttle)
    print('Vertical climb 1 throttle: ', vertical_climb_1_throttle)
    print('Vertical descent throttle: ', vertical_descent_throttle)
     
    
    # Truth values
    hover_throttle_truth              = 0.7194265956924623
    vertical_climb_1_throttle_truth   = 0.7300152759744121
    vertical_descent_throttle_truth   = 0.7083439429383551
    
    # Store errors 
    error = Data() 
    error.hover_throttle             = np.max(np.abs( hover_throttle_truth            - hover_throttle            )/ hover_throttle_truth            )
    error.vertical_climb_1_throttle  = np.max(np.abs( vertical_climb_1_throttle_truth - vertical_climb_1_throttle )/ vertical_climb_1_throttle_truth ) 
    error.vertical_descent_throttle  = np.max(np.abs( vertical_descent_throttle_truth - vertical_descent_throttle )/ vertical_descent_throttle_truth )
 
    print('Errors:')
    print(error)
     
    for k,v in list(error.items()):
        assert(np.abs(v)<1e-1)   # lower tolerance due to lose bounds on prop-rotor blade design 
    return

def stopped_rotor_transition_test(update_regression_values):
    SR_vehicle  = SR_vehicle_setup(update_regression_values)
    
    # Set up configs
    SR_configs  = SR_configs_setup(SR_vehicle)

    # vehicle analyses
    SR_analyses = SR_analyses_setup(SR_configs)

    # mission analyses
    SR_mission  = SR_mission_setup(SR_analyses,SR_vehicle)
    SR_missions = missions_setup(SR_mission) 
     
    SR_results = SR_missions.base_mission.evaluate()  
    
    # Extract sample values from computation    
    hover_throttle     = SR_results.segments.vertical_climb.conditions.energy.propulsors['lift_propulsor_1'].throttle[1][0]
    lst_throttle       = SR_results.segments.low_speed_transition.conditions.energy.propulsors['lift_propulsor_1'].throttle[1][0] 
    hsct_throttle      = SR_results.segments.high_speed_climbing_transition.conditions.energy.propulsors['lift_propulsor_1'].throttle[1][0] 
    
    print('Aircraft: Stopped Rotor')
    print('Computed values:')
    print('Hover throttle: ', hover_throttle)
    print('Low speed transition throttle: ', lst_throttle)
    print('High speed climbing transition throttle: ', hsct_throttle) 
    
    # Truth values
    hover_throttle_truth  = 0.5378614012803635
    lst_throttle_truth    = 0.5208595949616026
    hsct_throttle_truth   = 0.4137930404945779
    
    # Store errors 
    error = Data() 
    error.hover_throttle = np.max(np.abs( hover_throttle_truth  - hover_throttle  )/ hover_throttle_truth )
    error.lst_throttle   = np.max(np.abs( lst_throttle_truth    - lst_throttle    )/ lst_throttle_truth   ) 
    error.hsct_throttle  = np.max(np.abs( hsct_throttle_truth   - hsct_throttle   )/ hsct_throttle_truth  )

    print('Errors:')
    print(error)
     
    for k,v in list(error.items()):
        assert(np.abs(v)<1e-1)   # lower tolerance due to lose bounds on prop-rotor blade design 
    return  

# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------
def TW_analyses_setup(configs): 
    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = TW_base_analysis(config)
        if config.networks.electric.propulsors['prop_rotor_propulsor_1'].rotor.orientation_euler_angles[1] > 45*Units.degrees: 
            analysis.aerodynamics.settings.drag_coefficient_increment =  0.10
        elif config.networks.electric.propulsors['prop_rotor_propulsor_1'].rotor.orientation_euler_angles[1] > 15*Units.degrees: 
            analysis.aerodynamics.settings.drag_coefficient_increment =  0.05
        analyses[tag] = analysis

    return analyses

def SR_analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = SR_base_analysis(config)
        analyses[tag] = analysis

    return analyses

 
def TR_analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = TR_base_analysis(config)
        if config.networks.electric.propulsors['front_port_propulsor'].rotor.orientation_euler_angles[1] > 45*Units.degrees: 
            analysis.aerodynamics.settings.drag_coefficient_increment =  0.10
        elif config.networks.electric.propulsors['front_port_propulsor'].rotor.orientation_euler_angles[1] > 15*Units.degrees: 
            analysis.aerodynamics.settings.drag_coefficient_increment =  0.05
        analyses[tag] = analysis

    return analyses

def TR_base_analysis(vehicle): 

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle 

    # ------------------------------------------------------------------
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.vehicle                               = vehicle 
    geometry.settings.update_center_of_gravity     = True 
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Electric_VTOL()
    weights.aircraft_type = "VTOL"
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics         = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.maximum_lift_coefficient       =  1.5
    aerodynamics.settings.drag_coefficient_increment     =  0.01
    aerodynamics.settings.number_of_spanwise_vortices    =  12 # reducing the number of vortices to speed up the test
    aerodynamics.settings.number_of_chordwise_vortices   =  6  # reducing the number of vortices to speed up the test
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy()
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


def TW_base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    analyses.vehicle = vehicle

    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.unique_geometry = True
    analyses.append(geometry)
    
    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Electric_VTOL()   
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.maximum_lift_coefficient       =  1.5
    aerodynamics.settings.drag_coefficient_increment     =  0.01
    aerodynamics.settings.number_of_spanwise_vortices    =  12 # reducing the number of vortices to speed up the test
    aerodynamics.settings.number_of_chordwise_vortices   =  6  # reducing the number of vortices to speed up the test
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy()
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

def SR_base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    analyses.vehicle = vehicle
    
    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry() 
    geometry.settings.unique_geometry = True
    analyses.append(geometry)
    
    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Electric_VTOL()   
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.number_of_spanwise_vortices    =  12 # reducing the number of vortices to speed up the test
    aerodynamics.settings.number_of_chordwise_vortices   =  6  # reducing the number of vortices to speed up the test
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy()
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

# ----------------------------------------------------------------------
#   Define the Missions
# ----------------------------------------------------------------------
def TR_mission_setup(analyses): 
    
   
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment()
    base_segment.state.numerics.mission_solver.type = 'optimize'
    # Halved from the default 16 -- for a full-mission run-through, not accuracy. SLSQP's
    # finite-difference gradient costs ~(n_unknowns+1) objective evals per iteration, and
    # n_unknowns scales with control points, so this is a ~4x cut to gradient cost per
    # optimize-type segment (plus a cheaper per-eval trajectory integration on top).
    base_segment.state.numerics.number_of_control_points = 8


    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment                                            = Segments.Vertical_Flight.Climb(base_segment)
    segment.tag                                        = "Vertical_Climb"
    segment.analyses.extend(analyses.vertical_flight)
    segment.altitude_start                             = 0.0  * Units.ft
    segment.altitude_end                               = 50.  * Units.ft
    segment.climb_rate                                 = 300. * Units['ft/min']
    segment.initial_battery_conditions.state_of_charge = 1.0
    segment.true_course                                = 0   * Units.degree
    segment.state.numerics.mission_solver.type = 'root_finder'

    # define flight dynamics to model
    segment.flight_dynamics.force_z                    = True

    # define flight controls
    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                          'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.8]]

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #  First Transition Segment
    # ------------------------------------------------------------------
    segment                                               = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                                           = "departure_transition_1"
    segment.analyses.extend( analyses.transition_setting_1)
    # explicit altitude (matches Vertical_Climb's altitude_end) -- keeps this segment
    # well-posed whether or not Vertical_Climb precedes it
    segment.altitude                                      = 50 * Units.ft
    segment.air_speed_start                               = 15 * Units['mph']
    segment.air_speed_end                                 = 35 * Units['mph']
    segment.acceleration                                  = 0.2

    # square (2 unknowns: throttle, thrust_vector_angle vs 2 residuals:
    # force_x, force_z) -- root_finder converges in ~35s vs. optimize's
    # 511.9s SLSQP failure ("Singular matrix C in LSQ subproblem"). The
    # step_size/tolerance overrides previously here were SLSQP-specific
    # tuning and don't apply to fsolve, so they're dropped along with the
    # type change (fsolve uses Numerics.py's tighter defaults instead)
    segment.state.numerics.mission_solver.type                    = 'root_finder'
    # default max_evaluations (200) hits "maxfev" before converging on this segment;
    # 600 was the value found to work when this was debugged earlier
    segment.state.numerics.mission_solver.max_evaluations         = 600

    # define flight dynamics to model
    segment.flight_dynamics.force_x                       = True
    segment.flight_dynamics.force_z                       = True

    # define flight controls
    # bounds needed: unset defaults to -inf/+inf, letting SLSQP diverge
    segment.assigned_control_variables.throttle.active                                = True
    segment.assigned_control_variables.throttle.assigned_propulsors                   = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                          'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    segment.assigned_control_variables.throttle.bounds                                = [[0.0, 1.0]]
    # warm start near the known BEMT-converged value (~0.4499) -- the unset default of
    # 0.5 is close enough on its own, but thrust_vector_angle's default (below) isn't
    segment.assigned_control_variables.throttle.initial_guess_values                  = [[0.45]]

    segment.assigned_control_variables.thrust_vector_angle.active                     = True
    segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                          'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    segment.assigned_control_variables.thrust_vector_angle.bounds                     = [[0.0, 90.0 * Units.degrees]]
    # unset default guess is 0.5 rad (~28.6deg); the known-converged value is ~0.0384 rad
    # (~2.2deg) -- that gap is what made fsolve stall ("not making good progress")
    segment.assigned_control_variables.thrust_vector_angle.initial_guess_values       = [[2.5 * Units.degrees]]

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Departure Transition 2
    # ------------------------------------------------------------------
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "departure_transition_2"
    segment.analyses.extend(analyses.transition_setting_2)
    segment.climb_rate               = 728. * Units['ft/min']
    segment.air_speed_start          = 35 * Units['mph']
    segment.air_speed_end            = 100 * Units['mph']
    segment.altitude_end             = 500.0 * Units.ft
    segment.true_course              = 0 * Units.degree
    segment.state.numerics.mission_solver.type = 'root_finder'

    segment.flight_dynamics.force_x                       = True
    segment.flight_dynamics.force_z                       = True

    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                          'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    # BEMT-converged (prior full-mission run, solver-speedup branch): throttle 0.436-0.465, pitch_angle 4.79-10.05 deg.
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.45]]
    segment.assigned_control_variables.pitch_angle.active             = True
    segment.assigned_control_variables.pitch_angle.initial_guess_values = [[7.5 * Units.degrees]]

    mission.append_segment(segment)

    # ------------------------------------------------------------------------------------------------------------------------------------
    # Circular departure pattern
    # ------------------------------------------------------------------------------------------------------------------------------------
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Departure_Pattern_Curve"
    segment.analyses.extend( analyses.transition_setting_2 )
    segment.air_speed   = 90 * Units['knots']
    segment.turn_radius = 4000 * Units.feet
    segment.true_course = 0 * Units.degree
    segment.turn_angle  = 90 * Units.degree
    segment.altitude    = 500 * Units.feet
    segment.state.numerics.mission_solver.type = 'optimize'

    segment.flight_dynamics.force_x                                             = True
    segment.flight_dynamics.force_z                                             = True
    segment.flight_dynamics.force_y                                             = True

    # Placeholder wide-open bounds (carried over unresolved from the prior full-mission run on
    # solver-speedup) -- this segment previously ran fully unbounded and caused a multi-hour
    # hang; tighten once a real BEMT-converged trim is found for this geometry.
    segment.assigned_control_variables.throttle.active                          = True
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                          'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    segment.assigned_control_variables.pitch_angle.active                        = True
    # Sign of the turn (and hence bank) wasn't actually confirmed -- widened symmetric rather
    # than one-sided so a wrong sign guess can't rule out the true solution.
    segment.assigned_control_variables.bank_angle.active                        = True
    segment.assigned_control_variables.bank_angle.initial_guess_values          = [[20.0 * Units.degree]]

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Climb to Cruise Altitude/Speed -- split into two segments at 130mph (avoids SLSQP
    #  blowing up at the high-speed end of a single wide climb segment).
    # ------------------------------------------------------------------
    segment                           = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                       = "climb_1"
    segment.analyses.extend(analyses.cruise)
    segment.altitude_start            = 500.0 * Units.ft
    segment.climb_rate                = 300. * Units['ft/min']
    segment.air_speed_start           = 90.  * Units['knots']
    segment.air_speed_end             = 170.  * Units['mph']
    segment.altitude_end              = 700.0 * Units.ft
    segment.true_course               = 90 * Units.degree
    segment.state.numerics.mission_solver.type      = 'optimize'
    segment.state.numerics.mission_solver.step_size = 1E-2
    segment.state.numerics.mission_solver.tolerance = 1E-6
    segment.state.numerics.mission_solver.objective = None

    segment.flight_dynamics.force_x                       = True
    segment.flight_dynamics.force_z                       = True

    # Widened on purpose (no bounds/guess below) -- tighter bounds tuned during earlier LL work
    # stalled for 50+ min on the restored rotor under BEMT without going infeasible, just very
    # slow. Wider bounds give SLSQP more room to move each iteration.
    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                          'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    segment.assigned_control_variables.pitch_angle.active               = True

    segment.assigned_control_variables.thrust_vector_angle.active                     = True
    segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                        'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]

    segment.assigned_control_variables.blade_pitch_command.active                     = True
    segment.assigned_control_variables.blade_pitch_command.assigned_rotors            = [['front_port_rotor','front_starboard_rotor','outboard_port_rotor',
                                                                                        'outboard_starboard_rotor','rear_port_rotor','rear_starboard_rotor']]
    segment.assigned_control_variables.blade_pitch_command.bounds                     = [[5.0 * Units.degrees, 35.0 * Units.degrees]]
    segment.assigned_control_variables.blade_pitch_command.initial_guess_values       = [[26.0 * Units.degrees]]

    mission.append_segment(segment)

    segment                           = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                       = "climb_2"
    segment.analyses.extend(analyses.cruise)
    segment.altitude_start            = 700.0 * Units.ft
    segment.climb_rate                = 300. * Units['ft/min']
    segment.air_speed_start           = 130.  * Units['mph']
    segment.air_speed_end             = 170.  * Units['mph']
    segment.altitude_end              = 1000.0 * Units.ft
    segment.true_course               = 100 * Units.degree
    segment.state.numerics.mission_solver.type      = 'optimize'
    segment.state.numerics.mission_solver.step_size = 1E-2
    segment.state.numerics.mission_solver.tolerance = 1E-6
    segment.state.numerics.mission_solver.objective = None

    segment.flight_dynamics.force_x                       = True
    segment.flight_dynamics.force_z                       = True

    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                          'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    segment.assigned_control_variables.throttle.bounds               = [[0.05, 0.8]]
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.3]]

    segment.assigned_control_variables.pitch_angle.active               = True
    segment.assigned_control_variables.pitch_angle.bounds               = [[-5.0 * Units.degrees, 20.0 * Units.degrees]]
    segment.assigned_control_variables.pitch_angle.initial_guess_values = [[4.33 * Units.degrees]]

    segment.assigned_control_variables.thrust_vector_angle.active                     = True
    segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                        'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    segment.assigned_control_variables.thrust_vector_angle.bounds                     = [[10.0 * Units.degrees, 90.0 * Units.degrees]]
    segment.assigned_control_variables.thrust_vector_angle.initial_guess_values       = [[61.0 * Units.degrees]]

    segment.assigned_control_variables.blade_pitch_command.active                     = True
    segment.assigned_control_variables.blade_pitch_command.assigned_rotors            = [['front_port_rotor','front_starboard_rotor','outboard_port_rotor',
                                                                                        'outboard_starboard_rotor','rear_port_rotor','rear_starboard_rotor']]
    segment.assigned_control_variables.blade_pitch_command.bounds                     = [[5.0 * Units.degrees, 40.0 * Units.degrees]]
    segment.assigned_control_variables.blade_pitch_command.initial_guess_values       = [[24.0 * Units.degrees]]

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #  Cruise Segment
    # ------------------------------------------------------------------
    segment                                               = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "cruise"
    segment.analyses.extend( analyses.cruise)
    segment.air_speed                                     = 150 * Units['mph']
    segment.altitude                                      = 1000 *  Units.feet
    segment.throttle                                      = 0.33197

    # define flight dynamics to model
    segment.flight_dynamics.force_x                       = True
    segment.flight_dynamics.force_z                       = True

    # define flight controls
    segment.assigned_control_variables.pitch_angle
    segment.assigned_control_variables.pitch_angle.active             = True

    segment.assigned_control_variables.blade_pitch_command.active                     = True
    segment.assigned_control_variables.blade_pitch_command.assigned_rotors            =  [['front_port_rotor','front_starboard_rotor','outboard_port_rotor',
                                                                                           'outboard_starboard_rotor', 'rear_port_rotor','rear_starboard_rotor']]

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #    Descent Segment 1
    # ------------------------------------------------------------------
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "descent_1"
    segment.analyses.extend(analyses.cruise)
    segment.climb_rate               = -100. * Units['ft/min']
    segment.air_speed_start          = 170.  * Units['mph']
    segment.air_speed_end            = 130.  * Units['mph']
    segment.altitude_start           = 1000.0 * Units.ft
    segment.altitude_end             = 750.0 * Units.ft
    segment.true_course              = 90 * Units.degree
    segment.state.numerics.mission_solver.type = 'root_finder'

    segment.flight_dynamics.force_x                                             = True
    segment.flight_dynamics.force_z                                             = True

    segment.assigned_control_variables.throttle.active                          = True
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                        'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    # BEMT-converged (prior full-mission run, solver-speedup branch): throttle 0.262-0.325, pitch_angle 3.02-6.68 deg.
    segment.assigned_control_variables.throttle.initial_guess_values        = [[0.29]]
    segment.assigned_control_variables.pitch_angle.active                        = True
    segment.assigned_control_variables.pitch_angle.initial_guess_values        = [[4.7 * Units.degrees]]

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #    Descent Segment 2 -- 130 -> 115 mph, still comfortably above wing stall
    # ------------------------------------------------------------------
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "descent_2"
    segment.analyses.extend(analyses.cruise)
    segment.climb_rate               = -300. * Units['ft/min']
    segment.air_speed_start          = 130.  * Units['mph']
    segment.air_speed_end            = 115.  * Units['mph']
    segment.altitude_start           = 750.0 * Units.ft
    segment.altitude_end             = 600.0 * Units.ft
    segment.true_course              = 90 * Units.degree

    segment.state.numerics.mission_solver.type      = 'optimize'
    segment.state.numerics.mission_solver.step_size = 1E-2
    segment.state.numerics.mission_solver.tolerance = 1E-6
    segment.state.numerics.mission_solver.objective = None

    segment.flight_dynamics.force_x                       = True
    segment.flight_dynamics.force_z                       = True

    # Bounded -- unbounded throttle/pitch_angle here previously produced catastrophic mu
    # (up to 266, near-zero throttle -> near-zero omega).
    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                        'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    segment.assigned_control_variables.throttle.bounds               = [[0.06, 0.4]]
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.17]]
    segment.assigned_control_variables.pitch_angle.active             = True
    segment.assigned_control_variables.pitch_angle.bounds             = [[-2.0 * Units.degrees, 15.0 * Units.degrees]]
    segment.assigned_control_variables.pitch_angle.initial_guess_values = [[5.8 * Units.degrees]]
    segment.assigned_control_variables.thrust_vector_angle.active                     = True
    segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        =  [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                        'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    segment.assigned_control_variables.thrust_vector_angle.bounds                     = [[25.0 * Units.degrees, 85.0 * Units.degrees]]
    segment.assigned_control_variables.thrust_vector_angle.initial_guess_values       = [[56.0 * Units.degrees]]

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #    Descent Segment 3 -- 115mph -> 90kts, the stall-crossing tail; blade_pitch_command
    #    freed here since the wing alone can no longer carry the required CL by this speed.
    # ------------------------------------------------------------------
    segment                          = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "descent_3"
    segment.analyses.extend(analyses.cruise)
    segment.climb_rate               = -300. * Units['ft/min']
    segment.air_speed_start          = 115.  * Units['mph']
    segment.air_speed_end            = 90 * Units.kts
    segment.altitude_start           = 600.0 * Units.ft
    segment.altitude_end             = 500.0 * Units.ft
    segment.true_course              = 90 * Units.degree

    segment.state.numerics.mission_solver.type      = 'optimize'
    segment.state.numerics.mission_solver.step_size = 1E-2
    segment.state.numerics.mission_solver.tolerance = 1E-6
    segment.state.numerics.mission_solver.objective = None

    segment.flight_dynamics.force_x                       = True
    segment.flight_dynamics.force_z                       = True

    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                        'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    # BEMT-converged (prior full-mission run, solver-speedup branch): throttle 0.2965-0.3360, pitch_angle 4.18-4.45 deg.
    segment.assigned_control_variables.throttle.bounds               = [[0.15, 0.5]]
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.32]]
    segment.assigned_control_variables.pitch_angle.active             = True
    segment.assigned_control_variables.pitch_angle.bounds             = [[-2.0 * Units.degrees, 15.0 * Units.degrees]]
    segment.assigned_control_variables.pitch_angle.initial_guess_values = [[4.3 * Units.degrees]]

    segment.assigned_control_variables.thrust_vector_angle.active                     = True
    segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        =  [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                        'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    # BEMT-converged: 82.23-84.86 deg.
    segment.assigned_control_variables.thrust_vector_angle.bounds                     = [[65.0 * Units.degrees, 90.0 * Units.degrees]]
    segment.assigned_control_variables.thrust_vector_angle.initial_guess_values       = [[84.0 * Units.degrees]]

    segment.assigned_control_variables.blade_pitch_command.active                     = True
    segment.assigned_control_variables.blade_pitch_command.assigned_rotors            = [['front_port_rotor','front_starboard_rotor','outboard_port_rotor',
                                                                                        'outboard_starboard_rotor','rear_port_rotor','rear_starboard_rotor']]
    segment.assigned_control_variables.blade_pitch_command.bounds                     = [[2.0 * Units.degrees, 15.0 * Units.degrees]]
    segment.assigned_control_variables.blade_pitch_command.initial_guess_values       = [[4.5 * Units.degrees]]

    mission.append_segment(segment)

    #------------------------------------------------------------------------------------------------------------------------------------
    # Circular approach pattern
    #------------------------------------------------------------------------------------------------------------------------------------
    segment                                               = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag                                           = "Approach_Pattern_Curve"
    segment.analyses.extend( analyses.transition_setting_2 )
    segment.air_speed   = 90 * Units.kts
    segment.turn_radius = 4000 * Units.feet
    segment.true_course = 0 * Units.degree
    segment.turn_angle  = 90 * Units.degree
    segment.altitude    = 500 * Units.feet
    segment.state.numerics.mission_solver.type = 'optimize'

    segment.flight_dynamics.force_x                                             = True
    segment.flight_dynamics.force_z                                             = True
    segment.flight_dynamics.force_y                                             = True

    segment.assigned_control_variables.throttle.active                          = True
    segment.assigned_control_variables.throttle.assigned_propulsors             = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                        'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    segment.assigned_control_variables.throttle.bounds                          = [[0.05, 0.9]]
    segment.assigned_control_variables.throttle.initial_guess_values        = [[0.33]]
    segment.assigned_control_variables.pitch_angle.active                        = True
    segment.assigned_control_variables.pitch_angle.bounds                       = [[-5.0 * Units.degrees, 20.0 * Units.degrees]]
    segment.assigned_control_variables.pitch_angle.initial_guess_values         = [[4.5 * Units.degrees]]
    segment.assigned_control_variables.bank_angle.active                        = True
    segment.assigned_control_variables.bank_angle.bounds                        = [[-45.0 * Units.degree, 45.0 * Units.degree]]
    segment.assigned_control_variables.bank_angle.initial_guess_values          = [[-10.0 * Units.degree]]

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Arriving Transition 1
    # ------------------------------------------------------------------
    segment                          = Segments.Descent.Linear_Speed_Constant_Rate(base_segment)
    segment.tag                      = "arriving_transition_1"
    segment.analyses.extend(analyses.low_speed_transition)
    segment.descent_rate               = 728. * Units['ft/min']
    segment.air_speed_start            = 90 * Units.kts
    segment.air_speed_end              = 35 * Units['mph']
    segment.altitude_start             = 500.0 * Units.ft
    segment.altitude_end               = 50.0 * Units.ft
    segment.true_course                = 0 * Units.degree
    segment.state.numerics.mission_solver.step_size = 1E-2
    segment.state.numerics.mission_solver.tolerance = 1E-6
    segment.state.numerics.mission_solver.objective = None

    segment.flight_dynamics.force_x                       = True
    segment.flight_dynamics.force_z                       = True

    segment.assigned_control_variables.throttle.active                                = True
    segment.assigned_control_variables.throttle.assigned_propulsors                   = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                        'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    # BEMT-converged (prior full-mission run, solver-speedup branch): throttle 0.3924-0.5481, thrust_vector_angle
    # 22.19-24.53 deg, blade_pitch_command 22.71-26.86 deg.
    segment.assigned_control_variables.throttle.bounds                            = [[0.25, 0.7]]
    segment.assigned_control_variables.throttle.initial_guess_values              = [[0.48]]

    segment.assigned_control_variables.thrust_vector_angle.active                     = True
    segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                        'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    segment.assigned_control_variables.thrust_vector_angle.bounds                     = [[5.0 * Units.degrees, 45.0 * Units.degrees]]
    segment.assigned_control_variables.thrust_vector_angle.initial_guess_values       = [[23.75 * Units.degree]]

    segment.assigned_control_variables.blade_pitch_command.active                     = True
    segment.assigned_control_variables.blade_pitch_command.assigned_rotors            =  [['front_port_rotor','front_starboard_rotor','outboard_port_rotor',
                                                                                        'outboard_starboard_rotor','rear_port_rotor','rear_starboard_rotor']]
    segment.assigned_control_variables.blade_pitch_command.bounds                     = [[15.0 * Units.degrees, 32.0 * Units.degrees]]
    segment.assigned_control_variables.blade_pitch_command.initial_guess_values       = [[24.6 * Units.degrees]]

    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #  Arriving Transition 2 / Landing Transition
    # ------------------------------------------------------------------
    segment                                               = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                                           = "arriving_transition_2"
    segment.analyses.extend( analyses.low_speed_transition)
    segment.air_speed_start            = 35 * Units['mph']
    segment.air_speed_end              = 10 * Units['knots']
    segment.acceleration               = -1.0
    segment.true_course                = 0 * Units.degree
    segment.altitude                   = 50.0 * Units.ft
    segment.state.numerics.mission_solver.type      = 'optimize'
    segment.state.numerics.mission_solver.objective = None

    segment.flight_dynamics.force_x                       = True
    segment.flight_dynamics.force_z                       = True

    segment.assigned_control_variables.throttle.active                                = True
    segment.assigned_control_variables.throttle.assigned_propulsors                   = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                        'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    # BEMT-converged (prior full-mission run, solver-speedup branch): throttle 0.559-0.568, thrust_vector_angle 22.43-22.75 deg,
    # blade_pitch_command 22.85-22.97 deg, pitch_angle 3.03-3.05 deg.
    segment.assigned_control_variables.throttle.bounds                            = [[0.35, 0.75]]
    segment.assigned_control_variables.throttle.initial_guess_values              = [[0.56]]

    segment.assigned_control_variables.thrust_vector_angle.active                     = True
    segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                        'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    segment.assigned_control_variables.thrust_vector_angle.bounds                     = [[5.0 * Units.degrees, 40.0 * Units.degrees]]
    segment.assigned_control_variables.thrust_vector_angle.initial_guess_values       = [[22.6 * Units.degree]]

    segment.assigned_control_variables.blade_pitch_command.active                     = True
    segment.assigned_control_variables.blade_pitch_command.assigned_rotors            = [['front_port_rotor','front_starboard_rotor','outboard_port_rotor',
                                                                                        'outboard_starboard_rotor','rear_port_rotor','rear_starboard_rotor']]
    segment.assigned_control_variables.blade_pitch_command.bounds                     = [[15.0 * Units.degrees, 30.0 * Units.degrees]]
    segment.assigned_control_variables.blade_pitch_command.initial_guess_values       = [[22.9 * Units.degrees]]

    segment.assigned_control_variables.pitch_angle.active                             = True
    segment.assigned_control_variables.pitch_angle.bounds                             = [[-2.0 * Units.degrees, 15.0 * Units.degrees]]
    segment.assigned_control_variables.pitch_angle.initial_guess_values               = [[3.04 * Units.degrees]]

    mission.append_segment(segment)

    #------------------------------------------------------------------------------------------------------------------------------------
    # Vertical Descent
    #------------------------------------------------------------------------------------------------------------------------------------
    segment                                                         = Segments.Vertical_Flight.Climb(base_segment)
    segment.tag                                                     = "Vertical_Descent"
    segment.analyses.extend( analyses.vertical_flight)
    segment.altitude_end                                            = 0.   * Units.ft
    segment.climb_rate                                              = -300. * Units['ft/min']
    segment.true_course                                             = 0 * Units.degree
    segment.state.numerics.mission_solver.type = 'root_finder'

    segment.flight_dynamics.force_z                                  = True

    segment.assigned_control_variables.throttle.active               = True
    segment.assigned_control_variables.throttle.assigned_propulsors  =[['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                        'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]
    # BEMT-converged (prior full-mission run, solver-speedup branch): 0.4793-0.4796 -- close to hover's own ~0.487.
    segment.assigned_control_variables.throttle.initial_guess_values = [[0.48]]

    mission.append_segment(segment)


    return mission


def TW_mission_setup(analyses ): 

     # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment()
  
    # ------------------------------------------------------------------
    # Vertical Climb 
    # ------------------------------------------------------------------ 
    segment                                                          = Segments.Vertical_Flight.Climb(base_segment)
    segment.tag                                                      = "Vertical_Climb"
    segment.state.numerics.mission_solver.type                       = "root_finder"
    segment.analyses.extend(analyses.vertical_climb)
    segment.altitude_start                                           = 0  * Units.ft
    segment.altitude_end                                             = 100.  * Units.ft
    segment.climb_rate                                               = 300. * Units['ft/min']  
    segment.initial_battery_conditions.state_of_charge               = 1.0

    # define flight dynamics to model  
    segment.flight_dynamics.force_z                                  = True 

    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                                         'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]
    
    mission.append_segment(segment)     


    # ------------------------------------------------------------------
    #   Hover 
    # ------------------------------------------------------------------ 
    segment                                                          = Segments.Vertical_Flight.Hover(base_segment)
    segment.tag                                                      = "Hover"   
    segment.analyses.extend(analyses.vertical_climb)

    segment.state.numerics.mission_solver.type                               = "root_finder"    
    segment.altitude                                                 = 100.0  * Units.ft   
                        
    # define flight dynamics to model              
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                                         'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]
      
    mission.append_segment(segment)
 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Vertical Descent 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                                         = Segments.Vertical_Flight.Descent(base_segment)
    segment.tag                                                     = "Vertical_Descent"
    segment.state.numerics.mission_solver.type                      = "root_finder"
    segment.analyses.extend( analyses.vertical_descent)
    segment.altitude_start                                          = 100.0 * Units.ft   
    segment.altitude_end                                            = 0.   * Units.ft  
    segment.descent_rate                                            = 300. * Units['ft/min']   
                  
    # define flight dynamics to model              
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3','prop_rotor_propulsor_4',
                                                                             'prop_rotor_propulsor_5','prop_rotor_propulsor_6','prop_rotor_propulsor_7','prop_rotor_propulsor_8']]  
            
    mission.append_segment(segment)      
    return mission

def SR_mission_setup(analyses,vehicle): 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission     = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'baseline_mission' 
    
    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments

    # base segment           
    base_segment  = Segments.Segment()    
     
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Vertical Climb 
    #------------------------------------------------------------------------------------------------------------------------------------  
    segment     = Segments.Vertical_Flight.Climb(base_segment)
    segment.tag = "Vertical_Climb"   
    segment.analyses.extend( analyses.vertical_flight )  
    segment.altitude_start                                = 0.0  * Units.ft  
    segment.altitude_end                                  = 200.  * Units.ft   
    segment.initial_battery_conditions.state_of_charge    = 1.0 
    segment.climb_rate                                    = 500. * Units['ft/min']   
    segment.state.numerics.mission_solver.type                    = "root_finder"
            
    # define flight dynamics to model  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['lift_propulsor_1','lift_propulsor_2','lift_propulsor_3','lift_propulsor_4',
                                                                         'lift_propulsor_5','lift_propulsor_6','lift_propulsor_7','lift_propulsor_8']] 
       
    mission.append_segment(segment)
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Low-Speed Transition
    #------------------------------------------------------------------------------------------------------------------------------------  
 
    segment                                               = Segments.Cruise.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    segment.tag                                           = "Low_Speed_Transition"  
    segment.analyses.extend( analyses.transition_flight )   
    segment.altitude                                      = 200.  * Units.ft           
    segment.air_speed_start                               = 500. * Units['ft/min']
    segment.air_speed_end                                 = 80 *  Units.mph
    segment.acceleration                                  = 1.5
    segment.pitch_initial                                 = 0.0 * Units.degrees
    segment.pitch_final                                   = 2.  * Units.degrees 
    segment.state.numerics.mission_solver.type                    = "root_finder"

    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['cruise_propulsor_1','cruise_propulsor_2'],
                                                             ['lift_propulsor_1','lift_propulsor_2','lift_propulsor_3','lift_propulsor_4',
                                                            'lift_propulsor_5','lift_propulsor_6','lift_propulsor_7','lift_propulsor_8']] 
    mission.append_segment(segment) 
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # High-Speed Climbing Transition 
    #------------------------------------------------------------------------------------------------------------------------------------  
    segment                                               = Segments.Climb.Constant_Acceleration_Constant_Pitchrate_Constant_Angle(base_segment)
    segment.tag                                           = "High_Speed_Climbing_Transition"
    segment.state.numerics.mission_solver.type             = "root_finder"
    segment.analyses.extend( analyses.transition_flight)
    segment.altitude_start                                = 200.0 * Units.ft   
    segment.altitude_end                                  = 500.0 * Units.ft 
    segment.climb_angle                                   = 3     * Units.degrees   
    segment.acceleration                                  = 0.25  * Units['m/s/s'] 
    segment.pitch_initial                                 = 2.    * Units.degrees 
    segment.pitch_final                                   = 7.    * Units.degrees   


    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['cruise_propulsor_1','cruise_propulsor_2'],
                                                             ['lift_propulsor_1','lift_propulsor_2','lift_propulsor_3','lift_propulsor_4',
                                                            'lift_propulsor_5','lift_propulsor_6','lift_propulsor_7','lift_propulsor_8']]
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
