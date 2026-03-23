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
    # make true only when resizing aircraft. should be left false for regression
    update_regression_values = False
    
    # TEST 1
    tiltrotor_transition_test(update_regression_values)
     
    # TEST 2
    tiltwing_transition_test(update_regression_values)
    
    # TEST 3
    stopped_rotor_transition_test(update_regression_values)
    
    return

def tiltrotor_transition_test(update_regression_values): 
         
    vehicle  = TR_vehicle_setup(redesign_rotors=update_regression_values)  
        
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
      
    tf                   = time.time()
    elapsed_time         = round((tf-ti)/60,2)
    print('Simulation Time: ' + str(elapsed_time) + ' mins')      
    
    #print values for resetting regression
    show_vals = True
    if show_vals:
        data = [ hover_throttle,cruise_rpm ]
        for val in data:
            print(val)
    
    # Truth values 
    hover_throttle_truth    = 0.5961266565749898
    cruise_rpm_truth        = 396.22413812373867
    
    # Store errors 
    error = Data() 
    error.hover_throttle  = np.max(np.abs( hover_throttle_truth - hover_throttle )/ hover_throttle_truth )
    error.cruise_rpm      = np.max(np.abs( cruise_rpm_truth - cruise_rpm  )/ cruise_rpm_truth )
    
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
    
    #print values for resetting regression
    show_vals = True
    if show_vals:
        data = [ hover_throttle,  vertical_climb_1_throttle , vertical_descent_throttle ]
        for val in data:
            print(val)
    
    # Truth values 
    hover_throttle_truth              = 0.7336052390182969
    vertical_climb_1_throttle_truth   = 0.7436815431612642
    vertical_descent_throttle_truth   = 0.723131676668686
    
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
    
    #print values for resetting regression
    show_vals = True
    if show_vals:
        data = [ hover_throttle, lst_throttle , hsct_throttle]
        for val in data:
            print(val)
    
    # Truth values 
    hover_throttle_truth  = 0.5574925738438762
    lst_throttle_truth    = 0.5406669816914917
    hsct_throttle_truth   = 0.43935354191548714
    
    # Store errors 
    error = Data() 
    error.hover_throttle = np.max(np.abs( hover_throttle_truth  - hover_throttle  )/ hover_throttle_truth )
    error.lst_throttle   = np.max(np.abs( lst_throttle_truth    - lst_throttle    )/ lst_throttle_truth   ) 
    error.hsct_throttle  = np.max(np.abs( hsct_throttle_truth   - hsct_throttle   )/ hsct_throttle_truth  )
 
    print('Errors:')
    print(error)
     
    for k,v in list(error.items()):
        assert(np.abs(v)<1.5e-1)   # lower tolerance due to lose bounds on prop-rotor blade design 
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
    aerodynamics.settings.maximum_lift_coefficient   =  1.5 
    aerodynamics.settings.drag_coefficient_increment =  0.01  
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
    aerodynamics.settings.maximum_lift_coefficient   =  1.5 
    aerodynamics.settings.drag_coefficient_increment =  0.01   
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
    base_segment.state.numerics.solver.type = 'optimize' 

    
    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                            = Segments.Vertical_Flight.Climb(base_segment)
    segment.tag                                        = "Vertical_Climb"   
    segment.analyses.extend(analyses.vertical_flight) 
    segment.altitude_start                             = 0.0  * Units.ft  
    segment.altitude_end                               = 50.  * Units.ft   
    segment.climb_rate                                 = 300. * Units['ft/min'] 
    segment.initial_battery_state_of_charge            = 1.0 
    segment.true_course                                = 0   * Units.degree  
    segment.state.numerics.solver.type = 'root_finder' 

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
    segment.air_speed_start                               = 15 * Units['mph']    
    segment.air_speed_end                                 = 35 * Units['mph']     
    segment.acceleration                                  = 0.2
    
    
    segment.state.numerics.solver.type                    = 'optimize' 
    segment.state.numerics.solver.step_size               = 1E-3 
    segment.state.numerics.solver.tolerance_solution      = 1E-2 
    segment.state.numerics.solver.objective               = None 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active                                = True           
    segment.assigned_control_variables.throttle.assigned_propulsors                   = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                          'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']] 
    
    segment.assigned_control_variables.thrust_vector_angle.active                     = True        
    segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        = [['front_port_propulsor','front_starboard_propulsor','outboard_port_propulsor',
                                                                                          'outboard_starboard_propulsor','rear_port_propulsor','rear_starboard_propulsor']]  
     
    mission.append_segment(segment)
    

    # ------------------------------------------------------------------
    #  First Transition Segment
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
    segment.assigned_control_variables.body_angle                   
    segment.assigned_control_variables.body_angle.active             = True                
           
    segment.assigned_control_variables.blade_pitch_command.active                     = True        
    segment.assigned_control_variables.blade_pitch_command.assigned_rotors            =  [['front_port_rotor','front_starboard_rotor','outboard_port_rotor',
                                                                                           'outboard_starboard_rotor', 'rear_port_rotor','rear_starboard_rotor']]

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
    segment.analyses.extend(analyses.vertical_climb)                
    segment.altitude_start                                           = 0  * Units.ft  
    segment.altitude_end                                             = 100.  * Units.ft   
    segment.climb_rate                                               = 300. * Units['ft/min']  
    segment.initial_battery_state_of_charge                          = 1.0 

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

    segment.state.numerics.solver.type                               = "root_finder"    
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
    segment.initial_battery_state_of_charge               = 1.0 
    segment.climb_rate                                    = 500. * Units['ft/min']   
    segment.state.numerics.solver.type                    = "root_finder"
            
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
    segment.state.numerics.solver.type                    = "root_finder"

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
