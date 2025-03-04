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

# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
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
    tiltwing_transition_test(update_regression_values)
    
    # TEST 2
    stopped_rotor_transition_test(update_regression_values)
    
    return 

def tiltwing_transition_test(update_regression_values):    
    TW_vehicle  = TW_vehicle_setup(update_regression_values)  
        
    # Set up configs
    TW_configs  = TW_configs_setup(TW_vehicle)

    # vehicle analyses
    TW_analyses = TW_analyses_setup(TW_configs)

    # mission analyses
    TW_mission  = TW_mission_setup(TW_analyses)
    TW_missions = TW_missions_setup(TW_mission) 
     
    TW_results = TW_missions.base_mission.evaluate()  
    
    # Extract sample values from computation    
    hover_throttle            = TW_results.segments.hover.conditions.energy['lift_rotor_propulsor_1'].throttle[1][0]
    vertical_climb_1_throttle = TW_results.segments.vertical_climb_1.conditions.energy['lift_rotor_propulsor_1'].throttle[1][0] 
    vertical_descent_throttle = TW_results.segments.vertical_descent.conditions.energy['lift_rotor_propulsor_1'].throttle[1][0] 
    
    #print values for resetting regression
    show_vals = True
    if show_vals:
        data = [ hover_throttle,  vertical_climb_1_throttle , vertical_descent_throttle ]
        for val in data:
            print(val)
    
    # Truth values 
    hover_throttle_truth              = 0.6032731206904142
    vertical_climb_1_throttle_truth   = 0.606892139245807
    vertical_descent_throttle_truth   = 0.5965847517907135
    
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
    SR_missions = SR_missions_setup(SR_mission) 
     
    SR_results = SR_missions.base_mission.evaluate()  
    
    # Extract sample values from computation    
    hover_throttle     = SR_results.segments.vertical_climb.conditions.energy['lift_propulsor_1'].throttle[1][0]
    lst_throttle       = SR_results.segments.low_speed_transition.conditions.energy['lift_propulsor_1'].throttle[1][0] 
    hsct_throttle      = SR_results.segments.high_speed_climbing_transition.conditions.energy['lift_propulsor_1'].throttle[1][0] 
    
    #print values for resetting regression
    show_vals = True
    if show_vals:
        data = [ hover_throttle, lst_throttle , hsct_throttle]
        for val in data:
            print(val)
    
    # Truth values 
    hover_throttle_truth  = 0.650918038833368
    lst_throttle_truth    = 0.6359963559059645
    hsct_throttle_truth   = 0.5475420672366067
    
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
        analyses[tag] = analysis

    return analyses

def SR_analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = SR_base_analysis(config)
        analyses[tag] = analysis

    return analyses

def TW_base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    
    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Electric()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)   

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle  = vehicle 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    # done!
    return analyses    
 
def SR_base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    
    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Electric()
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle = vehicle
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)   

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle  = vehicle 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    # done!
    return analyses    

def TW_mission_setup(analyses ): 

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment() 
    base_segment.state.numerics.number_of_control_points    = 3 
   

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                                          = Segments.Vertical_Flight.Hover(base_segment)
    segment.tag                                                      = "Hover"   
    segment.analyses.extend(analyses.vertical_climb)
    
    segment.altitude                                                 = 40.  * Units.ft  
    segment.initial_battery_state_of_charge                          = 1.0 
                        
    # define flight dynamics to model              
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['lift_rotor_propulsor_1','lift_rotor_propulsor_2','lift_rotor_propulsor_3','lift_rotor_propulsor_4',
                                                            'lift_rotor_propulsor_5','lift_rotor_propulsor_6','lift_rotor_propulsor_7','lift_rotor_propulsor_8']]
      
    mission.append_segment(segment)
 
 

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------ 
    segment                                                          = Segments.Vertical_Flight.Climb(base_segment)
    segment.tag                                                      = "Vertical_Climb_1"   
    segment.analyses.extend(analyses.vertical_climb)                
    segment.altitude_end                                             = 60.  * Units.ft  
    segment.initial_battery_state_of_charge                          = 1.0 
    segment.climb_rate                                               = 100. * Units['ft/min'] 
          
    # define flight dynamics to model            
    segment.flight_dynamics.force_z                                  = True 

    # define flight controls  
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['lift_rotor_propulsor_1','lift_rotor_propulsor_2','lift_rotor_propulsor_3','lift_rotor_propulsor_4',
                                                            'lift_rotor_propulsor_5','lift_rotor_propulsor_6','lift_rotor_propulsor_7','lift_rotor_propulsor_8']]
    
    mission.append_segment(segment)   
    

    #------------------------------------------------------------------------------------------------------------------------------------ 
    # Vertical Descent 
    #------------------------------------------------------------------------------------------------------------------------------------ 
    segment                                                         = Segments.Vertical_Flight.Descent(base_segment)
    segment.tag                                                     = "Vertical_Descent" 
    segment.analyses.extend( analyses.vertical_descent)               
    segment.altitude_start                                          = 100.0 * Units.ft   
    segment.altitude_end                                            = 0.   * Units.ft  
    segment.descent_rate                                            = 200. * Units['ft/min']  
                  
    # define flight dynamics to model              
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['lift_rotor_propulsor_1','lift_rotor_propulsor_2','lift_rotor_propulsor_3','lift_rotor_propulsor_4',
                                                                             'lift_rotor_propulsor_5','lift_rotor_propulsor_6','lift_rotor_propulsor_7','lift_rotor_propulsor_8']]  
            
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
    # VSTALL Calculation  
    vehicle_mass   = vehicle.mass_properties.max_takeoff
    reference_area = vehicle.reference_area 
    Vstall         = estimate_stall_speed(vehicle_mass,reference_area,altitude = 0.0,maximum_lift_coefficient = 1.2)      
     
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
 
    segment                                               = Segments.Transition.Constant_Acceleration_Constant_Pitchrate_Constant_Altitude(base_segment)
    segment.tag                                           = "Low_Speed_Transition"  
    segment.analyses.extend( analyses.transition_flight )   
    segment.altitude                                      = 200.  * Units.ft           
    segment.air_speed_start                               = 500. * Units['ft/min']
    segment.air_speed_end                                 = 0.75 * Vstall
    segment.acceleration                                  = 1.5
    segment.pitch_initial                                 = 0.0 * Units.degrees
    segment.pitch_final                                   = 2.  * Units.degrees    

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
    segment                                               = Segments.Transition.Constant_Acceleration_Constant_Angle_Linear_Climb(base_segment)
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

def TW_missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions

def SR_missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions 


if __name__ == '__main__': 
    main()    
    plt.show()
