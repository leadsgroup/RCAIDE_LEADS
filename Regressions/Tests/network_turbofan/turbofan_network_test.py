# turbofan_network_test.py
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

# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Boeing_737    import vehicle_setup as vehicle_setup
from Boeing_737    import configs_setup as configs_setup 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(): 
    # -----------------------------------------
    # Multi-Point Mission Setup 
    # -----------------------------------------

    # vehicle data
    vehicle  = vehicle_setup()
    
    # plot vehicle 
    plot_3d_vehicle(vehicle, 
                    min_x_axis_limit            = 0,
                    max_x_axis_limit            = 40,
                    min_y_axis_limit            = -20,
                    max_y_axis_limit            = 20,
                    min_z_axis_limit            = -20,
                    max_z_axis_limit            = 20,
                    show_figure                 = False 
                    )        
    
    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = analyses_setup(configs)

    # mission analyses 
    mission = mission_setup(analyses)
    
    # create mission instances (for multiple types of missions)
    missions = missions_setup(mission) 
     
    # mission analysis 
    results = missions.base_mission.evaluate()   
 
    # Extract sample values from computation  
    takeoff_thrust     = results.segments.takeoff.conditions.energy['port_propulsor'].thrust[3][0]
    climb_throttle_1   = results.segments.climb_1.conditions.energy['port_propulsor'].throttle[3][0]
    climb_throttle_2   = results.segments.climb_2.conditions.energy['port_propulsor'].throttle[3][0]
    climb_throttle_3   = results.segments.climb_3.conditions.energy['port_propulsor'].throttle[3][0]
    climb_throttle_4   = results.segments.climb_4.conditions.energy['port_propulsor'].throttle[3][0]
    climb_throttle_5   = results.segments.climb_5.conditions.energy['port_propulsor'].throttle[3][0]
    climb_throttle_6   = results.segments.climb_6.conditions.energy['port_propulsor'].throttle[3][0]
    climb_throttle_7   = results.segments.climb_7.conditions.energy['port_propulsor'].throttle[3][0] 
    climb_throttle_8   = results.segments.climb_8.conditions.energy['port_propulsor'].throttle[3][0] 
    climb_throttle_9   = results.segments.climb_9.conditions.energy['port_propulsor'].throttle[3][0]  
    climb_10_CL        = results.segments.climb_10.conditions.aerodynamics.coefficients.lift.total[2][0]
    cruise_CL_1        = results.segments.cruise_1.conditions.aerodynamics.coefficients.lift.total[2][0]
    cruise_CL_2        = results.segments.cruise_2.conditions.aerodynamics.coefficients.lift.total[2][0]
    cruise_CL_3        = results.segments.cruise_3.conditions.aerodynamics.coefficients.lift.total[2][0] 
    descent_throttle_1 = results.segments.descent_1.conditions.energy['port_propulsor'].throttle[3][0]
    curved_cruise_CL   = results.segments.curved_cruise.conditions.aerodynamics.coefficients.lift.total[2][0] 
    descent_throttle_2 = results.segments.descent_2.conditions.energy['port_propulsor'].throttle[3][0]
    single_pt_CL_1     = results.segments.single_point_1.conditions.aerodynamics.coefficients.lift.total[0][0]
    single_pt_CL_2     = results.segments.single_point_2.conditions.aerodynamics.coefficients.lift.total[0][0]     
    cruise_4_CL        = results.segments.cruise_4.conditions.aerodynamics.coefficients.lift.total[2][0]  
    cruise_5_CL        = results.segments.cruise_5.conditions.aerodynamics.coefficients.lift.total[2][0] 
    cruise_6_CL        = results.segments.cruise_6.conditions.aerodynamics.coefficients.lift.total[2][0]    
    cruise_7_CL        = results.segments.cruise_7.conditions.aerodynamics.coefficients.lift.total[2][0]   
    cruise_8_CL        = results.segments.cruise_8.conditions.aerodynamics.coefficients.lift.total[2][0]
    descent_throttle_3 = results.segments.descent_3.conditions.energy['port_propulsor'].throttle[3][0]
    landing_thrust     = results.segments.landing.conditions.energy['port_propulsor'].thrust[3][0]
    
    #print values for resetting regression
    show_vals = True
    if show_vals:
        data = [takeoff_thrust, climb_throttle_1,   climb_throttle_2,   climb_throttle_3,   climb_throttle_4,   climb_throttle_5,  
                climb_throttle_6,   climb_throttle_7,   climb_throttle_8,   climb_throttle_9,   climb_10_CL,  
                cruise_CL_1,  cruise_CL_2,  cruise_CL_3,   descent_throttle_1,  curved_cruise_CL, descent_throttle_2,
                single_pt_CL_1,     single_pt_CL_2,     cruise_4_CL,   cruise_5_CL, cruise_6_CL,cruise_7_CL,cruise_8_CL, 
                descent_throttle_3,  landing_thrust]
        for val in data:
            print(val)
    
    # Truth values
    takeoff_thrust_truth     = 99134.30160723036
    climb_throttle_1_truth   = 1.395857698432127
    climb_throttle_2_truth   = 1.0684496813043596
    climb_throttle_3_truth   = 0.5
    climb_throttle_4_truth   = 0.8680810458622283
    climb_throttle_5_truth   = 0.8753486891710588
    climb_throttle_6_truth   = 1.1668519049114097
    climb_throttle_7_truth   = 1.3116873356066023
    climb_throttle_8_truth   = 0.6075723981460155
    climb_throttle_9_truth   = 0.8790351469765603
    climb_10_CL_truth        = 1.3872932460517462
    cruise_CL_1_truth        = 0.6738526682166986
    cruise_CL_2_truth        = 0.6691390399512774
    cruise_CL_3_truth        = 0.45025703310784687
    descent_throttle_1_truth = 0.11727849329204963
    curved_cruise_CL_truth   = 1.2889478665706608
    descent_throttle_2_truth = 0.5
    single_pt_CL_1_truth     = 0.0005711272577742668
    single_pt_CL_2_truth     = 0.0006616284937706831
    cruise_4_CL_truth        = 0.4798716853018943
    cruise_5_CL_truth        = 0.4798655832747957
    cruise_6_CL_truth        = 0.3238648058769196
    cruise_7_CL_truth        = 0.31458835547867875
    cruise_8_CL_truth        = 0.3064281877230094
    descent_throttle_3_truth = 0.14323028432387935
    landing_thrust_truth     = 10914.922642740088
    
    # Store errors 
    error = Data()
    error.takeoff_thrust     = np.max(np.abs(takeoff_thrust       - takeoff_thrust_truth))  
    error.climb_throttle_1   = np.max(np.abs(climb_throttle_1     - climb_throttle_1_truth))  
    error.climb_throttle_2   = np.max(np.abs(climb_throttle_2     - climb_throttle_2_truth))   
    error.climb_throttle_3   = np.max(np.abs(climb_throttle_3     - climb_throttle_3_truth))   
    error.climb_throttle_4   = np.max(np.abs(climb_throttle_4     - climb_throttle_4_truth))   
    error.climb_throttle_5   = np.max(np.abs(climb_throttle_5     - climb_throttle_5_truth))   
    error.climb_throttle_6   = np.max(np.abs(climb_throttle_6     - climb_throttle_6_truth))   
    error.climb_throttle_7   = np.max(np.abs(climb_throttle_7     - climb_throttle_7_truth))   
    error.climb_throttle_8   = np.max(np.abs(climb_throttle_8     - climb_throttle_8_truth))  
    error.climb_throttle_9   = np.max(np.abs(climb_throttle_9     - climb_throttle_9_truth))  
    error.climb_10_CL        = np.max(np.abs(climb_10_CL          - climb_10_CL_truth ))    
    error.cruise_CL_1        = np.max(np.abs(cruise_CL_1          - cruise_CL_1_truth ))     
    error.cruise_CL_2        = np.max(np.abs(cruise_CL_2          - cruise_CL_2_truth ))      
    error.cruise_CL_3        = np.max(np.abs(cruise_CL_3          - cruise_CL_3_truth ))     
    error.descent_throttle_1 = np.max(np.abs(descent_throttle_1   - descent_throttle_1_truth)) 
    error.curved_cruise_CL   = np.max(np.abs(curved_cruise_CL     - curved_cruise_CL_truth))
    error.descent_throttle_2 = np.max(np.abs(descent_throttle_2   - descent_throttle_2_truth))
    error.single_pt_CL_1     = np.max(np.abs(single_pt_CL_1       - single_pt_CL_1_truth ))     
    error.single_pt_CL_2     = np.max(np.abs(single_pt_CL_2       - single_pt_CL_2_truth ))  
    error.cruise_4_CL        = np.max(np.abs(cruise_4_CL         - cruise_4_CL_truth))      
    error.cruise_5_CL        = np.max(np.abs(cruise_5_CL         - cruise_5_CL_truth))   
    error.cruise_6_CL        = np.max(np.abs(cruise_6_CL         - cruise_6_CL_truth ))      
    error.cruise_7_CL        = np.max(np.abs(cruise_7_CL         - cruise_7_CL_truth ))      
    error.cruise_8_CL        = np.max(np.abs(cruise_8_CL         - cruise_8_CL_truth ))         
    error.descent_throttle_3 = np.max(np.abs(descent_throttle_3   - descent_throttle_3_truth))  
    error.landing_thrust     = np.max(np.abs(landing_thrust       - landing_thrust_truth))
     
    print('Errors:')
    print(error)
     
    for k,v in list(error.items()): 
        assert(np.abs(v)<1e-6)
        
    plot_results(results)
    return 
# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in list(configs.items()):
        analysis = base_analysis(config)
        analyses[tag] = analysis
 
    return analyses


def base_analysis(vehicle):

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
    aerodynamics                                        = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.vehicle                                = vehicle
    aerodynamics.settings.number_of_spanwise_vortices   = 5
    aerodynamics.settings.number_of_chordwise_vortices  = 2       
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
    base_segment.state.numerics.number_of_control_points = 4  
 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start                                           = 100.* Units.knots
    segment.velocity_end                                             = 150 * Units.knots
    segment.friction_coefficient                                     = 0.04
    segment.altitude                                                 = 0.0   
    mission.append_segment(segment)
    

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #    Climb 1 : Constant Speed Constant Rate  
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1" 
    segment.analyses.extend( analyses.takeoff )  
    segment.altitude_end                                             = 1.0  * Units.km
    segment.air_speed                                                = 150   * Units.knots
    segment.climb_rate                                               = 10.0  * Units['m/s'] 
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

   
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Climb 2 : Constant Dynamic Pressure Constant Angle 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Climb.Constant_Dynamic_Pressure_Constant_Angle(base_segment)
    segment.tag = "climb_2"
    segment.analyses.extend( analyses.base )  
    segment.altitude_end                                             = 2.    * Units.km
    segment.climb_angle                                              = 5.   * Units.degrees 
    segment.dynamic_pressure                                         = 3800 * Units.pascals  
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    segment.assigned_control_variables.altitude.active               = True 
      
    mission.append_segment(segment)

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Climb 3 : Constant Dynamic Pressure Constant Rate 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Climb.Constant_Dynamic_Pressure_Constant_Rate(base_segment)
    segment.tag = "climb_3"
    segment.analyses.extend( analyses.base )  
    segment.altitude_end                                             = 3.   * Units.km
    segment.climb_rate                                               = 730. * Units['ft/min']    
    segment.dynamic_pressure                                         = 12000 * Units.pascals
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
      
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Climb 4 : Constant Mach Constant Angle 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Climb.Constant_Mach_Constant_Angle(base_segment)
    segment.tag = "climb_4"
    segment.analyses.extend( analyses.base )  
    segment.altitude_start                                           = 3.   * Units.km
    segment.altitude_end                                             = 4.   * Units.km
    segment.mach_number                                              = 0.5
    segment.climb_angle                                              = 3.5 * Units.degrees  
              
    # define flight dynamics to model           
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
      
    mission.append_segment(segment)

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Climb 5 : 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Climb.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "climb_5"
    segment.analyses.extend( analyses.base )  
    segment.altitude_end                                             = 5.    * Units.km
    segment.air_speed                                                = 200   * Units.m / Units.s
    segment.climb_angle                                              = 3.5 * Units.degrees 
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
       
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Climb 6 : 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Climb.Constant_Mach_Linear_Altitude(base_segment)
    segment.tag = "climb_6"
    segment.analyses.extend( analyses.base )  
    segment.altitude_end                                             = 6. * Units.km   
    segment.mach_number                                              = 0.75   
                                                           
    # define flight dynamics to model                      
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Climb 7 : Constant Speed Linear Altitude 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Climb.Constant_Speed_Linear_Altitude(base_segment)
    segment.tag = "climb_7"
    segment.analyses.extend( analyses.base )  
    segment.altitude_end                                             = 7.    * Units.km   
    segment.air_speed                                                = 250.2 * Units.m / Units.s 
                 
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    mission.append_segment(segment)
 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Climb 8 : Constant EAS Constant Rate 
    # ------------------------------------------------------------------------------------------------------------------------------------  
    segment = Segments.Climb.Constant_EAS_Constant_Rate(base_segment)
    segment.tag = "climb_8"
    segment.analyses.extend( analyses.base )  
    segment.altitude_end                                            = 8.   * Units.km    
    segment.equivalent_air_speed                                    = 150. * Units.m / Units.s
    segment.climb_rate                                              = 1.   
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                 = True  
    segment.flight_dynamics.force_z                                 = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active              = True           
    segment.assigned_control_variables.throttle.assigned_propulsors = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active            = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Climb 9 : Constant EAS Constant Rate 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Climb.Constant_CAS_Constant_Rate(base_segment)
    segment.tag = "climb_9"
    segment.analyses.extend( analyses.base )  
    segment.altitude_end                                            = 10.9 * Units.km    
    segment.calibrated_air_speed                                    = 150. * Units.m / Units.s
    segment.climb_rate                                              = 1.  
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                 = True  
    segment.flight_dynamics.force_z                                 = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active              = True           
    segment.assigned_control_variables.throttle.assigned_propulsors = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active            = True                
     
    mission.append_segment(segment) 


    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Climb 10 : Constant EAS Constant Rate 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "climb_10"
    segment.analyses.extend( analyses.base )  
    segment.altitude_end                                                 = 11.   * Units.km    
    segment.air_speed                                                    = 150. * Units.m / Units.s
    segment.throttle                                                     = 0.55 
                     
    # define flight dynamics to model                  
    segment.flight_dynamics.force_x                                      = True  
    segment.flight_dynamics.force_z                                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.wind_angle.active                 = True  
    segment.assigned_control_variables.wind_angle.initial_guess          = True 
    segment.assigned_control_variables.wind_angle.initial_guess_values   = [[ 1.0 * Units.deg]] 
    segment.assigned_control_variables.body_angle.active                 = True               
    segment.assigned_control_variables.body_angle.initial_guess          = True  
    segment.assigned_control_variables.body_angle.initial_guess_values   = [[ 5.0 * Units.deg]]   
     
    mission.append_segment(segment) 
    
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Cruise Segment 1: constant Speed, constant altitude
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Dynamic_Pressure_Constant_Altitude(base_segment)
    segment.tag = "cruise_1" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                                 = 11. * Units.km    
    segment.dynamic_pressure                                         = 28000 * Units.pascals   
    segment.distance                                                 = 500 * Units.km  
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)    

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Cruise Segment 2: Constant Throttle Constant Altltude
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Throttle_Constant_Altitude(base_segment)
    segment.tag = "cruise_2" 
    segment.analyses.extend(analyses.base)  
    segment.state.numerics.number_of_control_points                        = 32 
    segment.altitude                                                       = 11. * Units.km  
    segment.air_speed_end                                                  = 215 * Units.m / Units.s  
    segment.throttle                                                       = 0.75
    segment.distance                                                       = 200 * Units.km  
                     
    # define flight dynamics to model                  
    segment.flight_dynamics.force_x                                        = True  
    segment.flight_dynamics.force_z                                        = True     
          
    # define flight controls       
    segment.assigned_control_variables.elapsed_time.active                 = True  
    segment.assigned_control_variables.elapsed_time.initial_guess          = True 
    segment.assigned_control_variables.elapsed_time.initial_guess_values   = [[10.]]  
    segment.assigned_control_variables.acceleration.active                 = True              
    segment.assigned_control_variables.acceleration.initial_guess          = True
    segment.assigned_control_variables.acceleration.initial_guess_values   = [[-1]] 
    segment.assigned_control_variables.body_angle.active                   = True   
    segment.assigned_control_variables.body_angle.initial_guess            = True 
    segment.assigned_control_variables.body_angle.initial_guess_values     = [[0*Units.degrees]]     

    mission.append_segment(segment)   
    
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Cruise Segment 3 : Constant Pitch Rate Constant Altltude
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Pitch_Rate_Constant_Altitude(base_segment)
    segment.tag = "cruise_3" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                                = 11. * Units.km    
    segment.pitch_rate                                              = 0.0001  * Units['rad/s/s']
    segment.pitch_final                                             = 4.  * Units.degrees 
    segment.distance                                                = 500 * Units.km   
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                 = True  
    segment.flight_dynamics.force_z                                 = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active              = True           
    segment.assigned_control_variables.throttle.assigned_propulsors = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.throttle.initial_guess       = True 
    segment.assigned_control_variables.throttle.initial_guess_values= [[0.9]] 
    segment.assigned_control_variables.velocity.active              = True           
    segment.assigned_control_variables.velocity.initial_guess       = True     
    segment.assigned_control_variables.velocity.initial_guess_values= [[ 200]] 
        
    mission.append_segment(segment)   
    
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Descent Segment 1: Constant Speed Constant Angle 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.base ) 
    segment.altitude_start                                           = 11. * Units.km    
    segment.air_speed                                                = 150 * Units.m / Units.s 
    segment.altitude_end                                             = 5  * Units.km  
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Curved Cruise Segment : Constant Radius Constant Speed Constant Altltude
    # ------------------------------------------------------------------------------------------------------------------------------------  
    segment     = Segments.Cruise.Curved_Constant_Radius_Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "curved_cruise" 
    segment.analyses.extend( analyses.base )   
    segment.altitude                                                  = 5  * Units.km  
    segment.air_speed                                                 = 240 * Units['mph']
    segment.turn_radius                                               = 1000 * Units.mile  
    segment.start_true_course                                         = 10 * Units.degrees 
    segment.turn_angle                                                = -1.0 * Units.degrees # + indicated right hand turn, negative indicates left-hand turn
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                                   = True    
    segment.flight_dynamics.force_z                                   = True    
                
    # define flight controls              
    segment.assigned_control_variables.throttle.active                = True           
    segment.assigned_control_variables.throttle.assigned_propulsors   = [['starboard_propulsor','port_propulsor']]   
    segment.assigned_control_variables.body_angle.active              = True   
    
    mission.append_segment(segment)
    

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Descent Segment 2: Constant CAS Constant Angle 
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Descent.Constant_CAS_Constant_Rate(base_segment)
    segment.tag = "descent_2" 
    segment.analyses.extend( analyses.base ) 
    segment.altitude_end                                             = 2500. * Units.feet
    segment.descent_rate                                             = 2.  * Units.m / Units.s
    segment.calibrated_air_speed                                     = 100 * Units.m / Units.s
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
     
    mission.append_segment(segment) 

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #  Single Point Segment 1: constant Speed, constant altitude
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Single_Point.Set_Speed_Set_Altitude(base_segment)
    segment.tag = "single_point_1" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                                 =  2500. * Units.feet
    segment.air_speed                                                =  200. * Units['m/s']  
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
     
    mission.append_segment(segment)    
    
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #  Single Point Segment 1: constant Speed, constant altitude
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Single_Point.Set_Speed_Set_Throttle(base_segment)
    segment.tag = "single_point_2" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                                 =  2500. * Units.feet
    segment.air_speed                                                =  200. * Units['m/s']   
    segment.throttle                                                 =  0.5 
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls   
    segment.assigned_control_variables.acceleration.active           = True             
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)    
    
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Loiter Segment: Constant Dynamic Pressure Constant Altitude Loiter
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Dynamic_Pressure_Constant_Altitude_Loiter(base_segment)
    segment.tag = "cruise_4" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                                 = 2500  * Units.feet
    segment.dynamic_pressure                                         = 12000 * Units.pascals  
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)  
    
        
    
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Loiter Segment: Constant Dynamic Pressure Constant Altitude Loiter
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Cruise.Constant_Dynamic_Pressure_Constant_Altitude_Loiter(base_segment)
    segment.tag = "cruise_5" 
    segment.analyses.extend(analyses.base) 
    segment.altitude                                                 = 2500  * Units.feet
    segment.dynamic_pressure                                         = 12000 * Units.pascals  
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)   
    

    
    # ------------------------------------------------------------------
    #   Loiter Segment: constant mach, constant time
    # ------------------------------------------------------------------
    
    segment = Segments.Cruise.Constant_Mach_Constant_Altitude_Loiter(base_segment)
    segment.tag = "cruise_6"

    segment.analyses.extend(analyses.cruise) 
    segment.mach_number                                              = 0.5
    segment.time                                                     = 30.0 * Units.minutes
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)    
    

    
    # ------------------------------------------------------------------
    #   Loiter Segment: constant mach, constant time
    # ------------------------------------------------------------------
    
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude_Loiter(base_segment)
    segment.tag = "cruise_7"

    segment.analyses.extend(analyses.cruise) 
    segment.airpseed                                                 = 168.6785
    segment.time                                                     = 30.0 * Units.minutes
               
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)    
    
        
    # ------------------------------------------------------------------
    #   Cruise Segment: constant speed, constant altitude
    # ------------------------------------------------------------------
    
    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "cruise_8"
    
    segment.analyses.extend( analyses.cruise )
    
    segment.mach_number                                              = 0.5
    segment.distance                                                 = 140.0 * Units.nautical_mile    
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)
       
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Descent Segment: Constant EAS Constant Rate
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    segment = Segments.Descent.Constant_EAS_Constant_Rate(base_segment)
    segment.tag = "descent_3" 
    segment.analyses.extend( analyses.base ) 
    segment.altitude_start                                           = 2500  * Units.feet
    segment.altitude_end                                             = 0  * Units.feet 
    segment.descent_rate                                             = 3.  * Units.m / Units.s
    segment.equivalent_air_speed                                     = 100 * Units.m / Units.s 
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)   

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Landing Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "landing"

    segment.analyses.extend( analyses.landing )
    segment.velocity_start                                                = 150 * Units.knots
    segment.velocity_end                                                  = 100 * Units.knots 
    segment.friction_coefficient                                          = 0.4
    segment.altitude                                                      = 0.0 
    segment.assigned_control_variables.elapsed_time.active                = True   
    segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[30.]]  
    mission.append_segment(segment)      


    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Non Converged Segment : Constant Throttle Constant Altltude
    # ------------------------------------------------------------------------------------------------------------------------------------  
    segment = Segments.Cruise.Constant_Throttle_Constant_Altitude(base_segment)
    segment.tag = "cruise_non_converged" 
    segment.analyses.extend(analyses.base)    
    segment.air_speed_end                                            = 150 * Units.knots
    segment.throttle                                                 = 0
    segment.distance                                                 = 10 * Units.km 
    segment.state.numerics.number_of_control_points                  = 2
    segment.state.numerics.max_evaluations                           = 10  
                
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.elapsed_time.active           = True   
    segment.assigned_control_variables.acceleration.active           = True          
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)
    
    
    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Mission definition complete    
    # ------------------------------------------------------------------------------------------------------------------------------------  
    
    return mission

def plot_results(results): 

    plot_propulsor_throttles(results)
    
    return

def missions_setup(mission): 
 
    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions  


if __name__ == '__main__': 
    main()    
