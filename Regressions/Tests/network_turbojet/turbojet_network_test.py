# Regression/scripts/Tests/turbofan_network_test.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Core                          import Units , Data 
from RCAIDE.Library.Plots                           import *   
from RCAIDE.load    import load 
from RCAIDE.save    import save     

# python imports     
import numpy as np  
import sys
import matplotlib.pyplot as plt 
import os

# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Concorde    import vehicle_setup as vehicle_setup
from Concorde    import configs_setup as configs_setup 

# ----------------------------------------------------------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------------------------------------------------------

def main():
    

    # vehicle data
    vehicle  = vehicle_setup()

    # plot vehicle 
    plot_3d_vehicle(vehicle, 
                    min_x_axis_limit            = 0,
                    max_x_axis_limit            = 60,
                    min_y_axis_limit            = -30,
                    max_y_axis_limit            = 30,
                    min_z_axis_limit            = -30,
                    max_z_axis_limit            = 30,
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
    plot_mission(results)    

    # Extract sample values from computation  
    thrust     = results.segments.climb_1.conditions.energy['inner_right_turbojet'].thrust[3][0]
    throttle   = results.segments.level_cruise.conditions.energy['inner_right_turbojet'].throttle[3][0] 
    CL         = results.segments.descent_1.conditions.aerodynamics.coefficients.lift.total[2][0] 
    
    #print values for resetting regression
    show_vals = True
    if show_vals:
        data = [thrust, throttle, CL]
        for val in data:
            print(val)
    
    # Truth values
    thrust_truth     = 182249.8877230102
    throttle_truth   = 0.8356192341618192
    CL_truth         = 0.20942980536787167
    
    # Store errors 
    error = Data()
    error.thrust    = np.max(np.abs(thrust - thrust_truth )/thrust_truth) 
    error.throttle  = np.max(np.abs(throttle  - throttle_truth  )/throttle_truth) 
    error.CL        = np.max(np.abs(CL - CL_truth   )/CL_truth)
    
    # Save and Load Test 
    save(error, 'turbojet_network_errors.res')
    old_errors = load('turbojet_network_errors.res')  
     
    print('Errors:')
    print(error)
     
    for k,v in list(error.items()): 
        assert(np.abs(v)<1e-6)
        
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
    weights         = RCAIDE.Framework.Analyses.Weights.Conventional()
    weights.vehicle = vehicle
    analyses.append(weights)
    
    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics                                       = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.vehicle                              = vehicle
    aerodynamics.settings.number_of_spanwise_vortices  = 5
    aerodynamics.settings.number_of_chordwise_vortices = 2       
    aerodynamics.settings.model_fuselage               = True
    aerodynamics.settings.drag_coefficient_increment   = 0.0000
    analyses.append(aerodynamics)


    # ------------------------------------------------------------------
    #  Emissions
    emissions = RCAIDE.Framework.Analyses.Emissions.Emission_Index_Correlation_Method()
    emissions.vehicle = vehicle          
    analyses.append(emissions)
  
    # ------------------------------------------------------------------
    #  Energy
    energy= RCAIDE.Framework.Analyses.Energy.Energy()
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

# ----------------------------------------------------------------------
#   Plot Mission
# ----------------------------------------------------------------------

def plot_mission(results):
    
    plot_altitude_sfc_weight(results) 
    
    plot_flight_conditions(results) 
    
    plot_aerodynamic_coefficients(results)  
    
    plot_aircraft_velocities(results)
    
    plot_drag_components(results) 
 
    plot_CO2e_emissions(results) 
  
    plot_aerodynamic_forces(results)
    
    plot_fuel_consumption(results)
     
        
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
    
    # ------------------------------------------------------------------
    #   First Climb Segment: constant Mach, constant segment angle 
    # ------------------------------------------------------------------
    
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1" 
    segment.analyses.extend( analyses.climb ) 
    segment.altitude_start = 0.0   * Units.km
    segment.altitude_end   = 4000. * Units.ft
    segment.airpseed       = 250.  * Units.kts
    segment.climb_rate     = 4000. * Units['ft/min']
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                   
      
    mission.append_segment(segment)
    
    
    # ------------------------------------------------------------------
    #   Second Climb Segment: constant Speed, constant segment angle 
    # ------------------------------------------------------------------    
    
    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.analyses.aerodynamics.settings.supersonic.wave_drag_type == 'Sears-Haack'    
    segment.altitude_end = 8000. * Units.ft
    segment.airpseed     = 250.  * Units.kts
    segment.climb_rate   = 2000. * Units['ft/min']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Second Climb Segment: constant Speed, constant segment angle 
    # ------------------------------------------------------------------    
    
    segment = Segments.Climb.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "climb_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end        = 33000. * Units.ft
    segment.mach_number_start   = .45
    segment.mach_number_end     = 0.95
    segment.climb_rate          = 3000. * Units['ft/min']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                 
    
    mission.append_segment(segment)    

    # ------------------------------------------------------------------
    #   Third Climb Segment: linear Mach, constant segment angle 
    # ------------------------------------------------------------------    
      
    segment = Segments.Climb.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "climb_3" 
    segment.analyses.extend( analyses.climb ) 
    segment.altitude_end        = 34000. * Units.ft
    segment.mach_number_start   = 0.95
    segment.mach_number_end     = 1.1
    segment.climb_rate          = 2000.  * Units['ft/min']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment) 

    # ------------------------------------------------------------------
    #   Third Climb Segment: linear Mach, constant segment angle 
    # ------------------------------------------------------------------   
    segment     = Segments.Climb.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "climb_4" 
    segment.analyses.extend( analyses.climb ) 
    segment.altitude_end        = 40000. * Units.ft
    segment.mach_number_start   = 1.1
    segment.mach_number_end     = 1.7
    segment.climb_rate          = 1750.  * Units['ft/min']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                 
    
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   Fourth Climb Segment: linear Mach, constant segment angle 
    # ------------------------------------------------------------------  
    segment = Segments.Climb.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "climb_5" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end        = 50000. * Units.ft
    segment.mach_number_start   = 1.7
    segment.mach_number_end     = 2.02
    segment.climb_rate          = 750.  * Units['ft/min']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                  
    
    mission.append_segment(segment)     
    

    # ------------------------------------------------------------------
    #   Fourth Climb Segment: linear Mach, constant segment angle 
    # ------------------------------------------------------------------  
    segment = Segments.Climb.Constant_Mach_Constant_Rate(base_segment)
    segment.tag = "climbing_cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                  = 56500. * Units.ft
    segment.mach_number                                   = 2.02
    segment.climb_rate                                    = 50.  * Units['ft/min']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------    
    #   Cruise Segment: constant speed 
    # ------------------------------------------------------------------    
    segment     = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "level_cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.mach_number                                   = 2.02
    segment.distance                                      = 10. * Units.nmi
    segment.state.numerics.number_of_control_points          = 4  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)    
    
    # ------------------------------------------------------------------
    #   First Descent Segment: decceleration
    # ------------------------------------------------------------------    
    segment     = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise )
    segment.acceleration                                  = -.5  * Units['m/s/s']
    segment.air_speed_end                                 = 1.5*573.  * Units.kts 
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                 
     
    mission.append_segment(segment)   
    
    # ------------------------------------------------------------------
    #   First Descent Segment
    # ------------------------------------------------------------------  
    segment     = Segments.Descent.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.cruise )
    segment.altitude_end      = 50000. * Units.ft
    segment.mach_number_end   = 1.2
    segment.descent_rate      = 2000. * Units['ft/min']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)     
    

    # ------------------------------------------------------------------
    #   First Descent Segment
    # ------------------------------------------------------------------  
    segment     = Segments.Descent.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_2" 
    segment.analyses.extend( analyses.cruise )
    segment.altitude_end      = 41000. * Units.ft
    segment.air_speed_end     = 800 *  Units.mph
    segment.descent_rate      = 2000. * Units['ft/min']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)
    
    # ------------------------------------------------------------------
    #   First Descent Segment: decceleration
    # ------------------------------------------------------------------   
    segment     = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag = "decel_2" 
    segment.analyses.extend( analyses.cruise )
    segment.acceleration      = -.5  * Units['m/s/s']
    segment.air_speed_end     = 0.95*573.  * Units.kts  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                   
    
    mission.append_segment(segment)     
    
    # ------------------------------------------------------------------
    #   First Descent Segment
    # ------------------------------------------------------------------    
      
    segment = Segments.Descent.Linear_Mach_Constant_Rate(base_segment)
    segment.tag = "descent_3" 
    segment.analyses.extend( analyses.cruise )
    segment.altitude_end      = 10000. * Units.ft
    segment.mach_number_end   = 0.15
    segment.descent_rate      = 2000. * Units['ft/min']  
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)     
    
    # ------------------------------------------------------------------
    #   First Descent Segment
    # ------------------------------------------------------------------     
    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_4" 
    segment.analyses.extend( analyses.cruise )
    segment.altitude_end = 0. * Units.ft
    segment.descent_rate = 1000. * Units['ft/min']    
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['inner_right_turbojet','outer_right_turbojet','outer_left_turbojet','inner_left_turbojet']] 
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment)      
    
    # ------------------------------------------------------------------    
    #   Mission definition complete    
    # ------------------------------------------------------------------
    
    return mission

def missions_setup(mission):

    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)
    
    # done!
    return missions   


if __name__ == '__main__': 
    main()    
    plt.show()
        
