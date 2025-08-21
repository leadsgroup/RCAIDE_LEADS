# RESEARCH/Aircraft/Boeing_787.py
# 
# 
# Created:  May 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import sys, os
import numpy as np
import time

import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Performance.compute_payload_range_diagram     import compute_payload_range_diagram

sys.path.append(os.path.abspath(os.path.join(os.path.join(sys.path[0]), "../../Vehicles")))
import Boeing_787 as Boeing_787


def main():
    ti                   = time.time()
    payload_range_results = payload_range_test()     

    # Reference (trusted) values
    # DO not change these values without consulting the airport planning manual first:
    #  "Airport Planning Manual": {
    #     "range": [0, 5500, 9500, 10000]  nmi,
    #     "payload": (([44000, 44000, 9071.8474, 0]) lbs
    #     "payload + oew": (([161025, 161025, 127005.864, 117934.016]) lbs
    truth_values = {
        "range": np.array([0., 10372385.56834877, 17987562.39084088,18643098.13266074]),
        "payload": np.array([44000.        , 44000.        ,  8587.76052736,     0.        ]),
        "oew_plus_payload": np.array([162019.23947264, 162019.23947264, 126607.        , 118019.23947264]),
        "fuel": np.array([     0.        ,  65910.76052736, 101323.        , 101323.        ]),
        "takeoff_weight": np.array([     0.        , 227930.        , 227930.        , 219342.23947264]),
    }
    # Tolerance checks
    for key in truth_values:
        denom = np.atleast_1d(truth_values[key])
        numer = np.abs(np.atleast_1d(payload_range_results[key]) - denom)

        # Avoid division by zero
        with np.errstate(divide='ignore', invalid='ignore'):
            rel_error = np.where(denom != 0, numer / denom, 0.0)
            error = np.max(rel_error)

        assert error < 1e-4, f"{key} error too large: {error}"
    tf                   = time.time()
    elapsed_time         = round((tf-ti),2)
    print('Payload Range simulation Time: ' + str(elapsed_time) + ' seconds') 
            
    return

def payload_range_test():
    
    vehicle  = Boeing_787.vehicle_setup()   
    configs  = Boeing_787.configs_setup(vehicle) 
    analyses = Boeing_787.analyses_setup(configs) 
    mission  = payload_range_mission_setup(analyses)
    missions = Boeing_787.missions_setup(mission)
     
    # run payload range analysis 
    payload_range_results =  compute_payload_range_diagram(mission = missions.base_mission, fuel_reserve_percentage = 0.05)
  
    
    return  payload_range_results

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def payload_range_mission_setup(analyses):
    """This function defines the baseline mission that will be flown by the aircraft in order
    to compute performance."""
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 
    base_segment.state.numerics.solver.type = 'root_finder'
    

    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Takeoff Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Takeoff(base_segment)
    segment.tag = "Takeoff_Ground_Run" 
    segment.analyses.extend( analyses.takeoff )
    segment.velocity_start           = 30.* Units.knots
    segment.velocity_end             = 167.0 * Units['knots']
    segment.friction_coefficient     = 0.03
    segment.altitude                 = 0.0   
    segment.throttle                 = 1.0
    mission.append_segment(segment)
      
    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    
    
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Takeoff_Climb" 
    segment.analyses.extend( analyses.takeoff ) 
    segment.altitude_start                                           = 0.0 * Units['knots'] 
    segment.altitude_end                                             = 35 * Units['ft']
    segment.air_speed_end                                            = 167.0 * Units['knots']
    segment.air_speed_start                                          = 175.0 * Units['knots']
    segment.climb_rate                                               = 250 * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment) 

    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Inital_Climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.air_speed_start                                            = 0.0 * Units['knots'] 
    segment.altitude_end                                             = 1000  * Units['feet']
    segment.air_speed                                                = 200.0 * Units['knots']
    segment.climb_rate                                               = 1800   * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                 

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------    

    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_1" 
    segment.analyses.extend( analyses.cutback ) 
    segment.air_speed_start                                          = 200.0 * Units['knots'] 
    segment.altitude_end                                             = 8000   * Units['ft']
    segment.air_speed_end                                            = 300 * Units['knots']
    segment.climb_rate                                               = 1700   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_2" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 16000   * Units['ft']
    segment.air_speed                                                = 350 * Units['knots']
    segment.climb_rate                                               = 1300   * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment)


    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Climb_to_Cruise_3" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 35000   * Units['ft']
    segment.air_speed                                                = 450 * Units['knots']
    segment.climb_rate                                               = 1000   * Units['fpm']  
              
    # define flight dynamics to model               
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                  

    mission.append_segment(segment) 

    # ------------------------------------------------------------------    
    #   Cruise Segment: Constant Speed Constant Altitude
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude                                                 = 35000 * Units['ft']  
    segment.air_speed                                                = 450 * Units['knots']
    segment.distance                                                 = 7370 * Units.km   
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   First Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "descent_1" 
    segment.analyses.extend( analyses.descent ) 
    segment.altitude_end                                             = 10000   * Units.ft
    segment.air_speed                                                = 380 * Units['knots']
    segment.descent_rate                                             = 1850   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Second Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag  = "approach" 
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = 2000 * Units.ft
    segment.air_speed                                                = 225.0 * Units['knots']
    segment.descent_rate                                             = 650  * Units['fpm']  
             
    # define flight dynamics to model              
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Third Descent Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "final_approach"  
    segment.analyses.extend( analyses.landing ) 
    segment.altitude_end                                             = .0   * Units.ft
    segment.air_speed                                                = 175.0 * Units['knots']
    segment.descent_rate                                             = 600.0   * Units['fpm']  
            
    # define flight dynamics to model             
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     

    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['propulsor_1','propulsor_2']] 
    segment.assigned_control_variables.body_angle.active             = True                

    mission.append_segment(segment) 


    # ------------------------------------------------------------------------------------------------------------------------------------ 
    #   Landing Roll
    # ------------------------------------------------------------------------------------------------------------------------------------ 

    segment = Segments.Ground.Landing(base_segment)
    segment.tag = "Landing"

    segment.analyses.extend( analyses.reverse_thrust ) 
    segment.velocity_start                                                = 160.0 * Units['knots']
    segment.velocity_end                                                  = 30 * Units.knots 
    segment.friction_coefficient                                          = 0.4
    segment.altitude                                                      = 0.0   
    segment.assigned_control_variables.elapsed_time.active                = True  
    segment.assigned_control_variables.elapsed_time.initial_guess_values  = [[30.]]  
    mission.append_segment(segment)     

 
    return mission

if __name__ == '__main__': 
    main()    