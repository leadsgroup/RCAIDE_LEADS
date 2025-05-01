# RESEARCH/Aircraft/Boeing_787.py
# 
# 
# Created:  May 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import sys, os
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Performance.compute_payload_range_diagram     import compute_payload_range_diagram

sys.path.append(os.path.abspath(os.path.join(os.path.join(sys.path[0]), "../../Vehicles")))
import Boeing_787 as Boeing_787


def main():
    payload_range_results = payload_range_test()     

    # Reference (trusted) values
    truth_values = {
        "range": np.array([       0.        , 10552907.42343903, 17765081.20957468,18536439.46836733]),
        "payload": np.array([44000.        , 44000.        , 11076.67030479,     0.        ]),
        "oew_plus_payload": np.array([159530.32969521, 159530.32969521, 126607.        , 115530.32969521]),
        "fuel": np.array([     0.        ,  68399.67030479, 101323.        , 101323.        ]),
        "takeoff_weight": np.array([     0.        , 227930.        , 227930.        , 216853.32969521]),
        "fuel_reserve_percentage": 0.05,
    }
    # Tolerance checks
    for key in truth_values:
        error = np.max(np.abs(np.atleast_1d(payload_range_results[key]) - np.atleast_1d(truth_values[key]))/np.atleast_1d(truth_values[key]))
        assert error < 1e-4, f"{key} error too large: {error}"
    
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
    mission.tag = 'simplified_payload_range_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.solver.type = 'root_finder' 

    #------------------------------------------------------------------
    #   First Climb Segment: Constant Speed Constant Rate  
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "Inital_Climb" 
    segment.analyses.extend( analyses.cutback )  
    segment.altitude_start                                           = 0 * Units['ft']
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
    segment.tag = "Climb_to_Cruise_4" 
    segment.analyses.extend( analyses.cruise ) 
    segment.altitude_end                                             = 40000   * Units['ft']
    segment.air_speed                                                = 480 * Units['knots']
    segment.climb_rate                                               = 280   * Units['fpm']  
              
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
    segment.altitude                                                 = 40000 * Units['ft']  
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
    segment.altitude_end                                             = 0 * Units.ft
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

    return mission


if __name__ == '__main__': 
    main()    