# VnV/Verification/analysis_stability/trimmed_stab_deriv_flight_test.py
# 
# Created: June 2025, A. Molloy
 
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

sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
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
    mission  = sideslip_cruise_mission_setup(analyses) 

    # create mission instances (for multiple types of missions)
    missions = missions_setup(mission) 

    # mission analysis 
    results = missions.base_mission.evaluate() 

    elevator_deflection        = results.segments.cruise.conditions.control_surfaces.elevator.deflection[0,0] / Units.deg
    elevator_deflection_true   = -2.018772944377962
    elevator_deflection_diff   = np.abs(elevator_deflection - elevator_deflection_true)
    print('Error1: ',elevator_deflection_diff)
    assert np.abs(elevator_deflection_diff/elevator_deflection_true) < 5e-3

    aileron_deflection        = results.segments.cruise.conditions.control_surfaces.aileron.deflection[0,0] / Units.deg
    aileron_deflection_true   = 9.630686598302654
    aileron_deflection_diff   = np.abs(aileron_deflection - aileron_deflection_true)
    print('Error2: ',aileron_deflection_diff)
    assert np.abs(aileron_deflection_diff/aileron_deflection_true) < 5e-3

    rudder_deflection        = results.segments.cruise.conditions.control_surfaces.rudder.deflection[0,0] / Units.deg
    rudder_deflection_true   = -12.221227727988088
    rudder_deflection_diff   = np.abs(rudder_deflection - rudder_deflection_true)
    print('Error3: ',rudder_deflection_diff)
    assert np.abs(rudder_deflection_diff/rudder_deflection_true) < 5e-3    
    
    return  
# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def base_analysis(vehicle, configs):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses        = RCAIDE.Framework.Analyses.Vehicle() 

    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.vehicle                             = vehicle
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------
    weights         = RCAIDE.Framework.Analyses.Weights.Conventional()
    weights.vehicle = vehicle
    weights.method  = "FLOPS"
    weights.aircraft_type = "General_Aviation"
    weights.settings.FLOPS.fidelity   = "Simple"
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    # ------------------------------------------------------------------ 
    
    aerodynamics                                    = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle                            = vehicle          
    aerodynamics.stability_derivatives.CX_alpha     = 0.0001
    aerodynamics.stability_derivatives.CX_u         =  0.0001
    aerodynamics.stability_derivatives.CY_beta      = -0.195398
    aerodynamics.stability_derivatives.CY_r         =  0.0001
    aerodynamics.stability_derivatives.CZ_alpha     = 4.837822
    aerodynamics.stability_derivatives.CZ_u         =  0.0001
    aerodynamics.stability_derivatives.CZ_q         =  0.0001
    aerodynamics.stability_derivatives.CL_beta      = -0.12228
    aerodynamics.stability_derivatives.CL_p         = 0.0001
    aerodynamics.stability_derivatives.CL_r         = 0.0001
    aerodynamics.stability_derivatives.CM_alpha     = -1.1509 
    aerodynamics.stability_derivatives.CM_u         = 0.0001
    aerodynamics.stability_derivatives.CM_q         = 0.0001
    aerodynamics.stability_derivatives.CN_beta      = 0.074425
    aerodynamics.stability_derivatives.CN_p         = 0.0001
    aerodynamics.stability_derivatives.CN_r         = 0.0001
    aerodynamics.stability_derivatives.CY_delta_a   = 0.000623 * 180/np.pi
    aerodynamics.stability_derivatives.CL_delta_a   = 0.001974 * 180/np.pi
    aerodynamics.stability_derivatives.CLift_delta_e= 0.009275 * 180/np.pi
    aerodynamics.stability_derivatives.CN_delta_a   = -0.000021 * 180/np.pi
    aerodynamics.stability_derivatives.CM_delta_e   = -0.024291 * 180/np.pi
    aerodynamics.stability_derivatives.CY_delta_r   = -0.001918 * 180/np.pi
    aerodynamics.stability_derivatives.CL_delta_r   = -0.000195 * 180/np.pi
    aerodynamics.stability_derivatives.CN_delta_r   = 0.001046 * 180/np.pi
    aerodynamics.stability_derivatives.CM_delta_f   =  0.0001  
    analyses.append(aerodynamics) 
     
    stability                                       = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method()   
    stability.vehicle                               = vehicle
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Energy
    # ------------------------------------------------------------------
    energy     = RCAIDE.Framework.Analyses.Energy.Energy()
    energy.vehicle = vehicle  
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    # ------------------------------------------------------------------
    planet     = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    # ------------------------------------------------------------------
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)   

    # done!
    return analyses 

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config, configs)
        analyses[tag] = analysis

    return analyses
 
# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def sideslip_cruise_mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments
    
    base_segment = Segments.Segment() 
 
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
    segment.assigned_control_variables.body_angle.initial_guess_values          = [[5 * Units.degrees]]    
    segment.assigned_control_variables.elevator_deflection.active               = True    
    segment.assigned_control_variables.elevator_deflection.assigned_surfaces    = [['elevator']] 
    segment.assigned_control_variables.aileron_deflection.active                = True    
    segment.assigned_control_variables.aileron_deflection.assigned_surfaces     = [['aileron']]
    segment.assigned_control_variables.aileron_deflection.initial_guess_values  = [[-10. * Units.degrees]]
    segment.assigned_control_variables.rudder_deflection.active                 = True    
    segment.assigned_control_variables.rudder_deflection.assigned_surfaces      = [['rudder']]
    segment.assigned_control_variables.rudder_deflection.initial_guess_values   = [[14 * Units.degrees]] 
    segment.assigned_control_variables.bank_angle.active                        = True        
    segment.assigned_control_variables.bank_angle.initial_guess_values          = [[12 * Units.degrees]] 
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
    segment.assigned_control_variables.body_angle.initial_guess_values          = [[5 * Units.degrees]]    
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
    segment.analyses.stability.settings.unique_segment_surrogate                = True

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