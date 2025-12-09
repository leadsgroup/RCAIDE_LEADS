''' 
# active_transition_test.py
# 
# Created: Aug 25, A. Molloy

'''
#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units, Data     
from RCAIDE.Library.Plots  import *       
  
# python imports     
import numpy as np  
import sys
import matplotlib.pyplot as plt    
import os

# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Tiltrotor_EVTOL         import vehicle_setup as  vehicle_setup 
from Tiltrotor_EVTOL         import configs_setup as  configs_setup 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main(): 
    # make true only when resizing aircraft. should be left false for regression
    update_regression_values = False
         
    TW_vehicle  = vehicle_setup(redesign_rotors=update_regression_values)  
        
    # Set up configs
    configs  = configs_setup(TW_vehicle)

    # vehicle analyses
    analyses = analyses_setup(configs)

    # mission analyses
    mission  = mission_setup(analyses)
    missions = missions_setup(mission) 
     
    results = missions.base_mission.evaluate()  
    
    # Extract sample values from computation    
    transition_throttle            = results.segments.departure_transition_3.conditions.energy.propulsors['prop_rotor_propulsor_1'].throttle[1][0]
    
    #print values for resetting regression
    show_vals = True
    if show_vals:
        data = [ transition_throttle ]
        for val in data:
            print(val)
    
    # Truth values 
    transition_throttle_truth              = 0.5150143546115612
    
    # Store errors 
    error = Data() 
    error.transition_throttle             = np.max(np.abs( transition_throttle_truth - transition_throttle )/ transition_throttle_truth )
    
    print('Errors:')
    print(error)
     
    for k,v in list(error.items()):
        assert(np.abs(v)<1e-1)   # lower tolerance due to lose bounds on prop-rotor blade design 
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
    analyses.vehicle = vehicle

    # ------------------------------------------------------------------
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.update_center_of_gravity     = True 
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Weights
    weights         = RCAIDE.Framework.Analyses.Weights.Electric()
    weights.aircraft_type = "VTOL"
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics         = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.maximum_lift_coefficient =  1.5 
    analyses.append(aerodynamics)
     
    # ------------------------------------------------------------------
    #  Stability Analysis
    stability         = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method()  
    analyses.append(stability)    

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


def mission_setup(analyses): 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment() 
    base_segment.state.numerics.solver.type = 'optimize' 
    

    beta_cruise = analyses.low_speed_transition.vehicle.networks.electric.propulsors.prop_rotor_propulsor_1.rotor.cruise.design_blade_pitch_command
    
    # ------------------------------------------------------------------
    #  Second Transition Segment
    # ------------------------------------------------------------------ 
    segment                           = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                       = "departure_transition_3"  
    segment.analyses.extend(analyses.high_speed_transition)   
    segment.air_speed_start           = 90.  * Units['mph']  
    segment.air_speed_end             = 91.  * Units['mph']  
    segment.acceleration              = 9.81/5 
    segment.true_course               = 90 * Units.degree
    segment.altitude                  = 1000 * Units.ft
    

    segment.state.numerics.solver.step_size                 = 1E-2 
    segment.state.numerics.solver.tolerance_solution        = 1E-6 
    segment.state.numerics.solver.objective                 = None
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active                                = True           
    segment.assigned_control_variables.throttle.assigned_propulsors                   = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                          'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    
    segment.assigned_control_variables.thrust_vector_angle.active                     = True        
    segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        =  [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                        'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]   
    
    segment.assigned_control_variables.blade_pitch_command.active                     = True        
    segment.assigned_control_variables.blade_pitch_command.assigned_rotors            =  [['prop_rotor_1','prop_rotor_2','prop_rotor_3',
                                                                                        'prop_rotor_4','prop_rotor_5','prop_rotor_6']]   
    segment.assigned_control_variables.blade_pitch_command.bounds                     = [[0,beta_cruise]] 
     
    mission.append_segment(segment)
    
    
    
    # ------------------------------------------------------------------
    #  Second Transition Segment
    # ------------------------------------------------------------------ 
    segment                           = Segments.Cruise.Constant_Acceleration_Constant_Altitude(base_segment)
    segment.tag                       = "departure_transition_3"  
    segment.analyses.extend(analyses.high_speed_transition)    
    segment.acceleration              = 9.81/5 
    segment.true_course               = 90 * Units.degree
    segment.altitude                  = 1000 * Units.ft
    segment.air_speed_start           = 91.  * Units['mph']   
    segment.air_speed_end             = 92.  * Units['mph']
    

    segment.state.numerics.solver.step_size                 = 1E-2 
    segment.state.numerics.solver.tolerance_solution        = 1E-6 
    segment.state.numerics.solver.objective                 = None
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                       = True  
    segment.flight_dynamics.force_z                       = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active                                = True           
    segment.assigned_control_variables.throttle.assigned_propulsors                   = [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                          'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]  
    
    segment.assigned_control_variables.thrust_vector_angle.active                     = True        
    segment.assigned_control_variables.thrust_vector_angle.assigned_propulsors        =  [['prop_rotor_propulsor_1','prop_rotor_propulsor_2','prop_rotor_propulsor_3',
                                                                                        'prop_rotor_propulsor_4','prop_rotor_propulsor_5','prop_rotor_propulsor_6']]   
    
    segment.assigned_control_variables.blade_pitch_command.active                     = True        
    segment.assigned_control_variables.blade_pitch_command.assigned_rotors            =  [['prop_rotor_1','prop_rotor_2','prop_rotor_3',
                                                                                        'prop_rotor_4','prop_rotor_5','prop_rotor_6']]   
    segment.assigned_control_variables.blade_pitch_command.bounds                     = [[0,beta_cruise]] 
     
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
