# Regression/scripts/network_electric/hydrogen_fuel_cell_aircraft_test.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
import RCAIDE  
from RCAIDE.Framework.Core   import Units, Data   
from RCAIDE.Library.Plots    import *   
from RCAIDE.Library.Methods.Performance.estimate_stall_speed  import estimate_stall_speed 
 
# package imports  
import numpy as np
import matplotlib.pyplot as plt 
import matplotlib.cm as cm

# local imports 
import sys 
import os
import numpy as np
import matplotlib.pyplot as plt 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Hydrogen_Fuel_Cell_Twin_Otter   import vehicle_setup , configs_setup  

# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------  

def main():  
 
    mdot_H2_true         = [0.02090216460932281,0.01991295354916295]
    fuel_cell_models     = ['PEM', 'Larminie', ]
    
    for i in range(2): 
    
        vehicle  = vehicle_setup(fuel_cell_models[i]) 
        
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
        
        # Voltage Cell Regression
        mdot_H2        = results.segments[0].conditions.energy.bus.cryogenic_tank.mass_flow_rate[0,0]
        print('Mass Flow Rate: ' + str(mdot_H2))
        mdot_H2_diff   = np.abs(mdot_H2 - mdot_H2_true[i]) 
        print(mdot_H2_diff) 
        assert np.abs((mdot_H2_diff)/mdot_H2_true[i]) < 1e-6
        
        if i == 0: 
            plot_results(results)
        
    return 
 
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
 
    # ------------------------------------------------------------------
    #  Weights
    weights          = RCAIDE.Framework.Analyses.Weights.Electric()
    weights.vehicle  = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis  
    aerodynamics                   = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle           = vehicle
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
 
    return analyses

# ----------------------------------------------------------------------
#   Define the Mission
# ---------------------------------------------------------------------- 
def mission_setup(analyses):
    

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission' 

    # unpack Segments module
    Segments = RCAIDE.Framework.Mission.Segments  
    base_segment = Segments.Segment()
    base_segment.temperature_deviation  = 2.5
    base_segment.state.numerics.number_of_control_points  = 16
    
    # VSTALL Calculation  
    vehicle        = analyses.base.aerodynamics.vehicle
    vehicle_mass   = vehicle.mass_properties.max_takeoff
    reference_area = vehicle.reference_area 
    Vstall         = estimate_stall_speed(vehicle_mass,reference_area,altitude = 0.0,maximum_lift_coefficient = 1.2)
    
    # ------------------------------------------------------------------
    #   Departure End of Runway Segment  
    # ------------------------------------------------------------------ 
    segment = Segments.Climb.Linear_Speed_Constant_Rate(base_segment) 
    segment.tag = 'climb'       
    segment.analyses.extend( analyses.base )  
    segment.altitude_start                                           = 0.0 * Units.feet
    segment.altitude_end                                             = 5 
    segment.air_speed_start                                          = Vstall *1.2  
    segment.air_speed_end                                            = Vstall *1.25
    segment.initial_battery_state_of_charge                          = 1.0
                       
    # define flight dynamics to model            
    segment.flight_dynamics.force_x                                  = True  
    segment.flight_dynamics.force_z                                  = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']] 
    segment.assigned_control_variables.body_angle.active             = True                  
       
    mission.append_segment(segment) 
    
    # ------------------------------------------------------------------
    #   Mission definition complete    
    # ------------------------------------------------------------------ 
    return mission

def missions_setup(mission): 
 
    missions         = RCAIDE.Framework.Mission.Missions()
    
    # base mission 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions  

def plot_results(results):
    # Plots fligh conditions 
    plot_flight_conditions(results)
    plot_fuel_consumption(results)
    return


 
if __name__ == '__main__':
    main()
    plt.show()