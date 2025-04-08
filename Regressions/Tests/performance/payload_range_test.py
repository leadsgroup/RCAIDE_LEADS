# Regression/scripts/Tests/performance_payload_range.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Core import Units , Container
from RCAIDE.Library.Methods.Performance.compute_payload_range_diagram        import compute_payload_range_diagram

# python imports     
import numpy as np  
import sys
import matplotlib.pyplot as plt  
import os
# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Embraer_190    import vehicle_setup as E190_vehicle_setup 
from NASA_X57       import vehicle_setup as X57_vehicle_setup      

# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------  
def main(): 
    fuel_aircraft_payload_range()
    electric_aircraft_payload_range() 
    return
    
def fuel_aircraft_payload_range():
    
    # vehicle data
    vehicle             = E190_vehicle_setup()

    # take out control surfaces to make regression run faster
    for wing in vehicle.wings:
        wing.control_surfaces  = Container() 
  
    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = fuel_aircraft_analyses_setup(configs)

    # mission analyses 
    mission = fuel_aircraft_mission_setup(analyses)
    
    # create mission instances (for multiple types of missions)
    missions = missions_setup(mission)  
        
    # run payload range analysis 
    payload_range_results =  compute_payload_range_diagram(mission = missions.base_mission)
                                
    fuel_r                 = payload_range_results.range[-1]  
    fuel_r_true            = 7441130.907817602
    print('Fuel Range: ' + str(fuel_r))
    fuel_error =  abs(fuel_r - fuel_r_true) /fuel_r_true
    assert(abs(fuel_error)<1e-6)
    
    return  

def electric_aircraft_payload_range(): 

    rotor_type  =  ['Blade_Element_Momentum_Theory_Helmholtz_Wake', 'Actuator_Disk_Theory']  
    electric_r_truth = [37039.99999999999, 37039.99999999999]
    for i in range(len(rotor_type)):
        
        # vehicle data
        vehicle             = X57_vehicle_setup(rotor_type[i]) 
    
        # Set up vehicle configs
        configs  = configs_setup(vehicle)
    
        # create analyses
        analyses = electric_aircraft_analyses_setup(configs)
    
        # mission analyses 
        mission = electric_aircraft_mission_setup(analyses)
        
        # create mission instances (for multiple types of missions)
        missions = missions_setup(mission)   
    
        payload_range_results =  compute_payload_range_diagram(mission = missions.base_mission) 
        electric_r         =  payload_range_results.range[-1]
        print('Electric Range: ' + str(electric_r ))
        electric_error =  abs(electric_r - electric_r_truth[i]) /electric_r_truth[i]
        assert(abs(electric_error)<1e-6)        

    return  

# ---------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
 
    
    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base'  
    configs.append(base_config)
 
    return configs
  
def fuel_aircraft_analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = fuel_aircraft_base_analysis(config)
        analyses[tag] = analysis

    return analyses

def electric_aircraft_analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = electric_aircraft_base_analysis(config)
        analyses[tag] = analysis

    return analyses


def fuel_aircraft_base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    
    # ------------------------------------------------------------------
    #  Weights 
    weights         = RCAIDE.Framework.Analyses.Weights.Conventional()
    weights.aircraft_type =  "Transport"
    weights.vehicle = vehicle 
    weights.settings.update_mass_properties         = False
    weights.settings.update_center_of_gravity       = False
    weights.settings.update_moment_of_inertia       = False   
    weights.print_weight_analysis_report  = False    
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis 
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle  = vehicle
    aerodynamics.settings.number_of_spanwise_vortices   = 5
    aerodynamics.settings.number_of_chordwise_vortices  = 2       
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



def electric_aircraft_base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    
    # ------------------------------------------------------------------
    #  Weights 
    weights = RCAIDE.Framework.Analyses.Weights.Electric()
    weights.aircraft_type   = 'General_Aviation'
    weights.print_weight_analysis_report  = False  
    weights.vehicle = vehicle
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis 
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.vehicle  = vehicle
    aerodynamics.settings.number_of_spanwise_vortices   = 5
    aerodynamics.settings.number_of_chordwise_vortices  = 2     
    aerodynamics.training.Mach                          = np.array([0.1  ,0.3,  0.5,  0.65 , 0.95])  
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

# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
 
def electric_aircraft_mission_setup(analyses): 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points  = 3

    # ------------------------------------------------------------------    
    #   Cruise Segment 
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.base ) 
    segment.initial_battery_state_of_charge              = 1.0 
    segment.altitude                                     = 15000   * Units.feet 
    segment.air_speed                                    = 130 * Units.kts
    segment.distance                                     = 20.   * Units.nautical_mile
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]   
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment) 
 

    return mission


def fuel_aircraft_mission_setup(analyses): 
    
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'mission'
  
    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points  = 3   

    # ------------------------------------------------------------------    
    #   Cruise Segment 
    # ------------------------------------------------------------------    

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.base )  
    segment.altitude  =  35000 *  Units.ft
    segment.air_speed =  450 * Units['knots']
    segment.distance  =  2700 * Units.nmi
    
    # define flight dynamics to model 
    segment.flight_dynamics.force_x                      = True  
    segment.flight_dynamics.force_z                      = True     
    
    # define flight controls 
    segment.assigned_control_variables.throttle.active               = True           
    segment.assigned_control_variables.throttle.assigned_propulsors  = [['starboard_propulsor','port_propulsor']]   
    segment.assigned_control_variables.body_angle.active             = True                
    
    mission.append_segment(segment) 
 

    return mission

def missions_setup(mission): 
 
    missions     = RCAIDE.Framework.Mission.Missions() 
    mission.tag  = 'base_mission'
    missions.append(mission)
 
    return missions   


if __name__ == '__main__': 
    main()    
    plt.show() 