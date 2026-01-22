import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Units, Data
from RCAIDE.Framework.Analyses.Process import Process   
from RCAIDE.Library.Methods.Propulsors.Turbofan_Propulsor   import design_turbofan

# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------   

def setup():
    
    # ------------------------------------------------------------------
    #   Analysis Procedure
    # ------------------------------------------------------------------ 
    
    # size the base config
    procedure = Process()
    procedure.update_aircraft = update_aircraft
    
    # find the weights
    procedure.weights = weight 
    
    # performance studies
    procedure.missions                   = Process()
    procedure.missions.design_mission    = design_mission

    # post process the results
    procedure.post_process = post_process
        
    return procedure

# ----------------------------------------------------------------------        
#   Target Range Function
# ----------------------------------------------------------------------    

def find_target_range(nexus,mission):
    
    segments = mission.segments
    climb_1  = segments['climb_1']
    climb_2  = segments['climb_2']
    climb_3  = segments['climb_3']
  
    descent_1 = segments['descent_1']
    descent_2 = segments['descent_2']
    descent_3 = segments['descent_3']

    x_climb_1   = climb_1.altitude_end/np.tan(np.arcsin(climb_1.climb_rate/climb_1.air_speed))
    x_climb_2   = (climb_2.altitude_end-climb_1.altitude_end)/np.tan(np.arcsin(climb_2.climb_rate/climb_2.air_speed))
    x_climb_3   = (climb_3.altitude_end-climb_2.altitude_end)/np.tan(np.arcsin(climb_3.climb_rate/climb_3.air_speed)) 
    x_descent_1 = (climb_3.altitude_end-descent_1.altitude_end)/np.tan(np.arcsin(descent_1.descent_rate/descent_1.air_speed))
    x_descent_2 = (descent_1.altitude_end-descent_2.altitude_end)/np.tan(np.arcsin(descent_2.descent_rate/descent_2.air_speed))
    x_descent_3 = (descent_2.altitude_end-descent_3.altitude_end)/np.tan(np.arcsin(descent_3.descent_rate/descent_3.air_speed))
    
    cruise_range = mission.design_range-(x_climb_1+x_climb_2+x_climb_3+x_descent_1+x_descent_2+x_descent_3)
  
    segments['cruise'].distance = cruise_range
    
    return nexus

# ----------------------------------------------------------------------        
#   Design Mission
# ----------------------------------------------------------------------    
def design_mission(nexus):
    
    mission = nexus.missions.base
    mission.design_range = 1500.*Units.nautical_miles
    find_target_range(nexus,mission)
    results = nexus.results
    results.base = mission.evaluate()
    
    return nexus

# ----------------------------------------------------------------------        
#   Sizing
# ----------------------------------------------------------------------    
def update_aircraft(nexus):
    configs = nexus.vehicle_configurations
    base    = configs.base
    
    # find conditions
    air_speed   = nexus.missions.base.segments['cruise'].air_speed 
    altitude    = nexus.missions.base.segments['climb_3'].altitude_end
    atmosphere  = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976() 
    freestream  = atmosphere.compute_values(altitude)
    freestream0 = atmosphere.compute_values(6000.*Units.ft)  #cabin altitude
    
    diff_pressure         = np.max(freestream0.pressure-freestream.pressure,0)
    fuselage              = base.fuselages['tube_fuselage']
    fuselage.differential_pressure = diff_pressure 
    
    # now size engine
    mach_number        = air_speed/freestream.speed_of_sound 
    
    for config in configs:
        config.wings.horizontal_stabilizer.areas.reference = (26.0/92.0)*config.wings.main_wing.areas.reference
            
        for wing in config.wings:
            wing = RCAIDE.Library.Methods.Geometry.Planform.wing_planform(wing)
            wing.areas.exposed  = 0.8 * wing.areas.wetted
            wing.areas.affected = 0.6 * wing.areas.reference
            
        # redesign turbofan 
        for network in  config.networks: 
            for propulsor in  network.propulsors: 
                propulsor.design_mach_number   = mach_number      
                design_turbofan(propulsor) 

    return nexus

# ----------------------------------------------------------------------        
#   Weights
# ----------------------------------------------------------------------    

def weight(nexus):
    
    vehicle = nexus.vehicle_configurations.base

    weight_analysis                               = RCAIDE.Framework.Analyses.Weights.Weights_Transport()
    weight_analysis.vehicle                       = vehicle 
    weight                                        = weight_analysis.evaluate()
    
    return nexus
      

# ----------------------------------------------------------------------
#   Post Process Results to give back to the optimizer
# ----------------------------------------------------------------------   

def post_process(nexus):
    
    # Unpack data
    vehicle                           = nexus.vehicle_configurations.base
    results                           = nexus.results
    summary                           = nexus.summary
    nexus.total_number_of_iterations +=1
    
    #throttle in design mission
    max_throttle = 0 
    for i in range(len(results.base.segments)):              
        for network in results.base.segments[i].analyses.energy.vehicle.networks: 
            for j ,  propulsor in enumerate(network.propulsors):
                max_segment_throttle = np.max(results.base.segments[i].conditions.energy[propulsor.tag].throttle[:,0])
                if max_segment_throttle > max_throttle:
                    max_throttle = max_segment_throttle
                 
    summary.max_throttle = max_throttle
    
    # Fuel margin and base fuel calculations
    design_landing_weight    = results.base.segments[-1].conditions.weights.total_mass[-1]
    design_takeoff_weight    = vehicle.mass_properties.takeoff
    zero_fuel_weight         = vehicle.mass_properties.weight_breakdown.zero_fuel_weight
    
    summary.max_zero_fuel_margin  = (design_landing_weight - zero_fuel_weight)/zero_fuel_weight
    summary.base_mission_fuelburn = design_takeoff_weight - results.base.segments['descent_3'].conditions.weights.total_mass[-1]
     
    return nexus    
