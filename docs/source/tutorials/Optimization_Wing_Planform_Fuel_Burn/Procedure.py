# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports 
import RCAIDE
from RCAIDE.Framework.Core import Units      
from RCAIDE.Framework.Analyses.Process import Process 
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan   import design_turbofan  

# python imports 
import numpy as np    

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
    
    # performance studies
    procedure.missions                   = Process()
    procedure.missions.design_mission    = design_mission

    # post process the results
    procedure.post_process = post_process
        
    return procedure 
 

# ----------------------------------------------------------------------        
#   Update Aircraft Properties 
# ----------------------------------------------------------------------    
def update_aircraft(nexus): 
    configs = nexus.vehicle_configurations 
    
    for config in configs:
        # the wing span (remember in RCAIDE, wing segments are scaled based on span and root chord)
        config.wings.main_wing.spans.projected = np.sqrt(config.wings.main_wing.aspect_ratio * config.wings.main_wing.areas.reference)  
        
        # resize engine 
        for network in  config.networks: 
            for propulsor in  network.propulsors:
            
                # redesign engine design mach number since altitude changes
                air_speed   = nexus.missions.base_mission.segments['cruise'].air_speed 
                altitude    = nexus.missions.base_mission.segments['cruise'].altitude 
                atmosphere  = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976() 
                freestream  = atmosphere.compute_values(altitude) 
                mach_number = air_speed/freestream.speed_of_sound[0][0]
                
                # update variable of aircraft 
                propulsor.design_mach_number   = mach_number
                
                # update properties 
                design_turbofan(propulsor) 

    return nexus 


# ----------------------------------------------------------------------        
#   Design Mission
# ----------------------------------------------------------------------    
def design_mission(nexus):

    # run mission 
    mission = nexus.missions.base_mission
    mission.design_range  =  1500 *  Units.nmi
    
    # store results  
    nexus.results.base_mission = mission.evaluate()
    
    return nexus


# ----------------------------------------------------------------------
#   Post Process Results to give back to the optimizer
# ----------------------------------------------------------------------   

def post_process(nexus):
    
    # Unpack data
    results                           = nexus.results
    summary                           = nexus.summary
    nexus.total_number_of_iterations +=1
    
    # throttle in design mission
    max_throttle = 0 
    for i in range(len(results.base_mission.segments)):              
        for network in results.base_mission.segments[i].analyses.vehicle.networks: 
            for j ,  propulsor in enumerate(network.propulsors):
                max_throttle = np.maximum(max_throttle, np.max(results.base_mission.segments[i].conditions.energy.propulsors[propulsor.tag].throttle[:,0]))  
    
    # Vehicle
    vehicle                  = results.base_mission.segments['takeoff'].analyses.vehicle
    
    # Fuel margin and base fuel calculations 
    design_takeoff_weight    = vehicle.mass_properties.takeoff
    max_fuel                 = vehicle.mass_properties.max_fuel
    fuel_burnt               = design_takeoff_weight - results.base_mission.segments['landing'].conditions.weights.vehicle.mass[-1,0]
    
    # store variables for optimizer 
    summary.fuel_margin           = (max_fuel - fuel_burnt)/max_fuel
    summary.fuel_burn             = fuel_burnt
    summary.range_residual        = results.base_mission.design_range -  results.base_mission.segments['landing'].conditions.frames.inertial.aircraft_range[-1,0]
    summary.max_throttle          = max_throttle
    
    return nexus    
