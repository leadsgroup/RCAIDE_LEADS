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
        # update wing aspect ratio  
        config.wings.main_wing.aspect_ratio = (config.wings.main_wing.spans.projected **2)/ config.wings.main_wing.areas.reference  
        
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
    
    ##throttle in design mission
    #max_throttle = 0 
    #for i in range(len(results.base.segments)):              
        #for network in results.base.segments[i].analyses.energy.vehicle.networks: 
            #for j ,  propulsor in enumerate(network.propulsors):
                #max_segment_throttle = np.max(results.base.segments[i].conditions.energy[propulsor.tag].throttle[:,0])
                #if max_segment_throttle > max_throttle:
                    #max_throttle = max_segment_throttle
                 
    #summary.max_throttle = max_throttle
    
    # get vehicle 
    vehicle                  = results.base_mission.segments['takeoff'].analyses.vehicle
    
    # Fuel margin and base fuel calculations
    design_landing_weight    = results.base_mission.segments['landing'].conditions.weights.vehicle.mass[-1,0] 
    design_takeoff_weight    = vehicle.mass_properties.takeoff
    zero_fuel_weight         = vehicle.mass_properties.weight_breakdown.zero_fuel_weight
    
    # store variables for optimizer 
    summary.max_zero_fuel_margin  = abs(design_landing_weight - zero_fuel_weight)/zero_fuel_weight
    summary.base_mission_fuelburn = design_takeoff_weight  - results.base_mission.segments['landing'].conditions.weights.vehicle.mass[-1,0]
    summary.design_range_residual = abs(results.base_mission.design_range -  results.base_mission.segments['landing'].conditions.frames.inertial.aircraft_range[-1,0])
    
    return nexus    
