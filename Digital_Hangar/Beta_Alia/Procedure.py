# Procedure.py
# 
# Created: Dec 2021, E. Botero
# Modified: 

# ----------------------------------------------------------------------        
#   Imports
# ----------------------------------------------------------------------    

import numpy as np

import RCAIDE
from RCAIDE.Core import Units, Data
from RCAIDE.Framework.Analyses.Process import Process
from RCAIDE.Optimization.write_optimization_outputs import write_optimization_outputs
from RCAIDE.Methods.Performance.electric_payload_range import electric_payload_range
from RCAIDE.Methods.Performance.propeller_range_endurance_speeds import propeller_range_endurance_speeds

from RCAIDE.Plots.Performance.Weights import eVTOL_Sunburst, eVTOL_Icicle

# ----------------------------------------------------------------------        
#   Setup
# ----------------------------------------------------------------------   

def setup():
    
    # ------------------------------------------------------------------
    #   Analysis Procedure
    # ------------------------------------------------------------------ 
    
    # size the base config
    procedure = Process()
    procedure.simple_sizing = simple_sizing
    
    # find the weights
    procedure.weights = weight
    
    # finalizes the data dependencies
    procedure.finalize = finalize
    
    # performance studies
    procedure.missions                        = Process()
    procedure.missions.payload_range          = payload_range
    procedure.missions.range_mission          = range_mission
    procedure.missions.sprint_mission         = sprint_mission
    procedure.missions.hover_mission          = hover_mission
    procedure.missions.range_endurance_speeds = range_endurance_speeds

    # post process the results
    procedure.post_process = post_process
        
    return procedure


# ----------------------------------------------------------------------        
#   Design Mission
# ----------------------------------------------------------------------    
def range_mission(nexus):
    
    mission = nexus.missions.base
    results = nexus.results
    results.base = mission.evaluate()
    
    return nexus

# ----------------------------------------------------------------------        
#   Design Mission
# ----------------------------------------------------------------------    
def sprint_mission(nexus):
    
    mission = nexus.missions.sprint
    results = nexus.results
    results.sprint = mission.evaluate()
    
    return nexus

# ----------------------------------------------------------------------        
#   Hover Mission
# ----------------------------------------------------------------------    
def hover_mission(nexus):
    
    mission = nexus.missions.hover
    results = nexus.results
    results.hover = mission.evaluate()
    
    return nexus

# ----------------------------------------------------------------------        
#   Payload Range
# ----------------------------------------------------------------------    
def payload_range(nexus):
    
    # unpack
    mission = nexus.missions.range_mission
    vehicle = nexus.vehicle_configurations.base
    
    payload_range = electric_payload_range(vehicle, mission, 'cruise', display_plot=True)
    
    nexus.results.payload_range = payload_range
    
    print('Payload Range')
    print(payload_range)

    
    return nexus


# ----------------------------------------------------------------------        
#   Range and Endurance Airspeeds
# ----------------------------------------------------------------------    
def range_endurance_speeds(nexus):
    
    # unpack
    analyses  = nexus.analyses.base
    altitude  = 1000 * Units.feet
    CL_max    = 1.2
    up_bnd    = 250. * Units['mph']
    delta_isa = 0.
    
    results = propeller_range_endurance_speeds(analyses,altitude,CL_max,up_bnd,delta_isa)
    
    nexus.results.range_endurance_speeds = results
    
    print('Speeds and L/D')
    print(results)

    
    return nexus

# ----------------------------------------------------------------------        
#   Sizing
# ----------------------------------------------------------------------    

def simple_sizing(nexus):


    return nexus

# ----------------------------------------------------------------------        
#   Weights
# ----------------------------------------------------------------------    

def weight(nexus):
    vehicle=nexus.vehicle_configurations.base

    # weight analysis
    weights = nexus.analyses.base.weights.evaluate()
    vehicle.mass_properties.breakdown = weights
    
    eVTOL_Sunburst(vehicle)
    eVTOL_Icicle(vehicle)

    return nexus

# ----------------------------------------------------------------------
#   Finalizing Function
# ----------------------------------------------------------------------    

def finalize(nexus):
    
    nexus.analyses.finalize()   
    
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
    

    filename = 'results.txt'
    write_optimization_outputs(nexus, filename)
   
    return nexus    
