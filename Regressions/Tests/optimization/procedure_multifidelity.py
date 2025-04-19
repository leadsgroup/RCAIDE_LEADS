# Procedure.py

# ----------------------------------------------------------------------        
#   Imports
# ---------------------------------------------------------------------- 
from RCAIDE.Framework.Analyses.Process import Process

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

    # post process the results
    procedure.post_process = post_process
        
    # done!
    return procedure

# ----------------------------------------------------------------------        
#   Design Functions
# ----------------------------------------------------------------------    
def design_mission(nexus):
    
    mission = nexus.missions.base
    results = nexus.results
    results.base = mission.evaluate()
    
    return nexus

# ----------------------------------------------------------------------        
#   Analysis Setting
# ----------------------------------------------------------------------   

def post_process(nexus):
    
    x1 = nexus.vehicle_configurations.base.x1
    x2 = nexus.vehicle_configurations.base.x2
    
    if 'fidelity_level' not in nexus:
        print('Fidelity level not set, defaulting to 1')
        nexus.fidelity_level = 1

    if nexus.fidelity_level == 2:
        obj = np.array([x2**2+(x1+.1)**2])      
    elif nexus.fidelity_level == 1:
        obj = np.array([x2**2+x1**2])
    else:
        raise ValueError('Selected fidelity level not supported')
    nexus.obj = obj
    
    return nexus