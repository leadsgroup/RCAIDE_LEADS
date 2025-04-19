# RCAIDE/Library/Missions/Common/Pre_Process/aerodynamics.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Library.Methods.Geometry.Planform  import wing_planform

# ----------------------------------------------------------------------------------------------------------------------
#  aerodynamics
# ----------------------------------------------------------------------------------------------------------------------  
def aerodynamics(mission):
    """ Runs aerdoynamics model and build surrogate
    
        Assumptions:
            N/A
        
        Inputs:
            None
            
        Outputs:
            None  
    """
    
        
    last_tag = None
    for tag,segment in mission.segments.items():  
        if segment.analyses.aerodynamics != None:
            # ensure all properties of wing are computed before drag calculations  
            vehicle =  segment.analyses.aerodynamics.vehicle
            for wing in  vehicle.wings: 
                wing_planform(wing)
                
            if (last_tag!=  None) and  ('compute' in mission.segments[last_tag].analyses.aerodynamics.process.keys()): 
                segment.analyses.aerodynamics.process.compute.lift.inviscid_wings = mission.segments[last_tag].analyses.aerodynamics.process.compute.lift.inviscid_wings
                segment.analyses.aerodynamics.surrogates       = mission.segments[last_tag].analyses.aerodynamics.surrogates 
                segment.analyses.aerodynamics.reference_values = mission.segments[last_tag].analyses.aerodynamics.reference_values  
            else:          
                aero   = segment.analyses.aerodynamics
                aero.initialize()   
                last_tag = tag  
    return 