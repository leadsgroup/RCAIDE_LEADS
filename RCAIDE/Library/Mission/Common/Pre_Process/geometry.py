# RCAIDE/Library/Missions/Common/Pre_Process/geometry.py
# 
# 
# Created:  Apr 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Geometry.LOPA      import  compute_layout_of_passenger_accommodations
from RCAIDE.Library.Methods.Geometry.Planform  import  update_blended_wing_body_planform 
from RCAIDE.Library.Methods.Geometry.Planform  import  fuselage_planform, wing_planform, compute_fuel_volume
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------
#  aerodynamics
# ----------------------------------------------------------------------------------------------------------------------  
def geometry(mission):
    """
  
    """
     
    last_tag = None
    for tag,segment in mission.segments.items():  
        if segment.analyses.aerodynamics != None:
            if last_tag!=  None:
                segment.analyses.aerodynamics.vehicle = vehicle 
            else: 
                vehicle  =  segment.analyses.aerodynamics.vehicle
                
                # update fuselage properties
                for fuselage in vehicle.fuselages:
                    fuselage_planform(fuselage, update_fuselage_properties=False)  # These defaults need to be put somewhere else   
                
                # ensure all properties of wing are computed before drag calculations  
                for wing in vehicle.wings: 
                    if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
                        
                        #if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body): 
                            #compute_layout_of_passenger_accommodations(wing)  # These defaults need to be put somewhere else
                            
                            #update_blended_wing_body_planform(wing, update_planform = False)
                            
                        # compute planform properties 
                        wing_planform(wing,overwrite_reference = False)  # These defaults need to be put somewhere else  
                        
                        #vehicle.reference_area = wing.areas.reference
                    # else:
                    #     # compute planform properties 
                    #     wing_planform(wing)  # These default need to be put somewhere else
                        
                # compute fuel volume
                #compute_fuel_volume(vehicle, update_max_fuel=False)
                
                last_tag = tag 

            ## update weights vehicle with correct geometric properties                  
            #if segment.analyses.weights != None:
                #segment.analyses.weights.vehicle = vehicle
    return 