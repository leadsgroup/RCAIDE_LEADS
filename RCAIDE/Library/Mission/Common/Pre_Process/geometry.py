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
from RCAIDE.Library.Methods.Geometry.Planform  import  fuselage_planform, wing_planform, bwb_wing_planform #, compute_fuel_volume


# ----------------------------------------------------------------------------------------------------------------------
#  geometry
# ----------------------------------------------------------------------------------------------------------------------  
def geometry(mission):
    """
  
    """
     
    last_tag = None
    for tag,segment in mission.segments.items():  
        if hasattr(segment.analyses, 'geometry') and segment.analyses.geometry is not None:
            if last_tag!=  None:
                segment.analyses.geometry.vehicle = vehicle 
            else: 
                vehicle  =  segment.analyses.geometry.vehicle
                
                # update fuselage properties
                if segment.analyses.geometry.settings.update_fuselage_properties:
                    for fuselage in vehicle.fuselages:
                        compute_layout_of_passenger_accommodations(fuselage)
                        fuselage_planform(fuselage) 
                
                for wing in vehicle.wings: 

                    # ----------------------------
                    #  Blended Wing Body
                    # ----------------------------
                    if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                        if segment.analyses.geometry.settings.update_fuselage_properties:
                            compute_layout_of_passenger_accommodations(wing)  
                            update_blended_wing_body_planform(wing)
                        if segment.analyses.geometry.settings.update_wing_properties and segment.analyses.geometry.settings.overwrite_reference:
                            bwb_wing_planform(wing,overwrite_reference = True)
                            vehicle.reference_area = wing.areas.reference

                    # ----------------------------
                    # All other Wing Surfaces
                    # ----------------------------
                    else:
                        if segment.analyses.geometry.settings.update_wing_properties and  segment.analyses.geometry.settings.overwrite_reference:
                            wing_planform(wing,overwrite_reference = True) 
                        
                # ----------------------------
                # Compute Fuel Volume to be added in next PR
                # ----------------------------
                #if segment.analyses.geometry.update_fuel_volume:
                    #compute_fuel_volume(vehicle, update_max_fuel=False)
                
                last_tag = tag 

            # update weights vehicle with correct geometric properties                  
            if segment.analyses.weights != None:
                segment.analyses.weights.vehicle = vehicle
        else:
            raise AttributeError('Geometry Analyses not defined')
    return 