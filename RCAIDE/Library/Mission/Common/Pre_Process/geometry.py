# RCAIDE/Library/Missions/Common/Pre_Process/geometry.py
# 
# 
# Created:  Apr 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Geometry.LOPA      import  compute_layout_of_passenger_accommodations 
from RCAIDE.Library.Methods.Geometry.Planform  import  fuselage_planform, wing_planform, bwb_wing_planform , compute_fuel_volume
from copy import  deepcopy

# ----------------------------------------------------------------------------------------------------------------------
#  geometry
# ----------------------------------------------------------------------------------------------------------------------  
def geometry(mission):
    """
  
    """  
    for i ,  segment in enumerate(mission.segments):

        # --------------------------------------------------------------------------------------------------------------------        
        # check if geometry analysis is defined 
        # --------------------------------------------------------------------------------------------------------------------
        if segment.analyses.geometry is None: 
            segment.analyses.geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
            segment.analyses.geometry.vehicle = segment.analyses.energy.vehicle
            
        # update fuselage properties
        if segment.analyses.geometry.settings.update_fuselage_properties:
            for fuselage in segment.analyses.geometry.vehicle.fuselages:
                compute_layout_of_passenger_accommodations(fuselage)
                fuselage_planform(fuselage) 
        
        for wing in segment.analyses.geometry.vehicle.wings: 

            # --------------------------------------------------------------------------------------------------------------------
            #  Blended Wing Body
            # --------------------------------------------------------------------------------------------------------------------
            if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
                if segment.analyses.geometry.settings.update_fuselage_properties:
                    compute_layout_of_passenger_accommodations(wing)  
                if segment.analyses.geometry.settings.update_wing_properties and segment.analyses.geometry.settings.overwrite_reference:
                    bwb_wing_planform(wing,overwrite_reference = True)
                    segment.analyses.geometry.vehicle.reference_area = wing.areas.reference

            # --------------------------------------------------------------------------------------------------------------------
            # All other wing surfaces
            # --------------------------------------------------------------------------------------------------------------------
            else:
                if segment.analyses.geometry.settings.update_wing_properties:
                    wing_planform(wing,overwrite_reference =  segment.analyses.geometry.settings.overwrite_reference) 
                    if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing) and segment.analyses.geometry.settings.overwrite_reference:
                        segment.analyses.geometry.vehicle.reference_area = wing.areas.reference
                
        # --------------------------------------------------------------------------------------------------------------------
        # Compute fuel volume  
        # --------------------------------------------------------------------------------------------------------------------
        if segment.analyses.geometry.settings.update_fuel_volume: 
            compute_fuel_volume(segment.analyses.geometry.vehicle, update_max_fuel=segment.analyses.geometry.settings.update_fuel_volume) 
                    
        if segment.analyses.weights == None: 
            weights = RCAIDE.Framework.Analyses.Weights.Weights()
            weights.vehicle = deepcopy(segment.analyses.geometry.vehicle)
            segment.analyses.weights = weights
                   
    return 