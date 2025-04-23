# RCAIDE/Library/Missions/Common/Pre_Process/mass_properties.py
# 
# 
# Created: Mar 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  RCAIDE
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia                             import compute_aircraft_moment_of_inertia
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity                             import compute_vehicle_center_of_gravity
# ----------------------------------------------------------------------------------------------------------------------
#  mass_properties
# ----------------------------------------------------------------------------------------------------------------------  
def mass_properties(mission):
    """ Performances mass property analysis       
    """

    last_tag = None    
    for segment in mission.segments:
        if segment.analyses.weights != None:
            if last_tag == None:
                # run weights analysis 
                weights_analysis =  segment.analyses.weights
                
                _ = weights_analysis.evaluate() 
             
                # CG Location  
                CG_location, _ = compute_vehicle_center_of_gravity(weights_analysis.vehicle, update_CG= weights_analysis.settings.update_center_of_gravity)  
                 
                # Operating Aircraft MOI    
                _, _ = compute_aircraft_moment_of_inertia(weights_analysis.vehicle, CG_location, update_MOI= weights_analysis.settings.update_moment_of_inertia)   


                for tag, item in weights_analysis.settings.weight_correction_factors.items():
                        if tag == 'empty':
                            for subtag, subitem in weights_analysis.settings.weight_correction_factors[tag].items():
                                for subsubtag, subsubitem in weights_analysis.settings.weight_correction_factors[tag][subtag].items():
                                    # Subtract Weight 
                                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag].total  -= weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag].total  -= weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                                    weights_analysis.vehicle.mass_properties.operating_empty -= weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                                    # Multiply the factor
                                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag] *= subsubitem
                                    # Re add the weight
                                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag].total  += weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag].total  += weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                                    weights_analysis.vehicle.mass_properties.operating_empty += weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag]
                        elif tag == 'operational_items':
                                for subtag, subitem in weights_analysis.settings.weight_correction_factors[tag].items():
                                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag].total  -= subitem
                                    weights_analysis.vehicle.mass_properties.operating_empty -= subitem
                                    # Multiply the factor
                                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag] *= subitem
                                    # Re add the weight
                                    weights_analysis.vehicle.mass_properties.weight_breakdown[tag].total  += weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag]
                                    weights_analysis.vehicle.mass_properties.operating_empty += weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag]
                                

            
                for tag, item in weights_analysis.settings.weight_correction_additions.items():
                    if tag == 'empty':
                        for subtag, subitem in weights_analysis.settings.weight_correction_additions[tag].items():
                            for subsubtag, subsubitem in weights_analysis.settings.weight_correction_additions[tag][subtag].items():
                                weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag][subsubtag] = subsubitem
                                weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag].total += subsubitem
                                weights_analysis.vehicle.mass_properties.weight_breakdown[tag].total  += subsubitem
                                weights_analysis.vehicle.mass_properties.operating_empty += subsubitem
                    elif tag == 'operational_items':
                            for subtag, subitem in weights_analysis.settings.weight_correction_additions[tag].items():
                                weights_analysis.vehicle.mass_properties.weight_breakdown[tag][subtag] = subitem
                                weights_analysis.vehicle.mass_properties.weight_breakdown[tag].total  += subitem
                                weights_analysis.vehicle.mass_properties.operating_empty += subitem


                # assign mass properties to WEIGHTS analyses
                # segment.analyses.aerodynamics.vehicle.mass_properties.operating_empty             = weights_analysis.vehicle.mass_properties.operating_empty
                # segment.analyses.aerodynamics.vehicle.mass_properties.center_of_gravity           = weights_analysis.vehicle.mass_properties.center_of_gravity 
                # segment.analyses.aerodynamics.vehicle.mass_properties.moments_of_inertia.tensor   = weights_analysis.vehicle.mass_properties.moments_of_inertia.tensor
                # segment.analyses.energy.vehicle.mass_properties.operating_empty                   = weights_analysis.vehicle.mass_properties.operating_empty
                # segment.analyses.energy.vehicle.mass_properties.center_of_gravity                 = weights_analysis.vehicle.mass_properties.center_of_gravity 
                # segment.analyses.energy.vehicle.mass_properties.moments_of_inertia.tensor         = weights_analysis.vehicle.mass_properties.moments_of_inertia.tensor
                
                last_tag = segment.tag.lower()
            
            else:  
                segment.analyses.aerodynamics.vehicle.mass_properties.operating_empty             = mission.segments[last_tag].analyses.weights.vehicle.mass_properties.operating_empty
                segment.analyses.aerodynamics.vehicle.mass_properties.center_of_gravity           = mission.segments[last_tag].analyses.weights.vehicle.mass_properties.center_of_gravity 
                segment.analyses.aerodynamics.vehicle.mass_properties.moments_of_inertia.tensor   = mission.segments[last_tag].analyses.weights.vehicle.mass_properties.moments_of_inertia.tensor
                segment.analyses.weights.vehicle.mass_properties.operating_empty                  = mission.segments[last_tag].analyses.weights.vehicle.mass_properties.operating_empty
                segment.analyses.weights.vehicle.mass_properties.center_of_gravity                = mission.segments[last_tag].analyses.weights.vehicle.mass_properties.center_of_gravity 
                segment.analyses.weights.vehicle.mass_properties.moments_of_inertia.tensor        = mission.segments[last_tag].analyses.weights.vehicle.mass_properties.moments_of_inertia.tensor
                segment.analyses.energy.vehicle.mass_properties.operating_empty                   = mission.segments[last_tag].analyses.weights.vehicle.mass_properties.operating_empty
                segment.analyses.energy.vehicle.mass_properties.center_of_gravity                 = mission.segments[last_tag].analyses.weights.vehicle.mass_properties.center_of_gravity 
                segment.analyses.energy.vehicle.mass_properties.moments_of_inertia.tensor         = mission.segments[last_tag].analyses.weights.vehicle.mass_properties.moments_of_inertia.tensor 
    return 