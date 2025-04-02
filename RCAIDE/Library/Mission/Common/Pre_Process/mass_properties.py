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
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Electric.VTOL.Physics_Based   import converge_physics_based_weight_buildup 
 
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
                
                if type(weights_analysis) == RCAIDE.Framework.Analyses.Weights.Electric and  weights_analysis.aircraft_type == 'VTOL':
                    # Converge weight of aircraft before doing final weight analyis
                    weights_analysis.print_weight_analysis_report = False
                    converged_vehicle,breakdown = converge_physics_based_weight_buildup(weights_analysis.vehicle)
                    print(breakdown)
                    weights_analysis.vehicle = converged_vehicle
                 
                _ = weights_analysis.evaluate() 
             
                # CG Location  
                CG_location, _ = compute_vehicle_center_of_gravity(weights_analysis.vehicle, update_CG= weights_analysis.settings.update_center_of_gravity)  
                 
                # Operating Aircraft MOI    
                _, _ = compute_aircraft_moment_of_inertia(weights_analysis.vehicle, CG_location, update_MOI= weights_analysis.settings.update_moment_of_inertia)        
                
                # assign mass properties to aerodynamics analyses 
                segment.analyses.aerodynamics.vehicle.mass_properties.operating_empty             = weights_analysis.vehicle.mass_properties.operating_empty
                segment.analyses.aerodynamics.vehicle.mass_properties.center_of_gravity           = weights_analysis.vehicle.mass_properties.center_of_gravity 
                segment.analyses.aerodynamics.vehicle.mass_properties.moments_of_inertia.tensor   = weights_analysis.vehicle.mass_properties.moments_of_inertia.tensor
                segment.analyses.energy.vehicle.mass_properties.operating_empty                   = weights_analysis.vehicle.mass_properties.operating_empty
                segment.analyses.energy.vehicle.mass_properties.center_of_gravity                 = weights_analysis.vehicle.mass_properties.center_of_gravity 
                segment.analyses.energy.vehicle.mass_properties.moments_of_inertia.tensor         = weights_analysis.vehicle.mass_properties.moments_of_inertia.tensor
                
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