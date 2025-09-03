# RCAIDE/Library/Methods/Aerodynamics/SU2/evaluate_SU2.py
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# package imports
import numpy   as np 

# ----------------------------------------------------------------------------------------------------------------------
#  evaluate_surrogate
# ---------------------------------------------------------------------------------------------------------------------- 
def evaluate_surrogate(state,settings,vehicle):
    """Evaluates surrogates forces and moments using built surrogates 
    
    Assumptions:
        
    Source:
        None

    Args:
        aerodynamics : SU2 analysis  [unitless]
        state        : flight conditions     [unitless]
        settings     : SU2 analysis settings [unitless]
        vehicle      : vehicle configuration [unitless] 
        
    Returns: 
        None  
    """          
    conditions    = state.conditions
    aerodynamics  = state.analyses.aerodynamics 
    sur       = aerodynamics.surrogates #.subsonic 
    ref_vals      = aerodynamics.reference_values
    AoA           = np.atleast_2d(conditions.aerodynamics.angles.alpha)    
    Mach          = np.atleast_2d(conditions.freestream.mach_number)  
     
    # loop through wings to determine what control surfaces are present 
    ###CANNOT TRIM AIRCRAFT FOR NOW
    # for wing in vehicle.wings: 
    #     for control_surface in wing.control_surfaces:  
    #         if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron:
    #             if trim !=  True:  
    #                 conditions.control_surfaces.aileron.deflection[:, 0] = control_surface.deflection
    #         if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator: 
    #             if trim !=  True:   
    #                 conditions.control_surfaces.elevator.deflection[:, 0] = control_surface.deflection
    #         if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder: 
    #             if trim !=  True:  
    #                 conditions.control_surfaces.rudder.deflection[:, 0] = control_surface.deflection
    #         if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Slat: 
    #             conditions.control_surfaces.slat.deflection[:, 0] = control_surface.deflection
    #         if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap:   
    #             conditions.control_surfaces.flap.deflection[:, 0] = control_surface.deflection
    #         if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Spoiler:   
    #             conditions.control_surfaces.spoiler.deflection[:, 0] = control_surface.deflection  

    # -----------------------------------------------------------------------------------------------------------------------
    # Query surrogates  
    # ----------------------------------------------------------------------------------------------------------------------- 
    pts   = np.hstack((AoA,Mach))    
    conditions.aerodynamics.coefficients.lift.inviscid.total            = np.atleast_2d(sur.Clift_alpha(pts)).T  
    conditions.aerodynamics.coefficients.drag.induced.inviscid          = np.atleast_2d(sur.Cdrag_alpha(pts)).T 
    conditions.static_stability.coefficients.M                          = np.atleast_2d(sur.CM_alpha(pts)).T   
    
    return 