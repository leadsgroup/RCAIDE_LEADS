this function should be similar to evalute VLM , with a surrogte and no surrogate options 

# RCAIDE/Library/Methods/Aerstructures/Finite_Element_Analysis/evaluate_FEA.py
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
import RCAIDE 
from RCAIDE.Framework.Core                                               import Data, orientation_product 
from RCAIDE.Library.Methods.Aerstructures.Finite_Element_Analysis.FEA       import FEA
from RCAIDE.Library.Methods.Utilities                                    import Cubic_Spline_Blender 
from RCAIDE.Library.Mission.Common.Update                                import orientations
from RCAIDE.Library.Mission.Common.Unpack_Unknowns                       import orientation

# package imports
import numpy   as np
from copy      import  deepcopy 

# ----------------------------------------------------------------------------------------------------------------------
#  Finite_Element_Analysis
# ---------------------------------------------------------------------------------------------------------------------- 
def evaluate_surrogate(state,settings,vehicle):
    """Evaluates forces and moments using built surrogates 
    
    Assumptions:
        - Drag due to angle of attack is the dominant drag component and the only one calculated
        - Aircraft is symmetric such that CY_L_0 = 0 and CN_L_0 = 0
        
    Source:
        None

    Args:
        aerodynamics : VLM analysis          [unitless]
        state        : flight conditions     [unitless]
        settings     : VLM analysis settings [unitless]
        vehicle      : vehicle configuration [unitless] 
        
    Returns: 
        None  
    """          
    conditions       = state.conditions
    aerodynamics     = state.analyses.aerodynamics  
    sub_sur          = aerodynamics.surrogates.subsonic
    sup_sur          = aerodynamics.surrogates.supersonic
    trans_sur        = aerodynamics.surrogates.transonic  
    AoA              = np.atleast_2d(conditions.aerodynamics.angles.alpha)  
    Beta             = np.atleast_2d(conditions.aerodynamics.angles.beta)    
    Mach             = np.atleast_2d(conditions.freestream.mach_number)  
    ones_row         = np.ones_like(AoA)  
    hsub_min         = aerodynamics.hsub_min
    hsub_max         = aerodynamics.hsub_max
    hsup_min         = aerodynamics.hsup_min
    hsup_max         = aerodynamics.hsup_max

    # Spline for Subsonic-to-Transonic-to-Supersonic Regimes
    sub_trans_spline = Cubic_Spline_Blender(hsub_min,hsub_max)
    h_sub            = lambda M:sub_trans_spline.compute(M)          
    sup_trans_spline = Cubic_Spline_Blender(hsup_max, hsup_min) 
    h_sup            = lambda M:sup_trans_spline.compute(M)            
    
    #Alpha 
    pts_alpha   = np.hstack((AoA,Mach))
    results_alpha = compute_coefficients(sub_sur.Clift_alpha,  sub_sur.Cdrag_induced_alpha,  sub_sur.CX_alpha,  sub_sur.CY_alpha,  sub_sur.CZ_alpha,  sub_sur.CL_alpha,  sub_sur.CM_alpha,   sub_sur.CN_alpha,
                                         trans_sur.Clift_alpha,trans_sur.Cdrag_induced_alpha,trans_sur.CX_alpha,trans_sur.CY_alpha,trans_sur.CZ_alpha,trans_sur.CL_alpha,trans_sur.CM_alpha, trans_sur.CN_alpha,
                                         sup_sur.Clift_alpha,  sup_sur.Cdrag_induced_alpha,  sup_sur.CX_alpha,  sup_sur.CY_alpha,  sup_sur.CZ_alpha,  sup_sur.CL_alpha,  sup_sur.CM_alpha,   sup_sur.CN_alpha,
                                         h_sub,h_sup,Mach, pts_alpha)      
    Clift_alpha             = results_alpha.Clift   
    Cdrag_induced_alpha     = results_alpha.Cdrag
    CM                      = results_alpha.CM
    
    conditions.static_stability.coefficients.M_0 = compute_stability_derivative(sub_sur.CM_0    ,trans_sur.CM_0    ,sup_sur.CM_0    ,h_sub,h_sup,Mach) 
    
    for wing in vehicle.wings:   
        inviscid_wing_lifts = compute_coefficient(sub_sur.Clift_wing_alpha[wing.tag],trans_sur.Clift_wing_alpha[wing.tag],sup_sur.Cdrag_induced_wing_alpha[wing.tag] ,h_sub,h_sup,Mach,pts_alpha)
        inviscid_wing_drags = compute_coefficient(sub_sur.Cdrag_induced_wing_alpha[wing.tag],trans_sur.Cdrag_induced_wing_alpha[wing.tag],sup_sur.Cdrag_induced_wing_alpha[wing.tag] ,h_sub,h_sup,Mach,pts_alpha) 
        conditions.aerodynamics.coefficients.lift.inviscid.wings[wing.tag] =  inviscid_wing_lifts  
        conditions.aerodynamics.coefficients.drag.induced.wings[wing.tag]  =  inviscid_wing_drags       
    
         # -----------------------------------------------------------------------------------------------------------------------
    # Pack Aero Results 
    # -----------------------------------------------------------------------------------------------------------------------   
    conditions.aerodynamics.coefficients.lift.inviscid.total    = Clift_alpha
    conditions.aerodynamics.coefficients.drag.induced.inviscid  = Cdrag_induced_alpha
   
       return

def evaluate_no_surrogate(state,settings,vehicle):
    """Evaluates forces and moments directly using VLM.
    
    Assumptions:
        
    Source:
        None

    Args:
        aerodynamics       : VLM analysis  [unitless]
        state      : flight conditions     [unitless]
        settings   : VLM analysis settings [unitless]
        vehicle    : vehicle configuration [unitless] 
        
    Returns: 
        None  
    """          

    # unpack 
    conditions    = state.conditions 
    aerodynamics  = state.analyses.aerodynamics 
    n_cpts        = len(conditions.aerodynamics.angles.alpha)
     
    
    VLM_results = VLM(conditions,settings,vehicle)
    Clift = VLM_results.CLift
    Cdrag = VLM_results.CDrag_induced
    CX    = VLM_results.CX
    CY    = VLM_results.CY
    CZ    = VLM_results.CZ
    CL    = VLM_results.CL
    CM    = VLM_results.CM
    CN    = VLM_results.CN
 
    conditions.aerodynamics.coefficients.lift.inviscid.wings          = VLM_results.CLift_wings 
    conditions.aerodynamics.coefficients.lift.inviscid.total          = Clift
    conditions.aerodynamics.coefficients.lift.inviscid.spanwise       = VLM_results.spanwise_CLift
    conditions.aerodynamics.coefficients.drag.induced.wings           = VLM_results.CDrag_induced_wings
    conditions.aerodynamics.coefficients.drag.induced.spanwise        = VLM_results.sectional_CDrag_induced
    conditions.aerodynamics.coefficients.drag.induced.inviscid        = Cdrag
    conditions.aerodynamics.coefficients.surface_pressure             = VLM_results.CP
    conditions.aerodynamics.angles.induced                            = VLM_results.alpha_induced    
    conditions.aerodynamics.spanwise_stations                         = VLM_results.spanwise_stations

    # corrections 
    RCAIDE.Library.Methods.Aerodynamics.Common.Lift.fuselage_correction(state,settings,vehicle)     
    for wing in  vehicle.wings: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_wing(state,settings,wing)
    for fuslage in vehicle.fuselages: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_fuselage(state,settings,fuslage)
    for boom in vehicle.booms: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_fuselage(state,settings,boom)      
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_nacelle(state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_pylon(state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_total(state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.induced_drag(state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.cooling_drag(state,settings,vehicle)     
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressibility_drag(state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.miscellaneous_drag(state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.form_drag(state,settings,vehicle)  
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.wave_drag(state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.trim_drag(state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.total_drag(state,settings,vehicle)
    
           
    return

 
    
 


 