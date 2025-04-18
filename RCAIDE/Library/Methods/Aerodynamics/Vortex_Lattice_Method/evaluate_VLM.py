# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/evaluate_VLM.py
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
import RCAIDE 
from RCAIDE.Framework.Core                                           import Data, orientation_product 
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM   import VLM
from RCAIDE.Library.Methods.Utilities                                import Cubic_Spline_Blender  
from RCAIDE.Library.Mission.Common.Update  import orientations
from RCAIDE.Library.Mission.Common.Unpack_Unknowns import orientation

# package imports
import numpy   as np
from copy      import  deepcopy 

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ---------------------------------------------------------------------------------------------------------------------- 
def evaluate_surrogate(state,settings,vehicle):
    """Evaluates surrogates forces and moments using built surrogates 
    
    Assumptions:
        
    Source:
        None

    Args:
        aerodynamics : VLM analysis  [unitless]
        state        : flight conditions     [unitless]
        settings     : VLM analysis settings [unitless]
        vehicle      : vehicle configuration [unitless] 
        
    Returns: 
        None  
    """          
    conditions    = state.conditions
    aerodynamics  = state.analyses.aerodynamics 
    trim          = aerodynamics.settings.trim_aircraft
    sub_sur       = aerodynamics.surrogates.subsonic
    sup_sur       = aerodynamics.surrogates.supersonic
    trans_sur     = aerodynamics.surrogates.transonic 
    ref_vals      = aerodynamics.reference_values
    AoA           = np.atleast_2d(conditions.aerodynamics.angles.alpha)  
    Beta          = np.atleast_2d(conditions.aerodynamics.angles.beta)    
    Mach          = np.atleast_2d(conditions.freestream.mach_number)  
    Phi           = np.atleast_2d(conditions.aerodynamics.angles.phi)
    ones_row      = np.ones_like(AoA)
     
    # loop through wings to determine what control surfaces are present  
    for wing in vehicle.wings: 
        for control_surface in wing.control_surfaces:  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron:
                if trim !=  True:  
                    conditions.control_surfaces.aileron.deflection[:, 0] = control_surface.deflection
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator: 
                if trim !=  True:   
                    conditions.control_surfaces.elevator.deflection[:, 0] = control_surface.deflection
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder: 
                if trim !=  True:  
                    conditions.control_surfaces.rudder.deflection[:, 0] = control_surface.deflection
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap:   
                conditions.control_surfaces.flap.deflection[:, 0] = control_surface.deflection
  
    hsub_min         = aerodynamics.hsub_min
    hsub_max         = aerodynamics.hsub_max
    hsup_min         = aerodynamics.hsup_min
    hsup_max         = aerodynamics.hsup_max

    # Spline for Subsonic-to-Transonic-to-Supersonic Regimes
    sub_trans_spline = Cubic_Spline_Blender(hsub_min,hsub_max)
    h_sub            = lambda M:sub_trans_spline.compute(M)          
    sup_trans_spline = Cubic_Spline_Blender(hsup_max, hsup_min) 
    h_sup            = lambda M:sup_trans_spline.compute(M)
    
    # -----------------------------------------------------------------------------------------------------------------------
    # Stability Results Without Control Surfaces 
    # -----------------------------------------------------------------------------------------------------------------------
    conditions.S_ref  = ref_vals.S_ref              
    conditions.c_ref  = ref_vals.c_ref              
    conditions.b_ref  = ref_vals.b_ref
    conditions.X_ref  = ref_vals.X_ref
    conditions.Y_ref  = ref_vals.Y_ref
    conditions.Z_ref  = ref_vals.Z_ref
    
    # -----------------------------------------------------------------------------------------------------------------------
    # Query surrogates  
    # ----------------------------------------------------------------------------------------------------------------------- 
    #pts_alpha_0   = np.hstack((AoA*0.0,Mach))
    #results_alpha_0 = compute_coefficients(sub_sur.Clift_alpha,  sub_sur.Cdrag_alpha,  sub_sur.CX_alpha,  sub_sur.CY_alpha,  sub_sur.CZ_alpha,  sub_sur.CL_alpha,  sub_sur.CM_alpha,   sub_sur.CN_alpha,
    #                                     trans_sur.Clift_alpha,trans_sur.Cdrag_alpha,trans_sur.CX_alpha,trans_sur.CY_alpha,trans_sur.CZ_alpha,trans_sur.CL_alpha,trans_sur.CM_alpha, trans_sur.CN_alpha,
    #                                     sup_sur.Clift_alpha,  sup_sur.Cdrag_alpha,  sup_sur.CX_alpha,  sup_sur.CY_alpha,  sup_sur.CZ_alpha,  sup_sur.CL_alpha,  sup_sur.CM_alpha,   sup_sur.CN_alpha,
    #                                     h_sub,h_sup,Mach, pts_alpha_0)     
    #CM_alpha_0 = results_alpha_0.CM
    
    ## Beta
    #pts_beta    = np.hstack((Beta,Mach))
    #results_beta = compute_coefficients(sub_sur.Clift_beta,  sub_sur.Cdrag_beta,  sub_sur.CX_beta,  sub_sur.CY_beta,  sub_sur.CZ_beta,  sub_sur.CL_beta,  sub_sur.CM_beta,   sub_sur.CN_beta,
                                         #trans_sur.Clift_beta,trans_sur.Cdrag_beta,trans_sur.CX_beta,trans_sur.CY_beta,trans_sur.CZ_beta,trans_sur.CL_beta,trans_sur.CM_beta, trans_sur.CN_beta,
                                         #sup_sur.Clift_beta,  sup_sur.Cdrag_beta,  sup_sur.CX_beta,  sup_sur.CY_beta,  sup_sur.CZ_beta,  sup_sur.CL_beta,  sup_sur.CM_beta,   sup_sur.CN_beta,
                                         #h_sub,h_sup,Mach, pts_beta)
    
    #CL_beta                      = results_beta.CL
    #CY_beta                      = results_beta.CY
    #CN_beta                      = results_beta.CN
    
    #Alpha 
    pts_alpha   = np.hstack((AoA,Mach))
    results_alpha = compute_coefficients(sub_sur.Clift_alpha,  sub_sur.Cdrag_alpha,  sub_sur.CX_alpha,  sub_sur.CY_alpha,  sub_sur.CZ_alpha,  sub_sur.CL_alpha,  sub_sur.CM_alpha,   sub_sur.CN_alpha,
                                         trans_sur.Clift_alpha,trans_sur.Cdrag_alpha,trans_sur.CX_alpha,trans_sur.CY_alpha,trans_sur.CZ_alpha,trans_sur.CL_alpha,trans_sur.CM_alpha, trans_sur.CN_alpha,
                                         sup_sur.Clift_alpha,  sup_sur.Cdrag_alpha,  sup_sur.CX_alpha,  sup_sur.CY_alpha,  sup_sur.CZ_alpha,  sup_sur.CL_alpha,  sup_sur.CM_alpha,   sup_sur.CN_alpha,
                                         h_sub,h_sup,Mach, pts_alpha)      

    Clift_alpha             = results_alpha.Clift   
    Cdrag_alpha             = results_alpha.Cdrag 
    CM_alpha                = results_alpha.CM
    
    # -----------------------------------------------------------------------------------------------------------------------
    # Query control surface surrogates if derivatives are not user defined
    # ----------------------------------------------------------------------------------------------------------------------- 
    if aerodynamics.stability_derivatives.dCX_dalpha ==None:  
        conditions.static_stability.derivatives.CX_alpha    = compute_stability_derivative(sub_sur.dCX_dalpha    ,trans_sur.dCX_dalpha    ,sup_sur.dCX_dalpha    ,h_sub,h_sup,Mach)  
    else:
        conditions.static_stability.derivatives.CX_alpha    = aerodynamics.stability_derivatives.dCX_dalpha * ones_row
    
    if aerodynamics.stability_derivatives.dCZ_dalpha ==None:
        conditions.static_stability.derivatives.CZ_alpha    = compute_stability_derivative(sub_sur.dCZ_dalpha    ,trans_sur.dCZ_dalpha    ,sup_sur.dCZ_dalpha    ,h_sub,h_sup,Mach) 
    else:
        conditions.static_stability.derivatives.CZ_alpha    = aerodynamics.stability_derivatives.dCZ_dalpha * ones_row
    
    if aerodynamics.stability_derivatives.dCM_dalpha ==None:
        conditions.static_stability.derivatives.CM_alpha    = compute_stability_derivative(sub_sur.dCM_dalpha    ,trans_sur.dCM_dalpha    ,sup_sur.dCM_dalpha    ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CM_alpha    = aerodynamics.stability_derivatives.dCM_dalpha * ones_row
    
    if aerodynamics.stability_derivatives.dCY_dbeta ==None:
        conditions.static_stability.derivatives.CY_beta     = compute_stability_derivative(sub_sur.dCY_dbeta     ,trans_sur.dCY_dbeta     ,sup_sur.dCY_dbeta     ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CY_beta     = aerodynamics.stability_derivatives.dCY_dbeta * ones_row
    
    if aerodynamics.stability_derivatives.dCL_dbeta ==None:
        conditions.static_stability.derivatives.CL_beta     = compute_stability_derivative(sub_sur.dCL_dbeta     ,trans_sur.dCL_dbeta     ,sup_sur.dCL_dbeta     ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CL_beta     = aerodynamics.stability_derivatives.dCL_dbeta * ones_row
    
    if aerodynamics.stability_derivatives.dCN_dbeta ==None:
        conditions.static_stability.derivatives.CN_beta     = compute_stability_derivative(sub_sur.dCN_dbeta     ,trans_sur.dCN_dbeta     ,sup_sur.dCN_dbeta     ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CN_beta     = aerodynamics.stability_derivatives.dCN_dbeta * ones_row

    if aerodynamics.stability_derivatives.dCX_du ==None:
        conditions.static_stability.derivatives.CX_u        = compute_stability_derivative(sub_sur.dCX_du        ,trans_sur.dCX_du        ,sup_sur.dCX_du        ,h_sub,h_sup,Mach)   
    else:
        conditions.static_stability.derivatives.CX_u        = aerodynamics.stability_derivatives.dCX_du * ones_row
    
    if aerodynamics.stability_derivatives.dCZ_du ==None:
        conditions.static_stability.derivatives.CZ_u        = compute_stability_derivative(sub_sur.dCZ_du        ,trans_sur.dCZ_du        ,sup_sur.dCZ_du        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CZ_u        = aerodynamics.stability_derivatives.dCZ_du * ones_row
    
    if aerodynamics.stability_derivatives.dCM_du ==None:
        conditions.static_stability.derivatives.CM_u        = compute_stability_derivative(sub_sur.dCM_du        ,trans_sur.dCM_du        ,sup_sur.dCM_du        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CM_u        = aerodynamics.stability_derivatives.dCM_du * ones_row
    
    if aerodynamics.stability_derivatives.dCY_dr ==None:
        conditions.static_stability.derivatives.CY_r        = compute_stability_derivative(sub_sur.dCY_dr        ,trans_sur.dCY_dr        ,sup_sur.dCY_dr        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CY_r        = aerodynamics.stability_derivatives.dCY_dr * ones_row
    
    if aerodynamics.stability_derivatives.dCZ_dq ==None:
        conditions.static_stability.derivatives.CZ_q        = compute_stability_derivative(sub_sur.dCZ_dq        ,trans_sur.dCZ_dq        ,sup_sur.dCZ_dq        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CZ_q        = aerodynamics.stability_derivatives.dCZ_dq*ones_row
    
    if aerodynamics.stability_derivatives.dCL_dp ==None:
        conditions.static_stability.derivatives.CL_p        = compute_stability_derivative(sub_sur.dCL_dp        ,trans_sur.dCL_dp        ,sup_sur.dCL_dp        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CL_p        = aerodynamics.stability_derivatives.dCL_dp*ones_row
    
    if aerodynamics.stability_derivatives.dCL_dr ==None:
        conditions.static_stability.derivatives.CL_r        = compute_stability_derivative(sub_sur.dCL_dr        ,trans_sur.dCL_dr        ,sup_sur.dCL_dr        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CL_r        = aerodynamics.stability_derivatives.dCL_dr * ones_row
    
    if aerodynamics.stability_derivatives.dCM_dq ==None:
        conditions.static_stability.derivatives.CM_q        = compute_stability_derivative(sub_sur.dCM_dq        ,trans_sur.dCM_dq        ,sup_sur.dCM_dq        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CM_q        = aerodynamics.stability_derivatives.dCM_dq * ones_row
    
    if aerodynamics.stability_derivatives.dCN_dp ==None:
        conditions.static_stability.derivatives.CN_p        = compute_stability_derivative(sub_sur.dCN_dp        ,trans_sur.dCN_dp        ,sup_sur.dCN_dp        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CN_p        = aerodynamics.stability_derivatives.dCN_dp * ones_row
    
    if aerodynamics.stability_derivatives.dCN_dr ==None:
        conditions.static_stability.derivatives.CN_r        = compute_stability_derivative(sub_sur.dCN_dr        ,trans_sur.dCN_dr        ,sup_sur.dCN_dr        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CN_r        = aerodynamics.stability_derivatives.dCN_dr * ones_row


     
    conditions.static_stability.coefficients.lift   = Clift_alpha #* np.cos(Phi)   
    conditions.static_stability.coefficients.drag   = Cdrag_alpha # + Cdrag_u + Cdrag_w + Cdrag_q
    conditions.static_stability.coefficients.Y      = conditions.static_stability.derivatives.CY_beta *Beta 
    conditions.static_stability.coefficients.L      = conditions.static_stability.derivatives.CL_beta *Beta 
    conditions.static_stability.coefficients.M      = conditions.static_stability.derivatives.CM_alpha *AoA 
    conditions.static_stability.coefficients.N      = conditions.static_stability.derivatives.CN_beta *Beta

    conditions.static_stability.derivatives.Clift_alpha = compute_stability_derivative(sub_sur.dClift_dalpha        ,trans_sur.dClift_dalpha        ,sup_sur.dClift_dalpha        ,h_sub,h_sup,Mach)
    

 
    #C_W =  conditions.weights.total_mass * conditions.freestream.gravity / (conditions.freestream.dynamic_pressure * ref_vals.S_ref)    
    #conditions.static_stability.coefficients.Y  += C_W * np.sin(Phi)     
    
    # -----------------------------------------------------------------------------------------------------------------------
    # Addition of Control Surface Effect 
    # -----------------------------------------------------------------------------------------------------------------------    
    if aerodynamics.aileron_flag:  
        if aerodynamics.stability_derivatives.CY_delta_a == None:
            conditions.static_stability.derivatives.CY_delta_a     = compute_stability_derivative(sub_sur.dCY_ddelta_a     ,trans_sur.dCY_ddelta_a     ,sup_sur.dCY_ddelta_a     ,h_sub,h_sup,Mach)
        else:
            conditions.static_stability.derivatives.CY_delta_a = aerodynamics.stability_derivatives.CY_delta_a* ones_row
        
        if aerodynamics.stability_derivatives.CL_delta_a == None:
            conditions.static_stability.derivatives.CL_delta_a     = compute_stability_derivative(sub_sur.dCL_ddelta_a     ,trans_sur.dCL_ddelta_a     ,sup_sur.dCL_ddelta_a     ,h_sub,h_sup,Mach)
        else:
            conditions.static_stability.derivatives.CL_delta_a = aerodynamics.stability_derivatives.CL_delta_a* ones_row
        
        if aerodynamics.stability_derivatives.CN_delta_a == None:
            conditions.static_stability.derivatives.CN_delta_a     = compute_stability_derivative(sub_sur.dCN_ddelta_a     ,trans_sur.dCN_ddelta_a     ,sup_sur.dCN_ddelta_a     ,h_sub,h_sup,Mach) 
        else:
            conditions.static_stability.derivatives.CN_delta_a = aerodynamics.stability_derivatives.CN_delta_a* ones_row
        
        conditions.static_stability.coefficients.Y                                   += conditions.static_stability.derivatives.CY_delta_a * conditions.control_surfaces.aileron.deflection * np.sin(Phi)     
        conditions.static_stability.coefficients.L                                   += conditions.static_stability.derivatives.CL_delta_a * conditions.control_surfaces.aileron.deflection
        conditions.static_stability.coefficients.N                                   += conditions.static_stability.derivatives.CN_delta_a * conditions.control_surfaces.aileron.deflection
                     
        conditions.control_surfaces.aileron.static_stability.coefficients.Y          = conditions.static_stability.derivatives.CY_delta_a * conditions.control_surfaces.aileron.deflection * np.sin(Phi)               
        conditions.control_surfaces.aileron.static_stability.coefficients.L          = conditions.static_stability.derivatives.CL_delta_a * conditions.control_surfaces.aileron.deflection          
        conditions.control_surfaces.aileron.static_stability.coefficients.N          = conditions.static_stability.derivatives.CN_delta_a * conditions.control_surfaces.aileron.deflection             
        
    if aerodynamics.elevator_flag: 
        if aerodynamics.stability_derivatives.CM_delta_e == None:
            conditions.static_stability.derivatives.CM_delta_e     = compute_stability_derivative(sub_sur.dCM_ddelta_e     ,trans_sur.dCM_ddelta_e     ,sup_sur.dCM_ddelta_e     ,h_sub,h_sup,Mach)
        else:
            conditions.static_stability.derivatives.CM_delta_e = aerodynamics.stability_derivatives.CM_delta_e* ones_row
        
        #CM_delta_e  = conditions.static_stability.derivatives.CM_delta_e * conditions.control_surfaces.elevator.deflection       
        #CM_delta_e[conditions.control_surfaces.elevator.deflection==0.0]    = 0 
        
        conditions.static_stability.coefficients.M                              += conditions.static_stability.derivatives.CM_delta_e * conditions.control_surfaces.elevator.deflection  
        conditions.control_surfaces.elevator.static_stability.coefficients.M    = conditions.static_stability.derivatives.CM_delta_e * conditions.control_surfaces.elevator.deflection      
        
    if aerodynamics.rudder_flag:  
        if aerodynamics.stability_derivatives.CY_delta_r == None:
            conditions.static_stability.derivatives.CY_delta_r     = compute_stability_derivative(sub_sur.dCY_ddelta_r     ,trans_sur.dCY_ddelta_r     ,sup_sur.dCY_ddelta_r     ,h_sub,h_sup,Mach)
        else:
            conditions.static_stability.derivatives.CY_delta_r = aerodynamics.stability_derivatives.CY_delta_r* ones_row
        
        if aerodynamics.stability_derivatives.CL_delta_r == None:
            conditions.static_stability.derivatives.CL_delta_r     = compute_stability_derivative(sub_sur.dCL_ddelta_r     ,trans_sur.dCL_ddelta_r     ,sup_sur.dCL_ddelta_r     ,h_sub,h_sup,Mach)
        else:
            conditions.static_stability.derivatives.CL_delta_r = aerodynamics.stability_derivatives.CL_delta_r* ones_row
        
        if aerodynamics.stability_derivatives.CN_delta_r == None:
            conditions.static_stability.derivatives.CN_delta_r     = compute_stability_derivative(sub_sur.dCN_ddelta_r     ,trans_sur.dCN_ddelta_r     ,sup_sur.dCN_ddelta_r     ,h_sub,h_sup,Mach) 
        else:
            conditions.static_stability.derivatives.CN_delta_r = aerodynamics.stability_derivatives.CN_delta_r* ones_row
              
        #CY_delta_r      =conditions.static_stability.derivatives.CY_delta_r * conditions.control_surfaces.rudder.deflection        
        #CL_delta_r      =conditions.static_stability.derivatives.CL_delta_r * conditions.control_surfaces.rudder.deflection      
        #CN_delta_r      =conditions.static_stability.derivatives.CN_delta_r * conditions.control_surfaces.rudder.deflection 
        #CY_delta_r[conditions.control_surfaces.rudder.deflection==0.0]    = 0          
        #CL_delta_r[conditions.control_surfaces.rudder.deflection==0.0]    = 0          
        #CN_delta_r[conditions.control_surfaces.rudder.deflection==0.0]    = 0
        
        conditions.static_stability.coefficients.Y                                   += conditions.static_stability.derivatives.CY_delta_r * conditions.control_surfaces.rudder.deflection * np.sin(Phi)     
        conditions.static_stability.coefficients.L                                   += conditions.static_stability.derivatives.CL_delta_r * conditions.control_surfaces.rudder.deflection
        conditions.static_stability.coefficients.N                                   += conditions.static_stability.derivatives.CN_delta_r * conditions.control_surfaces.rudder.deflection
        conditions.control_surfaces.rudder.static_stability.coefficients.Y           = conditions.static_stability.derivatives.CY_delta_r * conditions.control_surfaces.rudder.deflection * np.sin(Phi)            
        conditions.control_surfaces.rudder.static_stability.coefficients.L           = conditions.static_stability.derivatives.CL_delta_r * conditions.control_surfaces.rudder.deflection        
        conditions.control_surfaces.rudder.static_stability.coefficients.N           = conditions.static_stability.derivatives.CN_delta_r * conditions.control_surfaces.rudder.deflection       
        
    if aerodynamics.flap_flag:
        if aerodynamics.stability_derivatives.CM_delta_f == None:
            conditions.static_stability.derivatives.CM_delta_f     = compute_stability_derivative(sub_sur.dCM_ddelta_f     ,trans_sur.dCM_ddelta_f     ,sup_sur.dCM_ddelta_f     ,h_sub,h_sup,Mach)
        else:
            conditions.static_stability.derivatives.CM_delta_f = aerodynamics.stability_derivatives.CM_delta_f* ones_row
        
        #CM_delta_f      = conditions.static_stability.derivatives.CM_delta_f * conditions.control_surfaces.flap.deflection    
        #CM_delta_f[conditions.control_surfaces.flap.deflection==0.0]    = 0
        
        conditions.static_stability.coefficients.M                                   += conditions.static_stability.derivatives.CM_delta_f * conditions.control_surfaces.flap.deflection  
        conditions.control_surfaces.flap.static_stability.coefficients.M             = conditions.static_stability.derivatives.CM_delta_f * conditions.control_surfaces.flap.deflection            
    
    for wing in vehicle.wings:   
        inviscid_wing_lifts = compute_coefficient(sub_sur.Clift_wing_alpha[wing.tag],trans_sur.Clift_wing_alpha[wing.tag],sup_sur.Cdrag_wing_alpha[wing.tag] ,h_sub,h_sup,Mach,pts_alpha)
        inviscid_wing_drags = compute_coefficient(sub_sur.Cdrag_wing_alpha[wing.tag],trans_sur.Cdrag_wing_alpha[wing.tag],sup_sur.Cdrag_wing_alpha[wing.tag] ,h_sub,h_sup,Mach,pts_alpha)
        # Pack 
        conditions.aerodynamics.coefficients.lift.induced.inviscid_wings[wing.tag] =  inviscid_wing_lifts 
        conditions.aerodynamics.coefficients.lift.compressible_wings[wing.tag]     =  inviscid_wing_lifts 
        conditions.aerodynamics.coefficients.drag.induced.inviscid_wings[wing.tag] =  inviscid_wing_drags   
    
    conditions.aerodynamics.coefficients.lift.total            = conditions.static_stability.coefficients.lift 
    conditions.aerodynamics.coefficients.drag.induced.inviscid = conditions.static_stability.coefficients.drag
    
    return

def evaluate_no_surrogate(state,settings,base_vehicle):
    """Evaluates forces and moments directly using VLM.
    
    Assumptions:
        The following stability derivatives are multiplied by correction 
        factors to match with literature/flight tests:
            CY_beta multiplied by 2
            CL_beta multiplied by -1
            CL_p multiplied by -2
            CM_q multiplied by 10
            CN_p multiplied by -3
            CN_r multiplied by 3
            CL_delta_a multiplied by -1
            CN_delta_a multiplied by -10
            CLift_delta_e multiplied by 0.5
        
    Source:
        None

    Args:
        aerodynamics       : VLM analysis          [unitless]
        state      : flight conditions     [unitless]
        settings   : VLM analysis settings [unitless]
        vehicle    : vehicle configuration [unitless] 
        
    Returns: 
        None  
    """          

    # unpack 
    conditions    = state.conditions   
    aerodynamics  = state.analyses.aerodynamics
    vehicle       = aerodynamics.vehicle 
    Mach          = state.conditions.freestream.mach_number
    trim          = aerodynamics.settings.trim_aircraft 

    for wing in vehicle.wings: 
        for control_surface in wing.control_surfaces:  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron: 
                settings.aileron_flag  = True                  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator:   
                settings.elevator_flag = True  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder:   
                settings.rudder_flag   = True  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Slat: 
                settings.slat_flag     = True   
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap: 
                settings.flap_flag     = True    
    
    for i in range(len(Mach)): 
        for wing in vehicle.wings: 
            for control_surface in wing.control_surfaces:  
                if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron:  
                    if trim ==  True:
                        control_surface.deflection = conditions.control_surfaces.aileron.deflection[i,0]
                    else: 
                        conditions.control_surfaces.aileron.deflection[i, 0] = control_surface.deflection
                        
                if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator:    
                    if trim ==  True: 
                        control_surface.deflection = conditions.control_surfaces.elevator.deflection[i,0]
                    else:   
                        conditions.control_surfaces.elevator.deflection[i, 0] = control_surface.deflection
                        
                if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder:    
                    if trim ==  True: 
                        control_surface.deflection = conditions.control_surfaces.rudder.deflection[i,0]
                    else:   
                        conditions.control_surfaces.rudder.deflection[i, 0] = control_surface.deflection
                                            
                if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Slat:  
                    conditions.control_surfaces.slat.deflection[i, 0] = control_surface.deflection
                    
                if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap:   
                    conditions.control_surfaces.flap.deflection[i, 0] = control_surface.deflection
        
        VLM_results = VLM(conditions,settings,vehicle)
        Clift = VLM_results.CLift
        Cdrag = VLM_results.CDrag_induced
        CX    = VLM_results.CX
        CY    = VLM_results.CY
        CZ    = VLM_results.CZ
        CL    = VLM_results.CL
        CM    = VLM_results.CM
        CN    = VLM_results.CN
        S_ref = VLM_results.S_ref
        b_ref = VLM_results.b_ref
        c_ref = VLM_results.c_ref
        X_ref = VLM_results.X_ref
        Y_ref = VLM_results.Y_ref
        Z_ref = VLM_results.Z_ref
        
        # Dimensionalize the lift and drag for each wing  
        conditions.aerodynamics.coefficients.lift.induced.inviscid_wings  = VLM_results.CLift_wings
        conditions.aerodynamics.coefficients.lift.compressible_wings      = VLM_results.CLift_wings        
        conditions.aerodynamics.coefficients.drag.induced.inviscid_wings  = VLM_results.CDrag_induced_wings
        conditions.aerodynamics.coefficients.lift.induced.spanwise        = VLM_results.sectional_CLift
        conditions.aerodynamics.coefficients.drag.induced.spanwise        = VLM_results.sectional_CDrag_induced
        conditions.aerodynamics.coefficients.surface_pressure             = VLM_results.CP
        conditions.aerodynamics.coefficients.lift.total                   = Clift
        conditions.aerodynamics.coefficients.drag.induced.inviscid        = Cdrag
        conditions.aerodynamics.angles.induced                            = VLM_results.alpha_induced 
        conditions.aerodynamics.chord_sections                            = VLM_results.chord_sections    
        conditions.aerodynamics.spanwise_stations                         = VLM_results.spanwise_stations  
        
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
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.spoiler_drag(state,settings,vehicle)
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.total_drag(state,settings,vehicle)  
    
        T_wind2inertial = conditions.frames.wind.transform_to_inertial 
        Cdrag_visc      = state.conditions.aerodynamics.coefficients.drag.total
        CX_visc         = orientation_product(T_wind2inertial,Cdrag_visc)[:,0][:,None]   
      
        no_beta   = np.all(conditions.aerodynamics.angles.beta == 0)
        no_ail    = np.all(conditions.control_surfaces.aileron.deflection == 0) 
        no_rud    = np.all(conditions.control_surfaces.rudder.deflection == 0) 
        no_bank   = np.all(conditions.aerodynamics.angles.phi == 0)  
        
        if no_beta and no_ail and no_rud and no_bank:
            CY = CY * 0
        conditions.static_stability.coefficients.lift[i, 0]  = Clift[i, 0]
        conditions.static_stability.coefficients.drag[i, 0]  = Cdrag_visc[i, 0] 
        conditions.static_stability.coefficients.X[i, 0]     = CX[i, 0]
        conditions.static_stability.coefficients.Y[i, 0]     = CY[i, 0]
        conditions.static_stability.coefficients.Z[i, 0]     = CZ[i, 0]
        conditions.static_stability.coefficients.L[i, 0]     = CL[i, 0]
        conditions.static_stability.coefficients.M[i, 0]     = CM[i, 0] 
        conditions.static_stability.coefficients.N[i, 0]     = CN[i, 0]     

    # --------------------------------------------------------------------------------------------      
    # Unpack Pertubations 
    # --------------------------------------------------------------------------------------------   
    delta_angle     = aerodynamics.training.angle_purtubation     
    delta_speed     = aerodynamics.training.speed_purtubation   
    delta_rate      = aerodynamics.training.rate_purtubation
    delta_ctrl_surf = aerodynamics.training.control_surface_purtubation

    # --------------------------------------------------------------------------------------------      
    # Equilibrium Condition 
    # --------------------------------------------------------------------------------------------       

    atmosphere                                                         = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                                          = atmosphere.compute_values(altitude = conditions.freestream.altitude)    

    equilibrium_conditions                                             = RCAIDE.Framework.Mission.Common.Results()
    equilibrium_conditions.energy                                      = deepcopy(conditions.energy)
    equilibrium_conditions.freestream.density[:,0]                     = atmo_data.density[0,0]
    equilibrium_conditions.freestream.gravity[:,0]                     = conditions.freestream.gravity[0,0]
    equilibrium_conditions.freestream.speed_of_sound[:,0]              = atmo_data.speed_of_sound[0,0] 
    equilibrium_conditions.freestream.dynamic_viscosity[:,0]           = atmo_data.dynamic_viscosity[0,0]
    equilibrium_conditions.aerodynamics.angles.alpha[:,0]              = 1E-12
    equilibrium_conditions.freestream.temperature[:,0]                 = atmo_data.temperature[0,0] 
    equilibrium_conditions.freestream.velocity[:,0]                    = conditions.freestream.velocity[0,0]             
    equilibrium_conditions.frames.inertial.velocity_vector[:,0]        = conditions.frames.inertial.velocity_vector[0,0] 
    equilibrium_conditions.freestream.mach_number                      = equilibrium_conditions.freestream.velocity/equilibrium_conditions.freestream.speed_of_sound
    equilibrium_conditions.freestream.dynamic_pressure                 = 0.5 * equilibrium_conditions.freestream.density *  (equilibrium_conditions.freestream.velocity ** 2)
    equilibrium_conditions.freestream.reynolds_number                  = equilibrium_conditions.freestream.density * equilibrium_conditions.freestream.velocity * wing.chords.mean_aerodynamic/ equilibrium_conditions.freestream.dynamic_viscosity  
    
    VLM_results = VLM(equilibrium_conditions,settings,vehicle)
    Clift_0 = VLM_results.CLift
    Cdrag_0 = VLM_results.CDrag_induced
    CX_0    = VLM_results.CX
    CY_0    = VLM_results.CY
    CZ_0    = VLM_results.CZ
    CL_0    = VLM_results.CL
    CM_0    = VLM_results.CM
    CN_0    = VLM_results.CN
     
    # Dimensionalize the lift and drag for each wing  
    equilibrium_conditions.aerodynamics.coefficients.lift.induced.inviscid_wings      = VLM_results.CLift_wings        
    equilibrium_conditions.aerodynamics.coefficients.lift.compressible_wings          = VLM_results.CLift_wings        
    equilibrium_conditions.aerodynamics.coefficients.drag.induced.inviscid_wings      = VLM_results.CDrag_induced_wings
    equilibrium_conditions.aerodynamics.coefficients.lift.total                       = Clift_0
    equilibrium_conditions.aerodynamics.coefficients.drag.induced.inviscid            = Cdrag_0     

    equilibrium_state                    = RCAIDE.Framework.Mission.Common.State()
    equilibrium_state.conditions         = equilibrium_conditions  
    equilibrium_segment                  = RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude()
    equilibrium_segment.conditions       = equilibrium_conditions
    equilibrium_segment.state.conditions = equilibrium_conditions
    orientation(equilibrium_segment)
    orientations(equilibrium_segment)
    
    for wing in  vehicle.wings: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_wing(equilibrium_state,settings,wing)
    for fuslage in vehicle.fuselages: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_fuselage(equilibrium_state,settings,fuslage)
    for boom in vehicle.booms: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_fuselage(equilibrium_state,settings,boom)  
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_nacelle(equilibrium_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_pylon(equilibrium_state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_total(equilibrium_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.induced_drag(equilibrium_state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.cooling_drag(equilibrium_state,settings,vehicle)     
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressibility_drag(equilibrium_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.miscellaneous_drag(equilibrium_state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.spoiler_drag(equilibrium_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.total_drag(equilibrium_state,settings,vehicle)
    
    T_wind2inertial = equilibrium_conditions.frames.wind.transform_to_inertial 
    Cdrag_0         = equilibrium_state.conditions.aerodynamics.coefficients.drag.total
    CX_0            = orientation_product(T_wind2inertial,Cdrag_0)[:,0][:,None]
     
    # --------------------------------------------------------------------------------------------      
    # Alpha Purtubation  
    # --------------------------------------------------------------------------------------------    
    perturbation_state                                 = deepcopy(equilibrium_state)
    pertubation_conditions                             = deepcopy(equilibrium_conditions)   
    pertubation_conditions.aerodynamics.angles.alpha   += delta_angle
    
    VLM_results = VLM(pertubation_conditions,settings,vehicle)
    Clift_alpha_prime = VLM_results.CLift
    Cdrag_alpha_prime = VLM_results.CDrag_induced 
    CY_alpha_prime    = VLM_results.CY
    CZ_alpha_prime    = VLM_results.CZ
    CL_alpha_prime    = VLM_results.CL
    CM_alpha_prime    = VLM_results.CM
    CN_alpha_prime    = VLM_results.CN
     
    pertubation_conditions.aerodynamics.coefficients.lift.induced.inviscid_wings  = VLM_results.CLift_wings
    pertubation_conditions.aerodynamics.coefficients.lift.compressible_wings      = VLM_results.CLift_wings          
    pertubation_conditions.aerodynamics.coefficients.drag.induced.inviscid_wings  = VLM_results.CDrag_induced_wings  
    pertubation_conditions.aerodynamics.coefficients.lift.total                =  Clift_alpha_prime
    pertubation_conditions.aerodynamics.coefficients.drag.induced.inviscid     =  Cdrag_alpha_prime
     
    perturbation_state                  = RCAIDE.Framework.Mission.Common.State()
    perturbation_state.conditions       = pertubation_conditions  
    perturbation_state                  = RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude()
    perturbation_state.conditions       = pertubation_conditions
    perturbation_state.state.conditions = pertubation_conditions
    orientation(perturbation_state)
    orientations(perturbation_state) 
    
    for wing in  vehicle.wings: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_wing(perturbation_state,settings,wing)
    for fuslage in vehicle.fuselages: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_fuselage(perturbation_state,settings,fuslage)
    for boom in vehicle.booms: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_fuselage(perturbation_state,settings,boom)  
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_nacelle(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_pylon(perturbation_state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_total(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.induced_drag(perturbation_state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.cooling_drag(perturbation_state,settings,vehicle)     
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressibility_drag(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.miscellaneous_drag(perturbation_state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.spoiler_drag(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.total_drag(perturbation_state,settings,vehicle) 

    T_wind2inertial   = pertubation_conditions.frames.wind.transform_to_inertial 
    Cdrag_visc_prime  = perturbation_state.conditions.aerodynamics.coefficients.drag.total
    CX_visc_prime     = orientation_product(T_wind2inertial,Cdrag_visc_prime)[:,0][:,None] 
    
    conditions.static_stability.derivatives.Clift_alpha = (Clift_alpha_prime   - Clift_0) / (delta_angle)
    conditions.static_stability.derivatives.Cdrag_alpha = (Cdrag_alpha_prime   - Cdrag_0) / (delta_angle)  
    conditions.static_stability.derivatives.CX_alpha    = (CX_visc_prime       - CX_0) / (delta_angle)   
    conditions.static_stability.derivatives.CY_alpha    = (CY_alpha_prime      - CY_0) / (delta_angle)  
    conditions.static_stability.derivatives.CZ_alpha    = (CZ_alpha_prime      - CZ_0) / (delta_angle) 
    conditions.static_stability.derivatives.CL_alpha    = (CL_alpha_prime      - CL_0) / (delta_angle)  
    conditions.static_stability.derivatives.CM_alpha    = (CM_alpha_prime      - CM_0) / (delta_angle)  
    conditions.static_stability.derivatives.CN_alpha    = (CN_alpha_prime      - CN_0) / (delta_angle) 

    # --------------------------------------------------------------------------------------------      
    # Neutral Point - CG Purtubation 
    # --------------------------------------------------------------------------------------------
    perturbation_state                                 = deepcopy(equilibrium_state)
    pertubation_conditions                             = deepcopy(equilibrium_conditions)  
    pertubation_conditions.aerodynamics.angles.alpha   += delta_angle

    vehicle_shifted_CG = deepcopy(vehicle)
    delta_cg = 0.1
    vehicle_shifted_CG.mass_properties.center_of_gravity[0][0] +=delta_cg
    
    
    VLM_results = VLM(pertubation_conditions,settings,vehicle_shifted_CG)  
    CM_alpha_cg_prime  = VLM_results.CM  
  
    dCM_dalpha_cg = (CM_alpha_cg_prime   - CM_0) / (delta_angle)    
    dCM_dalpha    = (CM_alpha_prime   - CM_0) / (delta_angle)    
     
    m  =  (dCM_dalpha_cg[0] - dCM_dalpha[0]) /delta_cg 
    b  =  dCM_dalpha_cg[0]  - (m * vehicle_shifted_CG.mass_properties.center_of_gravity[0][0])
    NP =  -b / m  
     
    conditions.static_stability.neutral_point[0,0] = NP
    vehicle.mass_properties.neutral_point = NP 
    
    # --------------------------------------------------------------------------------------------      
    # Beta Purtubation  
    # --------------------------------------------------------------------------------------------   
    pertubation_conditions                             = deepcopy(equilibrium_conditions)   
    pertubation_conditions.aerodynamics.angles.beta    += delta_angle 

    VLM_results = VLM(pertubation_conditions,settings,vehicle)
    Clift_beta_prime = VLM_results.CLift
    Cdrag_beta_prime = VLM_results.CDrag_induced
    CX_beta_prime    = VLM_results.CX
    CY_beta_prime    = VLM_results.CY
    CZ_beta_prime    = VLM_results.CZ
    CL_beta_prime    = VLM_results.CL
    CM_beta_prime    = VLM_results.CM
    CN_beta_prime    = VLM_results.CN
    
    conditions.static_stability.derivatives.Clift_beta = (Clift_beta_prime   - Clift_0) / (delta_angle)
    conditions.static_stability.derivatives.Cdrag_beta = (Cdrag_beta_prime   - Cdrag_0) / (delta_angle) 
    conditions.static_stability.derivatives.CX_beta    = (CX_beta_prime      - CX_0) / (delta_angle)  
    conditions.static_stability.derivatives.CY_beta    = 2*(CY_beta_prime      - CY_0) / (delta_angle) 
    conditions.static_stability.derivatives.CZ_beta    = (CZ_beta_prime      - CZ_0) / (delta_angle) 
    conditions.static_stability.derivatives.CL_beta    = -(CL_beta_prime      - CL_0) / (delta_angle)   
    conditions.static_stability.derivatives.CM_beta    = (CM_beta_prime      - CM_0) / (delta_angle)  
    conditions.static_stability.derivatives.CN_beta    = (CN_beta_prime      - CN_0) / (delta_angle) 

    # --------------------------------------------------------------------------------------------      
    # U-Velocity Pertubation 
    # --------------------------------------------------------------------------------------------
    perturbation_state                                           = deepcopy(equilibrium_state)
    pertubation_conditions                                       = deepcopy(equilibrium_conditions) 
    pertubation_conditions.frames.inertial.velocity_vector[:,0]  += delta_speed 
    pertubation_conditions.freestream.velocity            [:,0]  += delta_speed 
    pertubation_conditions.freestream.mach_number                = np.linalg.norm(pertubation_conditions.frames.inertial.velocity_vector, axis=1)[:,None] /  equilibrium_conditions.freestream.speed_of_sound 
    pertubation_conditions.freestream.reynolds_number            = pertubation_conditions.freestream.density * pertubation_conditions.freestream.velocity * wing.chords.mean_aerodynamic/equilibrium_conditions.freestream.dynamic_viscosity   
    pertubation_conditions.freestream.dynamic_pressure           = 0.5 * pertubation_conditions.freestream.density * np.sum( pertubation_conditions.freestream.velocity**2, axis=1)[:,None] 
        
    VLM_results = VLM(pertubation_conditions,settings,vehicle)
    Clift_u_prime = VLM_results.CLift
    Cdrag_u_prime = VLM_results.CDrag_induced
    CX_u_prime    = VLM_results.CX
    CY_u_prime    = VLM_results.CY
    CZ_u_prime    = VLM_results.CZ
    CL_u_prime    = VLM_results.CL
    CM_u_prime    = VLM_results.CM
    CN_u_prime    = VLM_results.CN 

    # Dimensionalize the lift and drag for each wing 
    for wing in vehicle.wings: 
        pertubation_conditions.aerodynamics.coefficients.lift.induced.inviscid_wings  = VLM_results.CLift_wings   
        pertubation_conditions.aerodynamics.coefficients.lift.compressible_wings      = VLM_results.CLift_wings        
        pertubation_conditions.aerodynamics.coefficients.drag.induced.inviscid_wings  = VLM_results.CDrag_induced_wings
    pertubation_conditions.aerodynamics.coefficients.lift.total                =  Clift_u_prime
    pertubation_conditions.aerodynamics.coefficients.drag.induced.inviscid     =  Cdrag_u_prime

    perturbation_state                  = RCAIDE.Framework.Mission.Common.State()
    perturbation_state.conditions       = pertubation_conditions  
    perturbation_state                  = RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude()
    perturbation_state.conditions       = pertubation_conditions
    perturbation_state.state.conditions = pertubation_conditions
    orientation(perturbation_state)
    orientations(perturbation_state)
    
    for wing in  vehicle.wings: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_wing(perturbation_state,settings,wing)
    for fuslage in vehicle.fuselages: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_fuselage(perturbation_state,settings,fuslage)
    for boom in vehicle.booms: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_fuselage(perturbation_state,settings,boom)  
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_nacelle(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_pylon(perturbation_state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_total(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.induced_drag(perturbation_state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.cooling_drag(perturbation_state,settings,vehicle)     
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressibility_drag(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.miscellaneous_drag(perturbation_state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.spoiler_drag(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.total_drag(perturbation_state,settings,vehicle) 

    T_wind2inertial   = pertubation_conditions.frames.wind.transform_to_inertial 
    Cdrag_visc_prime  = perturbation_state.conditions.aerodynamics.coefficients.drag.total
    CX_visc_prime     = orientation_product(T_wind2inertial,Cdrag_visc_prime)[:,0][:,None]        
 
    conditions.static_stability.derivatives.Clift_u = (Clift_u_prime   - Clift_0) / (delta_speed)
    conditions.static_stability.derivatives.Cdrag_u = (Cdrag_visc_prime   - Cdrag_0) / (delta_speed) 
    conditions.static_stability.derivatives.CX_u    = (CX_visc_prime   - CX_0) / (delta_speed)   
    conditions.static_stability.derivatives.CY_u    = (CY_u_prime      - CY_0) / (delta_speed) 
    conditions.static_stability.derivatives.CZ_u    = (CZ_u_prime      - CZ_0) / (delta_speed) 
    conditions.static_stability.derivatives.CL_u    = (CL_u_prime      - CL_0) / (delta_speed)  
    conditions.static_stability.derivatives.CM_u    = (CM_u_prime      - CM_0) / (delta_speed)  
    conditions.static_stability.derivatives.CN_u    = (CN_u_prime      - CN_0) / (delta_speed) 

    # --------------------------------------------------------------------------------------------      
    # V-Velocity Pertubation 
    # ------------------------------------------------------------------------------------------- 
    pertubation_conditions                                       = deepcopy(equilibrium_conditions)    
    pertubation_conditions.frames.inertial.velocity_vector[:,1]  += delta_speed
    pertubation_conditions.freestream.velocity                   = np.linalg.norm(pertubation_conditions.frames.inertial.velocity_vector, axis=1)[:,None] 
    pertubation_conditions.freestream.mach_number                = pertubation_conditions.freestream.velocity/ pertubation_conditions.freestream.speed_of_sound   
    pertubation_conditions.freestream.reynolds_number            = pertubation_conditions.freestream.density * pertubation_conditions.freestream.velocity / pertubation_conditions.freestream.dynamic_viscosity   
    pertubation_conditions.freestream.dynamic_pressure           = 0.5 * pertubation_conditions.freestream.density * np.sum( pertubation_conditions.freestream.velocity**2, axis=1)[:,None] 
    

    VLM_results = VLM(pertubation_conditions,settings,vehicle)
    Clift_v_prime = VLM_results.CLift
    Cdrag_v_prime = VLM_results.CDrag_induced
    CX_v_prime    = VLM_results.CX
    CY_v_prime    = VLM_results.CY
    CZ_v_prime    = VLM_results.CZ
    CL_v_prime    = VLM_results.CL
    CM_v_prime    = VLM_results.CM
    CN_v_prime    = VLM_results.CN
    
    conditions.static_stability.derivatives.Clift_v = (Clift_v_prime   - Clift_0) / (delta_speed)
    conditions.static_stability.derivatives.Cdrag_v = (Cdrag_v_prime   - Cdrag_0) / (delta_speed) 
    conditions.static_stability.derivatives.CX_v    = (CX_v_prime      - CX_0) / (delta_speed)  
    conditions.static_stability.derivatives.CY_v    = (CY_v_prime      - CY_0) / (delta_speed) 
    conditions.static_stability.derivatives.CZ_v    = (CZ_v_prime      - CZ_0) / (delta_speed) 
    conditions.static_stability.derivatives.CL_v    = (CL_v_prime      - CL_0) / (delta_speed)  
    conditions.static_stability.derivatives.CM_v    = (CM_v_prime      - CM_0) / (delta_speed)  
    conditions.static_stability.derivatives.CN_v    = (CN_v_prime      - CN_0) / (delta_speed)        

    # --------------------------------------------------------------------------------------------      
    # W-Velocity Pertubation 
    # --------------------------------------------------------------------------------------------  
    pertubation_conditions                                       = deepcopy(equilibrium_conditions)     
    pertubation_conditions.frames.inertial.velocity_vector[:,2]  += delta_speed 
    pertubation_conditions.freestream.velocity                   = np.linalg.norm(pertubation_conditions.frames.inertial.velocity_vector, axis=1)[:,None]     
    pertubation_conditions.freestream.mach_number                =pertubation_conditions.freestream.velocity / pertubation_conditions.freestream.speed_of_sound   
    pertubation_conditions.freestream.reynolds_number            = pertubation_conditions.freestream.density * pertubation_conditions.freestream.velocity /  pertubation_conditions.freestream.dynamic_viscosity 
    pertubation_conditions.freestream.dynamic_pressure           = 0.5 * pertubation_conditions.freestream.density * np.sum( pertubation_conditions.freestream.velocity**2, axis=1)[:,None] 
     
    VLM_results = VLM(pertubation_conditions,settings,vehicle)
    Clift_w_prime = VLM_results.CLift
    Cdrag_w_prime = VLM_results.CDrag_induced
    CX_w_prime    = VLM_results.CX
    CY_w_prime    = VLM_results.CY
    CZ_w_prime    = VLM_results.CZ
    CL_w_prime    = VLM_results.CL
    CM_w_prime    = VLM_results.CM
    CN_w_prime    = VLM_results.CN
    
    conditions.static_stability.derivatives.Clift_w  = (Clift_w_prime   - Clift_0) / (delta_speed)
    conditions.static_stability.derivatives.Cdrag_w  = (Cdrag_w_prime   - Cdrag_0) / (delta_speed) 
    conditions.static_stability.derivatives.CX_w     = (CX_w_prime      - CX_0) / (delta_speed)  
    conditions.static_stability.derivatives.CY_w     = (CY_w_prime      - CY_0) / (delta_speed) 
    conditions.static_stability.derivatives.CZ_w     = (CZ_w_prime      - CZ_0) / (delta_speed) 
    conditions.static_stability.derivatives.CL_w     = (CL_w_prime      - CL_0) / (delta_speed)  
    conditions.static_stability.derivatives.CM_w     = (CM_w_prime      - CM_0) / (delta_speed)  
    conditions.static_stability.derivatives.CN_w     = (CN_w_prime      - CN_0) / (delta_speed)
    

    # --------------------------------------------------------------------------------------------      
    # Roll Rate (p) Purtubation
    # --------------------------------------------------------------------------------------------  
    pertubation_conditions                                 = deepcopy(equilibrium_conditions)    
    pertubation_conditions.static_stability.roll_rate[:,0] += delta_rate
    
    VLM_results   = VLM(pertubation_conditions,settings,vehicle)
    Clift_p_prime = VLM_results.CLift
    Cdrag_p_prime = VLM_results.CDrag_induced
    CX_p_prime    = VLM_results.CX
    CY_p_prime    = VLM_results.CY
    CZ_p_prime    = VLM_results.CZ
    CL_p_prime    = VLM_results.CL
    CM_p_prime    = VLM_results.CM
    CN_p_prime    = VLM_results.CN
    
    conditions.static_stability.derivatives.Clift_p  = (Clift_p_prime   - Clift_0) / (delta_rate)
    conditions.static_stability.derivatives.Cdrag_p  = (Cdrag_p_prime   - Cdrag_0) / (delta_rate) 
    conditions.static_stability.derivatives.CX_p     = (CX_p_prime      - CX_0) / (delta_rate)  
    conditions.static_stability.derivatives.CY_p     = (CY_p_prime      - CY_0) / (delta_rate) 
    conditions.static_stability.derivatives.CZ_p     = (CZ_p_prime      - CZ_0) / (delta_rate) 
    conditions.static_stability.derivatives.CL_p     = -2*(CL_p_prime      - CL_0) / (delta_rate)  
    conditions.static_stability.derivatives.CM_p     = (CM_p_prime      - CM_0) / (delta_rate)  
    conditions.static_stability.derivatives.CN_p     = -3*(CN_p_prime      - CN_0) / (delta_rate)

    # ---------------------------------------------------------------------------------------------------      
    # Pitch Rate (q) Purtubation
    # ---------------------------------------------------------------------------------------------------    
    perturbation_state                                      = deepcopy(equilibrium_state)
    pertubation_conditions                                  = deepcopy(equilibrium_conditions)   
    pertubation_conditions.static_stability.pitch_rate[:,0] += delta_rate  
     
    VLM_results   = VLM(pertubation_conditions,settings,vehicle)
    Clift_q_prime = VLM_results.CLift
    Cdrag_q_prime = VLM_results.CDrag_induced
    CX_q_prime    = VLM_results.CX
    CY_q_prime    = VLM_results.CY
    CZ_q_prime    = VLM_results.CZ
    CL_q_prime    = VLM_results.CL
    CM_q_prime    = VLM_results.CM
    CN_q_prime    = VLM_results.CN
    
    # Dimensionalize the lift and drag for each wing  
    pertubation_conditions.aerodynamics.coefficients.lift.induced.inviscid_wings   = VLM_results.CLift_wings   
    pertubation_conditions.aerodynamics.coefficients.lift.compressible_wings       = VLM_results.CLift_wings        
    pertubation_conditions.aerodynamics.coefficients.drag.induced.inviscid_wings   = VLM_results.CDrag_induced_wings
    pertubation_conditions.aerodynamics.coefficients.lift.total                =  Clift_q_prime
    pertubation_conditions.aerodynamics.coefficients.drag.induced.inviscid     =  Cdrag_q_prime
    
    perturbation_state                  = RCAIDE.Framework.Mission.Common.State()
    perturbation_state.conditions       = pertubation_conditions  
    perturbation_state                  = RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude()
    perturbation_state.conditions       = pertubation_conditions
    perturbation_state.state.conditions = pertubation_conditions
    orientation(perturbation_state)
    orientations(perturbation_state)
    
    for wing in  vehicle.wings: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_wing(perturbation_state,settings,wing)
    for fuslage in vehicle.fuselages: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_fuselage(perturbation_state,settings,fuslage)
    for boom in vehicle.booms: 
        RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_fuselage(perturbation_state,settings,boom)  
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_nacelle(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_pylon(perturbation_state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_total(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.induced_drag(perturbation_state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.cooling_drag(perturbation_state,settings,vehicle)     
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressibility_drag(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.miscellaneous_drag(perturbation_state,settings,vehicle) 
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.spoiler_drag(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.total_drag(perturbation_state,settings,vehicle) 

    T_wind2inertial   = pertubation_conditions.frames.wind.transform_to_inertial 
    Cdrag_visc_prime  = perturbation_state.conditions.aerodynamics.coefficients.drag.total
    CX_visc_prime     = orientation_product(T_wind2inertial,Cdrag_visc_prime)[:,0][:,None]
  
    conditions.static_stability.derivatives.Clift_q  = (Clift_q_prime   - Clift_0) / (delta_rate)
    conditions.static_stability.derivatives.Cdrag_q  = (Cdrag_q_prime   - Cdrag_0)  / (delta_rate)
    conditions.static_stability.derivatives.CX_q     = (CX_visc_prime   - CX_0)/ (delta_rate)
    conditions.static_stability.derivatives.CY_q     = (CY_q_prime      - CY_0)  / (delta_rate)  
    conditions.static_stability.derivatives.CZ_q     = (CZ_q_prime      - CZ_0)   / (delta_rate)
    conditions.static_stability.derivatives.CL_q     = (CL_q_prime      - CL_0) / (delta_rate)  
    conditions.static_stability.derivatives.CM_q     = 10*(CM_q_prime      - CM_0) / (delta_rate)  
    conditions.static_stability.derivatives.CN_q     = (CN_q_prime      - CN_0)/ (delta_rate)   

    # ---------------------------------------------------------------------------------------------------      
    # Yaw Rate (r) Purtubation
    # ---------------------------------------------------------------------------------------------------     
    pertubation_conditions                                = deepcopy(equilibrium_conditions)   
    pertubation_conditions.static_stability.yaw_rate[:,0] += delta_rate   
    
    VLM_results   = VLM(pertubation_conditions,settings,vehicle)
    Clift_r_prime = VLM_results.CLift
    Cdrag_r_prime = VLM_results.CDrag_induced
    CX_r_prime    = VLM_results.CX
    CY_r_prime    = VLM_results.CY
    CZ_r_prime    = VLM_results.CZ
    CL_r_prime    = VLM_results.CL
    CM_r_prime    = VLM_results.CM
    CN_r_prime    = VLM_results.CN
     
    conditions.static_stability.derivatives.Clift_r  = (Clift_r_prime   - Clift_0) / (delta_rate)
    conditions.static_stability.derivatives.Cdrag_r  = (Cdrag_r_prime   - Cdrag_0) / (delta_rate) 
    conditions.static_stability.derivatives.CX_r     = (CX_r_prime      - CX_0) / (delta_rate)  
    conditions.static_stability.derivatives.CY_r     = (CY_r_prime      - CY_0) / (delta_rate) 
    conditions.static_stability.derivatives.CZ_r     = (CZ_r_prime      - CZ_0) / (delta_rate) 
    conditions.static_stability.derivatives.CL_r     = (CL_r_prime      - CL_0) / (delta_rate) 
    conditions.static_stability.derivatives.CM_r     = (CM_r_prime      - CM_0) / (delta_rate)  
    conditions.static_stability.derivatives.CN_r     = 3*(CN_r_prime      - CN_0) / (delta_rate) 
 
    # only compute derivative if control surface exists
    if settings.aileron_flag:  
        pertubation_conditions                             = deepcopy(equilibrium_conditions)  
        for wing in vehicle.wings: 
            for control_surface in wing.control_surfaces:  
                if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron:  
                    vehicle.wings[wing.tag].control_surfaces.aileron.deflection =  delta_ctrl_surf
                    
                    VLM_results = VLM(pertubation_conditions,settings,vehicle)
                    Clift_res = VLM_results.CLift
                    Cdrag_res = VLM_results.CDrag_induced
                    CX_res    = VLM_results.CX
                    CY_res    = VLM_results.CY
                    CZ_res    = VLM_results.CZ
                    CL_res    = VLM_results.CL
                    CM_res    = VLM_results.CM
                    CN_res    = VLM_results.CN
                    
                    vehicle.wings[wing.tag].control_surfaces.aileron.deflection = 0
        Clift_delta_a_prime   = Clift_res
        Cdrag_delta_a_prime   = Cdrag_res
        CX_delta_a_prime      = CX_res   
        CY_delta_a_prime      = CY_res   
        CZ_delta_a_prime      = CZ_res   
        CL_delta_a_prime      = CL_res   
        CM_delta_a_prime      = CM_res   
        CN_delta_a_prime      = CN_res   
        
        dClift_ddelta_a = (Clift_delta_a_prime   - Clift_0) / (delta_ctrl_surf)
        dCdrag_ddelta_a = (Cdrag_delta_a_prime   - Cdrag_0) / (delta_ctrl_surf)  
        dCX_ddelta_a    = (CX_delta_a_prime      - CX_0) / (delta_ctrl_surf)  
        dCY_ddelta_a    = (CY_delta_a_prime      - CY_0) / (delta_ctrl_surf) 
        dCZ_ddelta_a    = (CZ_delta_a_prime      - CZ_0) / (delta_ctrl_surf) 
        dCL_ddelta_a    = -(CL_delta_a_prime      - CL_0) / (delta_ctrl_surf)  
        dCM_ddelta_a    = (CM_delta_a_prime      - CM_0) / (delta_ctrl_surf)  
        dCN_ddelta_a    = -10*(CN_delta_a_prime      - CN_0) / (delta_ctrl_surf) 
        
        conditions.static_stability.derivatives.Clift_delta_a = dClift_ddelta_a 
        conditions.static_stability.derivatives.Cdrag_delta_a = dCdrag_ddelta_a 
        conditions.static_stability.derivatives.CX_delta_a    = dCX_ddelta_a    
        conditions.static_stability.derivatives.CY_delta_a    = dCY_ddelta_a    
        conditions.static_stability.derivatives.CZ_delta_a    = dCZ_ddelta_a    
        conditions.static_stability.derivatives.CL_delta_a    = dCL_ddelta_a    
        conditions.static_stability.derivatives.CM_delta_a    = dCM_ddelta_a    
        conditions.static_stability.derivatives.CN_delta_a    = dCN_ddelta_a
         
    if settings.elevator_flag:   
        pertubation_conditions                             = deepcopy(equilibrium_conditions)  
        for wing in vehicle.wings: 
            for control_surface in wing.control_surfaces:  
                if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator:  
                    vehicle.wings[wing.tag].control_surfaces.elevator.deflection =  delta_ctrl_surf

                    VLM_results = VLM(pertubation_conditions,settings,vehicle)
                    Clift_res = VLM_results.CLift
                    Cdrag_res = VLM_results.CDrag_induced
                    CX_res    = VLM_results.CX
                    CY_res    = VLM_results.CY
                    CZ_res    = VLM_results.CZ
                    CL_res    = VLM_results.CL
                    CM_res    = VLM_results.CM
                    CN_res    = VLM_results.CN 
                    vehicle.wings[wing.tag].control_surfaces.elevator.deflection = 0  
         
        Clift_delta_e_prime   = Clift_res
        Cdrag_delta_e_prime   = Cdrag_res
        CX_delta_e_prime      = CX_res   
        CY_delta_e_prime      = CY_res   
        CZ_delta_e_prime      = CZ_res   
        CL_delta_e_prime      = CL_res   
        CM_delta_e_prime      = CM_res   
        CN_delta_e_prime      = CN_res   
        
        dClift_ddelta_e = 0.5*(Clift_delta_e_prime   - Clift_0) / (delta_ctrl_surf)
        dCdrag_ddelta_e = (Cdrag_delta_e_prime   - Cdrag_0) / (delta_ctrl_surf)  
        dCX_ddelta_e    = (CX_delta_e_prime      - CX_0) / (delta_ctrl_surf)  
        dCY_ddelta_e    = (CY_delta_e_prime      - CY_0) / (delta_ctrl_surf) 
        dCZ_ddelta_e    = (CZ_delta_e_prime      - CZ_0) / (delta_ctrl_surf) 
        dCL_ddelta_e    = (CL_delta_e_prime      - CL_0) / (delta_ctrl_surf)  
        dCM_ddelta_e    = (CM_delta_e_prime      - CM_0) / (delta_ctrl_surf)  
        dCN_ddelta_e    = (CN_delta_e_prime      - CN_0) / (delta_ctrl_surf)
        
    
        conditions.static_stability.derivatives.Clift_delta_e = dClift_ddelta_e 
        conditions.static_stability.derivatives.Cdrag_delta_e = dCdrag_ddelta_e 
        conditions.static_stability.derivatives.CX_delta_e    = dCX_ddelta_e    
        conditions.static_stability.derivatives.CY_delta_e    = dCY_ddelta_e    
        conditions.static_stability.derivatives.CZ_delta_e    = dCZ_ddelta_e    
        conditions.static_stability.derivatives.CL_delta_e    = dCL_ddelta_e    
        conditions.static_stability.derivatives.CM_delta_e    = dCM_ddelta_e    
        conditions.static_stability.derivatives.CN_delta_e    = dCN_ddelta_e   
        
    if settings.rudder_flag: 
        pertubation_conditions                             = deepcopy(equilibrium_conditions)  
        for wing in vehicle.wings: 
            for control_surface in wing.control_surfaces:  
                if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder:  
                    vehicle.wings[wing.tag].control_surfaces.rudder.deflection =  delta_ctrl_surf 
                    VLM_results = VLM(pertubation_conditions,settings,vehicle)
                    Clift_res = VLM_results.CLift
                    Cdrag_res = VLM_results.CDrag_induced
                    CX_res    = VLM_results.CX
                    CY_res    = VLM_results.CY
                    CZ_res    = VLM_results.CZ
                    CL_res    = VLM_results.CL
                    CM_res    = VLM_results.CM
                    CN_res    = VLM_results.CN 
                    vehicle.wings[wing.tag].control_surfaces.rudder.deflection = 0
                     
        Clift_delta_r_prime   = Clift_res
        Cdrag_delta_r_prime   = Cdrag_res
        CX_delta_r_prime      = CX_res   
        CY_delta_r_prime      = CY_res   
        CZ_delta_r_prime      = CZ_res   
        CL_delta_r_prime      = CL_res   
        CM_delta_r_prime      = CM_res   
        CN_delta_r_prime      = CN_res   
        
        dClift_ddelta_r = (Clift_delta_r_prime   - Clift_0) / (delta_ctrl_surf)
        dCdrag_ddelta_r = (Cdrag_delta_r_prime   - Cdrag_0) / (delta_ctrl_surf)  
        dCX_ddelta_r    = (CX_delta_r_prime      - CX_0) / (delta_ctrl_surf)  
        dCY_ddelta_r    = (CY_delta_r_prime      - CY_0) / (delta_ctrl_surf) 
        dCZ_ddelta_r    = (CZ_delta_r_prime      - CZ_0) / (delta_ctrl_surf) 
        dCL_ddelta_r    = (CL_delta_r_prime      - CL_0) / (delta_ctrl_surf)  
        dCM_ddelta_r    = (CM_delta_r_prime      - CM_0) / (delta_ctrl_surf)  
        dCN_ddelta_r    = (CN_delta_r_prime      - CN_0) / (delta_ctrl_surf) 
    
        conditions.static_stability.derivatives.Clift_delta_r = dClift_ddelta_r 
        conditions.static_stability.derivatives.Cdrag_delta_r = dCdrag_ddelta_r 
        conditions.static_stability.derivatives.CX_delta_r    = dCX_ddelta_r    
        conditions.static_stability.derivatives.CY_delta_r    = dCY_ddelta_r    
        conditions.static_stability.derivatives.CZ_delta_r    = dCZ_ddelta_r    
        conditions.static_stability.derivatives.CL_delta_r    = dCL_ddelta_r    
        conditions.static_stability.derivatives.CM_delta_r    = dCM_ddelta_r    
        conditions.static_stability.derivatives.CN_delta_r    = dCN_ddelta_r
        
    if settings.flap_flag: 
        pertubation_conditions                             = deepcopy(equilibrium_conditions)

        for wing in vehicle.wings: 
            for control_surface in wing.control_surfaces:  
                if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap:  
                    vehicle.wings[wing.tag].control_surfaces.flap.deflection =  delta_ctrl_surf 
                    VLM_results = VLM(pertubation_conditions,settings,vehicle)
                    Clift_res = VLM_results.CLift
                    Cdrag_res = VLM_results.CDrag_induced
                    CX_res    = VLM_results.CX
                    CY_res    = VLM_results.CY
                    CZ_res    = VLM_results.CZ
                    CL_res    = VLM_results.CL
                    CM_res    = VLM_results.CM
                    CN_res    = VLM_results.CN
                    vehicle.wings[wing.tag].control_surfaces.flap.deflection = 0
                     
                    
        Clift_delta_f_prime   = Clift_res
        Cdrag_delta_f_prime   = Cdrag_res
        CX_delta_f_prime      = CX_res   
        CY_delta_f_prime      = CY_res   
        CZ_delta_f_prime      = CZ_res   
        CL_delta_f_prime      = CL_res   
        CM_delta_f_prime      = CM_res   
        CN_delta_f_prime      = CN_res   
        
        dClift_ddelta_f = (Clift_delta_f_prime   - Clift_0) / (delta_ctrl_surf)
        dCdrag_ddelta_f = (Cdrag_delta_f_prime   - Cdrag_0) / (delta_ctrl_surf)  
        dCX_ddelta_f    = (CX_delta_f_prime      - CX_0) / (delta_ctrl_surf)  
        dCY_ddelta_f    = (CY_delta_f_prime      - CY_0) / (delta_ctrl_surf) 
        dCZ_ddelta_f    = (CZ_delta_f_prime      - CZ_0) / (delta_ctrl_surf) 
        dCL_ddelta_f    = (CL_delta_f_prime      - CL_0) / (delta_ctrl_surf)  
        dCM_ddelta_f    = (CM_delta_f_prime      - CM_0) / (delta_ctrl_surf)  
        dCN_ddelta_f    = (CN_delta_f_prime      - CN_0) / (delta_ctrl_surf)
        
    
        conditions.static_stability.derivatives.Clift_delta_f = dClift_ddelta_f 
        conditions.static_stability.derivatives.Clift_delta_f = dCdrag_ddelta_f 
        conditions.static_stability.derivatives.CX_delta_f    = dCX_ddelta_f    
        conditions.static_stability.derivatives.CY_delta_f    = dCY_ddelta_f    
        conditions.static_stability.derivatives.CZ_delta_f    = dCZ_ddelta_f    
        conditions.static_stability.derivatives.CL_delta_f    = dCL_ddelta_f    
        conditions.static_stability.derivatives.CM_delta_f    = dCM_ddelta_f    
        conditions.static_stability.derivatives.CN_delta_f    = dCN_ddelta_f
         
                     
    if settings.slat_flag:
        pertubation_conditions                             = deepcopy(equilibrium_conditions) 
        for wing in vehicle.wings: 
            for control_surface in wing.control_surfaces:  
                if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Slat:  
                    vehicle.wings[wing.tag].control_surfaces.slat.deflection =  delta_ctrl_surf 
                    VLM_results = VLM(pertubation_conditions,settings,vehicle)
                    Clift_res = VLM_results.CLift
                    Cdrag_res = VLM_results.CDrag_induced
                    CX_res    = VLM_results.CX
                    CY_res    = VLM_results.CY
                    CZ_res    = VLM_results.CZ
                    CL_res    = VLM_results.CL
                    CM_res    = VLM_results.CM
                    CN_res    = VLM_results.CN  
                    vehicle.wings[wing.tag].control_surfaces.slat.deflection = 0
                     
        Clift_delta_s_prime   = Clift_res
        Cdrag_delta_s_prime   = Cdrag_res
        CX_delta_s_prime      = CX_res   
        CY_delta_s_prime      = CY_res   
        CZ_delta_s_prime      = CZ_res   
        CL_delta_s_prime      = CL_res   
        CM_delta_s_prime      = CM_res   
        CN_delta_s_prime      = CN_res   
        
        dClift_ddelta_s = (Clift_delta_s_prime   - Clift_0) / (delta_ctrl_surf)
        dCdrag_ddelta_s = (Cdrag_delta_s_prime   - Cdrag_0) / (delta_ctrl_surf)  
        dCX_ddelta_s    = (CX_delta_s_prime      - CX_0) / (delta_ctrl_surf)  
        dCY_ddelta_s    = (CY_delta_s_prime      - CY_0) / (delta_ctrl_surf) 
        dCZ_ddelta_s    = (CZ_delta_s_prime      - CZ_0) / (delta_ctrl_surf) 
        dCL_ddelta_s    = (CL_delta_s_prime      - CL_0) / (delta_ctrl_surf)  
        dCM_ddelta_s    = (CM_delta_s_prime      - CM_0) / (delta_ctrl_surf)  
        dCN_ddelta_s    = (CN_delta_s_prime      - CN_0) / (delta_ctrl_surf)
        
    
        conditions.static_stability.derivatives.Clift_delta_s = dClift_ddelta_s 
        conditions.static_stability.derivatives.Cdrag_delta_s = dCdrag_ddelta_s 
        conditions.static_stability.derivatives.CX_delta_s    = dCX_ddelta_s    
        conditions.static_stability.derivatives.CY_delta_s    = dCY_ddelta_s    
        conditions.static_stability.derivatives.CZ_delta_s    = dCZ_ddelta_s    
        conditions.static_stability.derivatives.CL_delta_s    = dCL_ddelta_s    
        conditions.static_stability.derivatives.CM_delta_s    = dCM_ddelta_s    
        conditions.static_stability.derivatives.CN_delta_s    = dCN_ddelta_s 
          
    # Stability Results  
    conditions.S_ref  = S_ref              
    conditions.c_ref  = c_ref              
    conditions.b_ref  = b_ref
    conditions.X_ref  = X_ref
    conditions.Y_ref  = Y_ref
    conditions.Z_ref  = Z_ref  

    return

def compute_stability_derivative(sub_sur,trans_sur,sup_sur,h_sub,h_sup,Mach):
    if trans_sur ==  None and  sup_sur == None:
        derivative = h_sub(Mach)*sub_sur(Mach) 
        return derivative
        
    derivative = h_sub(Mach)*sub_sur(Mach) +   (1 - (h_sup(Mach) + h_sub(Mach)))*trans_sur(Mach)  + h_sup(Mach)*sup_sur(Mach) 
    return derivative



def compute_coefficients(sub_sur_Clift,sub_sur_Cdrag,sub_sur_CX,sub_sur_CY,sub_sur_CZ,sub_sur_CL,sub_sur_CM,sub_sur_CN,
                         trans_sur_Clift,trans_sur_Cdrag,trans_sur_CX,trans_sur_CY,trans_sur_CZ,trans_sur_CL,trans_sur_CM,trans_sur_CN,
                         sup_sur_Clift,sup_sur_Cdrag,sup_sur_CX,sup_sur_CY,sup_sur_CZ,sup_sur_CL,sup_sur_CM,sup_sur_CN,
                         h_sub,h_sup,Mach, pts): 
    

     #  subsonic 
    sub_Clift     = np.atleast_2d(sub_sur_Clift(pts)).T  
    sub_Cdrag     = np.atleast_2d(sub_sur_Cdrag(pts)).T  
    sub_CX        = np.atleast_2d(sub_sur_CX(pts)).T 
    sub_CY        = np.atleast_2d(sub_sur_CY(pts)).T     
    sub_CZ        = np.atleast_2d(sub_sur_CZ(pts)).T     
    sub_CL        = np.atleast_2d(sub_sur_CL(pts)).T     
    sub_CM        = np.atleast_2d(sub_sur_CM(pts)).T     
    sub_CN        = np.atleast_2d(sub_sur_CN(pts)).T
    
    
    if trans_sur_Clift ==  None and  sup_sur_Clift == None:
    
        results       = Data() 
        results.Clift = h_sub(Mach) * sub_Clift
        results.Cdrag = h_sub(Mach) * sub_Cdrag
        results.CX    = h_sub(Mach) * sub_CX   
        results.CY    = h_sub(Mach) * sub_CY   
        results.CZ    = h_sub(Mach) * sub_CZ   
        results.CL    = h_sub(Mach) * sub_CL   
        results.CM    = h_sub(Mach) * sub_CM   
        results.CN    = h_sub(Mach) * sub_CN   
        
        return results
   
    
    # transonic   
    trans_Clift   = np.atleast_2d(trans_sur_Clift(pts)).T  
    trans_Cdrag   = np.atleast_2d(trans_sur_Cdrag(pts)).T  
    trans_CX      = np.atleast_2d(trans_sur_CX(pts)).T 
    trans_CY      = np.atleast_2d(trans_sur_CY(pts)).T     
    trans_CZ      = np.atleast_2d(trans_sur_CZ(pts)).T     
    trans_CL      = np.atleast_2d(trans_sur_CL(pts)).T     
    trans_CM      = np.atleast_2d(trans_sur_CM(pts)).T     
    trans_CN      = np.atleast_2d(trans_sur_CN(pts)).T

    # supersonic 
    sup_Clift     = np.atleast_2d(sup_sur_Clift(pts)).T  
    sup_Cdrag     = np.atleast_2d(sup_sur_Cdrag(pts)).T  
    sup_CX        = np.atleast_2d(sup_sur_CX(pts)).T 
    sup_CY        = np.atleast_2d(sup_sur_CY(pts)).T     
    sup_CZ        = np.atleast_2d(sup_sur_CZ(pts)).T     
    sup_CL        = np.atleast_2d(sup_sur_CL(pts)).T     
    sup_CM        = np.atleast_2d(sup_sur_CM(pts)).T     
    sup_CN        = np.atleast_2d(sup_sur_CN(pts)).T            

    # apply 
    results       = Data() 
    results.Clift = h_sub(Mach)*sub_Clift + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_Clift  + h_sup(Mach)*sup_Clift
    results.Cdrag = h_sub(Mach)*sub_Cdrag + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_Cdrag  + h_sup(Mach)*sup_Cdrag
    results.CX    = h_sub(Mach)*sub_CX    + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_CX     + h_sup(Mach)*sup_CX   
    results.CY    = h_sub(Mach)*sub_CY    + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_CY     + h_sup(Mach)*sup_CY   
    results.CZ    = h_sub(Mach)*sub_CZ    + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_CZ     + h_sup(Mach)*sup_CZ   
    results.CL    = h_sub(Mach)*sub_CL    + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_CL     + h_sup(Mach)*sup_CL   
    results.CM    = h_sub(Mach)*sub_CM    + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_CM     + h_sup(Mach)*sup_CM   
    results.CN    = h_sub(Mach)*sub_CN    + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_CN     + h_sup(Mach)*sup_CN

    return results


def compute_coefficient(sub_sur_coef,trans_sur_coef, sup_sur_coef, h_sub,h_sup,Mach, pts): 

    #  subsonic 
    sub_coef  = np.atleast_2d(sub_sur_coef(pts)).T     
   
    if trans_sur_coef == None and sup_sur_coef == None:
        coef = h_sub(Mach) 
        return  coef
    
    # transonic 
    trans_coef  = np.atleast_2d(trans_sur_coef(pts)).T    

    # supersonic 
    sup_coef  = np.atleast_2d(sub_sur_coef(pts)).T             

    # apply  
    coef = h_sub(Mach)*sub_coef +   (1 - (h_sup(Mach) + h_sub(Mach)))*trans_coef  + h_sub(Mach)*sup_coef 

    return coef