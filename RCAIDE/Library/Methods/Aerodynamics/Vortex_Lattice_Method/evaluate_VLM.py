# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/evaluate_VLM.py
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
import RCAIDE 
from RCAIDE.Framework.Core                                       import Units, Data, orientation_product 
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method   import VLM
from RCAIDE.Library.Methods.Utilities                            import Cubic_Spline_Blender  
from RCAIDE.Library.Mission.Common.Update  import orientations
from RCAIDE.Library.Mission.Common.Unpack_Unknowns import orientation

# package imports
import numpy   as np
from copy      import  deepcopy 
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ---------------------------------------------------------------------------------------------------------------------- 
def evaluate_surrogate(state,settings,vehicle):
    """Evaluates surrogates forces and moments using built surrogates 
    
    Assumptions:
        Aileron deflection is inverted to match convention 
        
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
     
    # loop through wings to determine what control surfaces are present
    delta_a       = np.zeros_like(Mach)
    delta_e       = np.zeros_like(Mach)
    delta_r       = np.zeros_like(Mach)
    delta_s       = np.zeros_like(Mach)
    delta_f       = np.zeros_like(Mach)
    
    for wing in vehicle.wings: 
        for control_surface in wing.control_surfaces:  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron:
                if trim ==  True: 
                    delta_a = np.atleast_2d(conditions.control_surfaces.aileron.deflection) 
                else:
                    delta_a = np.ones_like(Mach) * control_surface.deflection  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator: 
                if trim ==  True: 
                    delta_e = np.atleast_2d(conditions.control_surfaces.elevator.deflection)
                else:  
                    delta_e = np.ones_like(Mach) * control_surface.deflection
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder: 
                if trim ==  True: 
                    delta_r = np.atleast_2d(conditions.control_surfaces.rudder.deflection)
                else:  
                    delta_r = np.ones_like(Mach) * control_surface.deflection
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Slat:  
                delta_s  = np.ones_like(Mach) * control_surface.deflection
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap:   
                delta_f  = np.ones_like(Mach) * control_surface.deflection
 
    vehicle          = aerodynamics.vehicle 
    hsub_min         = aerodynamics.hsub_min
    hsub_max         = aerodynamics.hsub_max
    hsup_min         = aerodynamics.hsup_min
    hsup_max         = aerodynamics.hsup_max

    # Spline for Subsonic-to-Transonic-to-Supersonic Regimes
    sub_trans_spline = Cubic_Spline_Blender(hsub_min,hsub_max)
    h_sub            = lambda M:sub_trans_spline.compute(M)          
    sup_trans_spline = Cubic_Spline_Blender(hsup_max, hsup_min) 
    h_sup            = lambda M:sup_trans_spline.compute(M)
    
    u           = np.atleast_2d(conditions.freestream.u)
    v           = np.atleast_2d(conditions.freestream.v)
    w           = np.atleast_2d(conditions.freestream.w)
    p           = np.atleast_2d(conditions.static_stability.roll_rate)        
    q           = np.atleast_2d(conditions.static_stability.pitch_rate)
    r           = np.atleast_2d(conditions.static_stability.yaw_rate)   

    # -----------------------------------------------------------------------------------------------------------------------
    # Query surrogates  
    # ----------------------------------------------------------------------------------------------------------------------- 
    pts_alpha   = np.hstack((AoA,Mach))
    pts_beta    = np.hstack((Beta,Mach))  
    pts_u       = np.hstack((u,Mach))
    pts_v       = np.hstack((v,Mach))
    pts_w       = np.hstack((w,Mach))
    pts_p       = np.hstack((p,Mach))
    pts_q       = np.hstack((q,Mach))
    pts_r       = np.hstack((r,Mach))
    
    # Alpha 
    results_alpha = compute_coefficients(sub_sur.Clift_alpha,  sub_sur.Cdrag_alpha,  sub_sur.CX_alpha,  sub_sur.CY_alpha,  sub_sur.CZ_alpha,  sub_sur.CL_alpha,  sub_sur.CM_alpha,   sub_sur.CN_alpha,
                                         trans_sur.Clift_alpha,trans_sur.Cdrag_alpha,trans_sur.CX_alpha,trans_sur.CY_alpha,trans_sur.CZ_alpha,trans_sur.CL_alpha,trans_sur.CM_alpha, trans_sur.CN_alpha,
                                         sup_sur.Clift_alpha,  sup_sur.Cdrag_alpha,  sup_sur.CX_alpha,  sup_sur.CY_alpha,  sup_sur.CZ_alpha,  sup_sur.CL_alpha,  sup_sur.CM_alpha,   sup_sur.CN_alpha,
                                         h_sub,h_sup,Mach, pts_alpha)        

    Clift_alpha             = results_alpha.Clift   
    Cdrag_alpha             = results_alpha.Cdrag   
    CX_alpha                = results_alpha.CX      
    CY_alpha                = results_alpha.CY      
    CZ_alpha                = results_alpha.CZ      
    CL_alpha                = results_alpha.CL      
    CM_alpha                = results_alpha.CM      
    CN_alpha                = results_alpha.CN 
    Clift_alpha[AoA==0.0]   = 0   
    Cdrag_alpha[AoA==0.0]   = 0   
    CX_alpha[AoA==0.0]      = 0   
    CY_alpha[AoA==0.0]      = 0   
    CZ_alpha[AoA==0.0]      = 0   
    CL_alpha[AoA==0.0]      = 0   
    CM_alpha[AoA==0.0]      = 0   
    CN_alpha[AoA==0.0]      = 0  
    
    # Beta 
    results_beta  = compute_coefficients(sub_sur.Clift_beta,  sub_sur.Cdrag_beta,  sub_sur.CX_beta,  sub_sur.CY_beta,  sub_sur.CZ_beta,  sub_sur.CL_beta,  sub_sur.CM_beta,   sub_sur.CN_beta,
                                         trans_sur.Clift_beta,trans_sur.Cdrag_beta,trans_sur.CX_beta,trans_sur.CY_beta,trans_sur.CZ_beta,trans_sur.CL_beta,trans_sur.CM_beta, trans_sur.CN_beta,
                                         sup_sur.Clift_beta,  sup_sur.Cdrag_beta,  sup_sur.CX_beta,  sup_sur.CY_beta,  sup_sur.CZ_beta,  sup_sur.CL_beta,  sup_sur.CM_beta,   sup_sur.CN_beta,
                                         h_sub,h_sup,Mach, pts_beta)
     
    Clift_beta              = results_beta.Clift   
    Cdrag_beta              = results_beta.Cdrag 
    CX_beta                 = results_beta.CX     
    CY_beta                 = results_beta.CY     
    CZ_beta                 = results_beta.CZ     
    CL_beta                 = results_beta.CL     
    CM_beta                 = results_beta.CM     
    CN_beta                 = results_beta.CN 
    Clift_beta[Beta==0.0]   = 0   
    Cdrag_beta[Beta==0.0]   = 0
    CX_beta[Beta==0.0]      = 0 
    CY_beta[Beta==0.0]      = 0 
    CZ_beta[Beta==0.0]      = 0 
    CL_beta[Beta==0.0]      = 0 
    CM_beta[Beta==0.0]      = 0 
    CN_beta[Beta==0.0]      = 0

    # u  
    results_u     =  compute_coefficients(sub_sur.Clift_u,  sub_sur.Cdrag_u,  sub_sur.CX_u,  sub_sur.CY_u,  sub_sur.CZ_u,  sub_sur.CL_u,  sub_sur.CM_u,   sub_sur.CN_u,
                                          trans_sur.Clift_u,trans_sur.Cdrag_u,trans_sur.CX_u,trans_sur.CY_u,trans_sur.CZ_u,trans_sur.CL_u,trans_sur.CM_u, trans_sur.CN_u,
                                          sup_sur.Clift_u,  sup_sur.Cdrag_u,  sup_sur.CX_u,  sup_sur.CY_u,  sup_sur.CZ_u,  sup_sur.CL_u,  sup_sur.CM_u,   sup_sur.CN_u,
                                          h_sub,h_sup,Mach, pts_u)
                  
    Clift_u           = results_u.Clift   
    Cdrag_u           = results_u.Cdrag   
    CX_u              = results_u.CX      
    CY_u              = results_u.CY      
    CZ_u              = results_u.CZ      
    CL_u              = results_u.CL      
    CM_u              = results_u.CM      
    CN_u              = results_u.CN  
    Clift_u[u==0.0]   = 0   
    Cdrag_u[u==0.0]   = 0   
    CX_u[u==0.0]      = 0   
    CY_u[u==0.0]      = 0   
    CZ_u[u==0.0]      = 0   
    CL_u[u==0.0]      = 0   
    CM_u[u==0.0]      = 0   
    CN_u[u==0.0]      = 0  

    # v  
    results_v     =  compute_coefficients(sub_sur.Clift_v,  sub_sur.Cdrag_v,  sub_sur.CX_v,  sub_sur.CY_v,  sub_sur.CZ_v,  sub_sur.CL_v,  sub_sur.CM_v,   sub_sur.CN_v,
                                          trans_sur.Clift_v,trans_sur.Cdrag_v,trans_sur.CX_v,trans_sur.CY_v,trans_sur.CZ_v,trans_sur.CL_v,trans_sur.CM_v, trans_sur.CN_v,
                                          sup_sur.Clift_v,  sup_sur.Cdrag_v,  sup_sur.CX_v,  sup_sur.CY_v,  sup_sur.CZ_v,  sup_sur.CL_v,  sup_sur.CM_v,   sup_sur.CN_v,
                                          h_sub,h_sup,Mach, pts_v)
     
    Clift_v           = results_v.Clift   
    Cdrag_v           = results_v.Cdrag   
    CX_v              = results_v.CX      
    CY_v              = results_v.CY      
    CZ_v              = results_v.CZ      
    CL_v              = results_v.CL      
    CM_v              = results_v.CM      
    CN_v              = results_v.CN   
    Clift_v[v==0.0]   = 0
    Cdrag_v[v==0.0]   = 0
    CX_v[v==0.0]      = 0
    CY_v[v==0.0]      = 0
    CZ_v[v==0.0]      = 0
    CL_v[v==0.0]      = 0
    CM_v[v==0.0]      = 0
    CN_v[v==0.0]      = 0

    # w  
    results_w    =  compute_coefficients(sub_sur.Clift_w,  sub_sur.Cdrag_w,  sub_sur.CX_w,  sub_sur.CY_w,  sub_sur.CZ_w,  sub_sur.CL_w,  sub_sur.CM_w,   sub_sur.CN_w,
                                         trans_sur.Clift_w,trans_sur.Cdrag_w,trans_sur.CX_w,trans_sur.CY_w,trans_sur.CZ_w,trans_sur.CL_w,trans_sur.CM_w, trans_sur.CN_w,
                                         sup_sur.Clift_w,  sup_sur.Cdrag_w,  sup_sur.CX_w,  sup_sur.CY_w,  sup_sur.CZ_w,  sup_sur.CL_w,  sup_sur.CM_w,   sup_sur.CN_w,
                                         h_sub,h_sup,Mach, pts_w)
     
    Clift_w           = results_w.Clift   
    Cdrag_w           = results_w.Cdrag   
    CX_w              = results_w.CX      
    CY_w              = results_w.CY      
    CZ_w              = results_w.CZ      
    CL_w              = results_w.CL      
    CM_w              = results_w.CM      
    CN_w              = results_w.CN
    Clift_w[w==0.0]   = 0
    Cdrag_w[w==0.0]   = 0
    CX_w[w==0.0]      = 0
    CY_w[w==0.0]      = 0
    CZ_w[w==0.0]      = 0
    CL_w[w==0.0]      = 0
    CM_w[w==0.0]      = 0
    CN_w[w==0.0]      = 0
                        
    # p  
    results_p    =  compute_coefficients(sub_sur.Clift_p,  sub_sur.Cdrag_p,  sub_sur.CX_p,  sub_sur.CY_p,  sub_sur.CZ_p,  sub_sur.CL_p,  sub_sur.CM_p,   sub_sur.CN_p,
                                         trans_sur.Clift_p,trans_sur.Cdrag_p,trans_sur.CX_p,trans_sur.CY_p,trans_sur.CZ_p,trans_sur.CL_p,trans_sur.CM_p, trans_sur.CN_p,
                                         sup_sur.Clift_p,  sup_sur.Cdrag_p,  sup_sur.CX_p,  sup_sur.CY_p,  sup_sur.CZ_p,  sup_sur.CL_p,  sup_sur.CM_p,   sup_sur.CN_p,
                                         h_sub,h_sup,Mach, pts_p)
     
    Clift_p           = results_p.Clift   
    Cdrag_p           = results_p.Cdrag   
    CX_p              = results_p.CX      
    CY_p              = results_p.CY      
    CZ_p              = results_p.CZ      
    CL_p              = results_p.CL      
    CM_p              = results_p.CM      
    CN_p              = results_p.CN
    Clift_p[p==0.0]   = 0   
    Cdrag_p[p==0.0]   = 0   
    CX_p[p==0.0]      = 0   
    CY_p[p==0.0]      = 0   
    CZ_p[p==0.0]      = 0   
    CL_p[p==0.0]      = 0   
    CM_p[p==0.0]      = 0   
    CN_p[p==0.0]      = 0 
     
    # q  
    results_q    =  compute_coefficients(sub_sur.Clift_q,  sub_sur.Cdrag_q,  sub_sur.CX_q,  sub_sur.CY_q,  sub_sur.CZ_q,  sub_sur.CL_q,  sub_sur.CM_q,   sub_sur.CN_q,
                                         trans_sur.Clift_q,trans_sur.Cdrag_q,trans_sur.CX_q,trans_sur.CY_q,trans_sur.CZ_q,trans_sur.CL_q,trans_sur.CM_q, trans_sur.CN_q,
                                         sup_sur.Clift_q,  sup_sur.Cdrag_q,  sup_sur.CX_q,  sup_sur.CY_q,  sup_sur.CZ_q,  sup_sur.CL_q,  sup_sur.CM_q,   sup_sur.CN_q,
                                         h_sub,h_sup,Mach, pts_q)
     
    Clift_q           = results_q.Clift   
    Cdrag_q           = results_q.Cdrag   
    CX_q              = results_q.CX      
    CY_q              = results_q.CY      
    CZ_q              = results_q.CZ      
    CL_q              = results_q.CL      
    CM_q              = results_q.CM      
    CN_q              = results_q.CN
    Clift_q[q==0.0]   = 0
    Cdrag_q[q==0.0]   = 0
    CX_q[q==0.0]      = 0
    CY_q[q==0.0]      = 0
    CZ_q[q==0.0]      = 0
    CL_q[q==0.0]      = 0
    CM_q[q==0.0]      = 0
    CN_q[q==0.0]      = 0
    
    # r  
    results_r    =  compute_coefficients(sub_sur.Clift_r,  sub_sur.Cdrag_r,  sub_sur.CX_r,  sub_sur.CY_r,  sub_sur.CZ_r,  sub_sur.CL_r,  sub_sur.CM_r,   sub_sur.CN_r,
                                         trans_sur.Clift_r,trans_sur.Cdrag_r,trans_sur.CX_r,trans_sur.CY_r,trans_sur.CZ_r,trans_sur.CL_r,trans_sur.CM_r, trans_sur.CN_r,
                                         sup_sur.Clift_r,  sup_sur.Cdrag_r,  sup_sur.CX_r,  sup_sur.CY_r,  sup_sur.CZ_r,  sup_sur.CL_r,  sup_sur.CM_r,   sup_sur.CN_r,
                                         h_sub,h_sup,Mach, pts_r)
     
    Clift_r           = results_r.Clift   
    Cdrag_r           = results_r.Cdrag   
    CX_r              = results_r.CX      
    CY_r              = results_r.CY      
    CZ_r              = results_r.CZ      
    CL_r              = results_r.CL      
    CM_r              = results_r.CM      
    CN_r              = results_r.CN 
    Clift_r[r==0.0]   = 0  
    Cdrag_r[r==0.0]   = 0  
    CX_r[r==0.0]      = 0  
    CY_r[r==0.0]      = 0  
    CZ_r[r==0.0]      = 0  
    CL_r[r==0.0]      = 0  
    CM_r[r==0.0]      = 0  
    CN_r[r==0.0]      = 0 
    
    # -----------------------------------------------------------------------------------------------------------------------
    # Stability Results Without Control Surfaces 
    # -----------------------------------------------------------------------------------------------------------------------
    conditions.S_ref  = ref_vals.S_ref              
    conditions.c_ref  = ref_vals.c_ref              
    conditions.b_ref  = ref_vals.b_ref
    conditions.X_ref  = ref_vals.X_ref
    conditions.Y_ref  = ref_vals.Y_ref
    conditions.Z_ref  = ref_vals.Z_ref 
     
    conditions.static_stability.coefficients.lift   = Clift_alpha + Clift_u + Clift_w + Clift_q 
    conditions.static_stability.coefficients.drag   = Cdrag_alpha + Cdrag_u + Cdrag_w + Cdrag_q 
    conditions.static_stability.coefficients.X      = CX_alpha + CX_u + CX_w + CX_q
    conditions.static_stability.coefficients.Y      = CY_beta + CY_v + CY_p + CY_r
    conditions.static_stability.coefficients.Z      = CZ_alpha + CZ_u + CZ_w + CZ_q 
    conditions.static_stability.coefficients.L      = CL_beta + CL_v + CL_p +CL_r
    conditions.static_stability.coefficients.M      = CM_alpha + CM_u + CM_w + CM_q 
    conditions.static_stability.coefficients.N      = CN_beta + CN_v + CN_p + CN_r    
    

    # -----------------------------------------------------------------------------------------------------------------------
    # Addition of Control Surface Effect 
    # -----------------------------------------------------------------------------------------------------------------------    
    if aerodynamics.aileron_flag: 
        pts_delta_a     = np.hstack((delta_a,Mach))
        
        results_delta_a =  compute_coefficients(sub_sur.Clift_delta_a,sub_sur.Cdrag_delta_a,sub_sur.CX_delta_a,sub_sur.CY_delta_a,sub_sur.CZ_delta_a,sub_sur.CL_delta_a,sub_sur.CM_delta_a, sub_sur.CN_delta_a,
         trans_sur.Clift_delta_a,trans_sur.Cdrag_delta_a,trans_sur.CX_delta_a,trans_sur.CY_delta_a,trans_sur.CZ_delta_a,trans_sur.CL_delta_a,trans_sur.CM_delta_a, trans_sur.CN_delta_a,
         sup_sur.Clift_delta_a,sup_sur.Cdrag_delta_a,sup_sur.CX_delta_a,sup_sur.CY_delta_a,sup_sur.CZ_delta_a,sup_sur.CL_delta_a,sup_sur.CM_delta_a, sup_sur.CN_delta_a,
        h_sub,h_sup,Mach, pts_delta_a)
         
        Clift_delta_a   = results_delta_a.Clift   
        Cdrag_delta_a   = results_delta_a.Cdrag   
        CX_delta_a      = results_delta_a.CX      
        CY_delta_a      = results_delta_a.CY      
        CZ_delta_a      = results_delta_a.CZ      
        CL_delta_a      = results_delta_a.CL      
        CM_delta_a      = results_delta_a.CM      
        CN_delta_a      = results_delta_a.CN     
        Clift_delta_a[delta_a==0.0] = 0   
        Cdrag_delta_a[delta_a==0.0] = 0   
        CX_delta_a[delta_a==0.0]    = 0        
        CY_delta_a[delta_a==0.0]    = 0        
        CZ_delta_a[delta_a==0.0]    = 0        
        CL_delta_a[delta_a==0.0]    = 0        
        CM_delta_a[delta_a==0.0]    = 0        
        CN_delta_a[delta_a==0.0]    = 0
        
        conditions.static_stability.coefficients.lift                                += Clift_delta_a
        conditions.static_stability.coefficients.drag                                += Cdrag_delta_a
        conditions.static_stability.coefficients.X                                   += CX_delta_a 
        conditions.static_stability.coefficients.Y                                   += CY_delta_a
        conditions.static_stability.coefficients.Z                                   += CZ_delta_a
        conditions.static_stability.coefficients.L                                   += CL_delta_a
        conditions.static_stability.coefficients.M                                   += CM_delta_a
        conditions.static_stability.coefficients.N                                   += CN_delta_a

        conditions.control_surfaces.aileron.static_stability.coefficients.lift       = Clift_delta_a         
        conditions.control_surfaces.aileron.static_stability.coefficients.drag       = Cdrag_delta_a            
        conditions.control_surfaces.aileron.static_stability.coefficients.X          = CX_delta_a             
        conditions.control_surfaces.aileron.static_stability.coefficients.Y          = CY_delta_a            
        conditions.control_surfaces.aileron.static_stability.coefficients.Z          = CZ_delta_a          
        conditions.control_surfaces.aileron.static_stability.coefficients.L          = CL_delta_a          
        conditions.control_surfaces.aileron.static_stability.coefficients.M          = CM_delta_a          
        conditions.control_surfaces.aileron.static_stability.coefficients.N          = CN_delta_a             
        
    if aerodynamics.elevator_flag: 
        pts_delta_e     = np.hstack((delta_e,Mach))

        results_delta_e =  compute_coefficients(sub_sur.Clift_delta_e,sub_sur.Cdrag_delta_e,sub_sur.CX_delta_e,sub_sur.CY_delta_e,sub_sur.CZ_delta_e,sub_sur.CL_delta_e,sub_sur.CM_delta_e, sub_sur.CN_delta_e,
         trans_sur.Clift_delta_e,trans_sur.Cdrag_delta_e,trans_sur.CX_delta_e,trans_sur.CY_delta_e,trans_sur.CZ_delta_e,trans_sur.CL_delta_e,trans_sur.CM_delta_e, trans_sur.CN_delta_e,
         sup_sur.Clift_delta_e,sup_sur.Cdrag_delta_e,sup_sur.CX_delta_e,sup_sur.CY_delta_e,sup_sur.CZ_delta_e,sup_sur.CL_delta_e,sup_sur.CM_delta_e, sup_sur.CN_delta_e,
        h_sub,h_sup,Mach, pts_delta_e)
         
        Clift_delta_e   = results_delta_e.Clift   
        Cdrag_delta_e   = results_delta_e.Cdrag   
        CX_delta_e      = results_delta_e.CX      
        CY_delta_e      = results_delta_e.CY      
        CZ_delta_e      = results_delta_e.CZ      
        CL_delta_e      = results_delta_e.CL      
        CM_delta_e      = results_delta_e.CM      
        CN_delta_e      = results_delta_e.CN
        Clift_delta_e[delta_e==0.0] = 0     
        Cdrag_delta_e[delta_e==0.0] = 0    
        CX_delta_e[delta_e==0.0]    = 0          
        CY_delta_e[delta_e==0.0]    = 0          
        CZ_delta_e[delta_e==0.0]    = 0          
        CL_delta_e[delta_e==0.0]    = 0          
        CM_delta_e[delta_e==0.0]    = 0          
        CN_delta_e[delta_e==0.0]    = 0
        
        conditions.static_stability.coefficients.lift                                += Clift_delta_e
        conditions.static_stability.coefficients.drag                                += Cdrag_delta_e
        conditions.static_stability.coefficients.X                                   += CX_delta_e 
        conditions.static_stability.coefficients.Y                                   += CY_delta_e
        conditions.static_stability.coefficients.Z                                   += CZ_delta_e
        conditions.static_stability.coefficients.L                                   += CL_delta_e
        conditions.static_stability.coefficients.M                                   += CM_delta_e
        conditions.static_stability.coefficients.N                                   += CN_delta_e
        
        conditions.control_surfaces.elevator.static_stability.coefficients.lift      = Clift_delta_e         
        conditions.control_surfaces.elevator.static_stability.coefficients.drag      = Cdrag_delta_e            
        conditions.control_surfaces.elevator.static_stability.coefficients.X         = CX_delta_e             
        conditions.control_surfaces.elevator.static_stability.coefficients.Y         = CY_delta_e            
        conditions.control_surfaces.elevator.static_stability.coefficients.Z         = CZ_delta_e          
        conditions.control_surfaces.elevator.static_stability.coefficients.L         = CL_delta_e          
        conditions.control_surfaces.elevator.static_stability.coefficients.M         = CM_delta_e          
        conditions.control_surfaces.elevator.static_stability.coefficients.N         = CN_delta_e            
        
    if aerodynamics.rudder_flag:  
        pts_delta_r    = np.hstack((delta_r,Mach))
        
        results_delta_r =  compute_coefficients(sub_sur.Clift_delta_r,sub_sur.Cdrag_delta_r,sub_sur.CX_delta_r,sub_sur.CY_delta_r,sub_sur.CZ_delta_r,sub_sur.CL_delta_r,sub_sur.CM_delta_r, sub_sur.CN_delta_r,
         trans_sur.Clift_delta_r,trans_sur.Cdrag_delta_r,trans_sur.CX_delta_r,trans_sur.CY_delta_r,trans_sur.CZ_delta_r,trans_sur.CL_delta_r,trans_sur.CM_delta_r, trans_sur.CN_delta_r,
         sup_sur.Clift_delta_r,sup_sur.Cdrag_delta_r,sup_sur.CX_delta_r,sup_sur.CY_delta_r,sup_sur.CZ_delta_r,sup_sur.CL_delta_r,sup_sur.CM_delta_r, sup_sur.CN_delta_r,
        h_sub,h_sup,Mach, pts_delta_r)
         
        Clift_delta_r   = results_delta_r.Clift   
        Cdrag_delta_r   = results_delta_r.Cdrag   
        CX_delta_r      = results_delta_r.CX      
        CY_delta_r      = results_delta_r.CY      
        CZ_delta_r      = results_delta_r.CZ      
        CL_delta_r      = results_delta_r.CL      
        CM_delta_r      = results_delta_r.CM      
        CN_delta_r      = results_delta_r.CN 
        Clift_delta_r[delta_r==0.0] = 0    
        Cdrag_delta_r[delta_r==0.0] = 0    
        CX_delta_r[delta_r==0.0]    = 0          
        CY_delta_r[delta_r==0.0]    = 0          
        CZ_delta_r[delta_r==0.0]    = 0          
        CL_delta_r[delta_r==0.0]    = 0          
        CM_delta_r[delta_r==0.0]    = 0          
        CN_delta_r[delta_r==0.0]    = 0
        
        conditions.static_stability.coefficients.lift                                += Clift_delta_r
        conditions.static_stability.coefficients.drag                                += Cdrag_delta_r
        conditions.static_stability.coefficients.X                                   += CX_delta_r 
        conditions.static_stability.coefficients.Y                                   += CY_delta_r
        conditions.static_stability.coefficients.Z                                   += CZ_delta_r
        conditions.static_stability.coefficients.L                                   += CL_delta_r
        conditions.static_stability.coefficients.M                                   += CM_delta_r
        conditions.static_stability.coefficients.N                                   += CN_delta_r

        conditions.control_surfaces.rudder.static_stability.coefficients.lift        = Clift_delta_r         
        conditions.control_surfaces.rudder.static_stability.coefficients.drag        = Cdrag_delta_r            
        conditions.control_surfaces.rudder.static_stability.coefficients.X           = CX_delta_r             
        conditions.control_surfaces.rudder.static_stability.coefficients.Y           = CY_delta_r            
        conditions.control_surfaces.rudder.static_stability.coefficients.Z           = CZ_delta_r          
        conditions.control_surfaces.rudder.static_stability.coefficients.L           = CL_delta_r          
        conditions.control_surfaces.rudder.static_stability.coefficients.M           = CM_delta_r          
        conditions.control_surfaces.rudder.static_stability.coefficients.N           = CN_delta_r         
        
    if aerodynamics.flap_flag:
        pts_delta_f    = np.hstack((delta_f,Mach))
        
        results_delta_f =  compute_coefficients(sub_sur.Clift_delta_f,sub_sur.Cdrag_delta_f,sub_sur.CX_delta_f,sub_sur.CY_delta_f,sub_sur.CZ_delta_f,sub_sur.CL_delta_f,sub_sur.CM_delta_f, sub_sur.CN_delta_f,
         trans_sur.Clift_delta_f,trans_sur.Cdrag_delta_f,trans_sur.CX_delta_f,trans_sur.CY_delta_f,trans_sur.CZ_delta_f,trans_sur.CL_delta_f,trans_sur.CM_delta_f, trans_sur.CN_delta_f,
         sup_sur.Clift_delta_f,sup_sur.Cdrag_delta_f,sup_sur.CX_delta_f,sup_sur.CY_delta_f,sup_sur.CZ_delta_f,sup_sur.CL_delta_f,sup_sur.CM_delta_f, sup_sur.CN_delta_f,
        h_sub,h_sup,Mach, pts_delta_f)
         
        Clift_delta_f   = results_delta_f.Clift   
        Cdrag_delta_f   = results_delta_f.Cdrag   
        CX_delta_f      = results_delta_f.CX      
        CY_delta_f      = results_delta_f.CY      
        CZ_delta_f      = results_delta_f.CZ      
        CL_delta_f      = results_delta_f.CL      
        CM_delta_f      = results_delta_f.CM      
        CN_delta_f      = results_delta_f.CN
        Clift_delta_f[delta_f==0.0] = 0   
        Cdrag_delta_f[delta_f==0.0] = 0  
        CX_delta_f[delta_f==0.0]    = 0        
        CY_delta_f[delta_f==0.0]    = 0        
        CZ_delta_f[delta_f==0.0]    = 0        
        CL_delta_f[delta_f==0.0]    = 0        
        CM_delta_f[delta_f==0.0]    = 0        
        CN_delta_f[delta_f==0.0]    = 0
        
        conditions.static_stability.coefficients.lift                                += Clift_delta_f
        conditions.static_stability.coefficients.drag                                += Cdrag_delta_f
        conditions.static_stability.coefficients.X                                   += CX_delta_f 
        conditions.static_stability.coefficients.Y                                   += CY_delta_f
        conditions.static_stability.coefficients.Z                                   += CZ_delta_f
        conditions.static_stability.coefficients.L                                   += CL_delta_f
        conditions.static_stability.coefficients.M                                   += CM_delta_f
        conditions.static_stability.coefficients.N                                   += CN_delta_f
        
        conditions.control_surfaces.flap.static_stability.coefficients.lift          = Clift_delta_f         
        conditions.control_surfaces.flap.static_stability.coefficients.drag          = Cdrag_delta_f            
        conditions.control_surfaces.flap.static_stability.coefficients.X             = CX_delta_f             
        conditions.control_surfaces.flap.static_stability.coefficients.Y             = CY_delta_f            
        conditions.control_surfaces.flap.static_stability.coefficients.Z             = CZ_delta_f          
        conditions.control_surfaces.flap.static_stability.coefficients.L             = CL_delta_f          
        conditions.control_surfaces.flap.static_stability.coefficients.M             = CM_delta_f          
        conditions.control_surfaces.flap.static_stability.coefficients.N             = CN_delta_f           
        
    if aerodynamics.slat_flag: 
    
        pts_delta_s    = np.hstack((delta_s,Mach)) 
        
        results_delta_s =  compute_coefficients(sub_sur.Clift_delta_s,sub_sur.Cdrag_delta_s,sub_sur.CX_delta_s,sub_sur.CY_delta_s,sub_sur.CZ_delta_s,sub_sur.CL_delta_s,sub_sur.CM_delta_s, sub_sur.CN_delta_s,
         trans_sur.Clift_delta_s,trans_sur.Cdrag_delta_s,trans_sur.CX_delta_s,trans_sur.CY_delta_s,trans_sur.CZ_delta_s,trans_sur.CL_delta_s,trans_sur.CM_delta_s, trans_sur.CN_delta_s,
         sup_sur.Clift_delta_s,sup_sur.Cdrag_delta_s,sup_sur.CX_delta_s,sup_sur.CY_delta_s,sup_sur.CZ_delta_s,sup_sur.CL_delta_s,sup_sur.CM_delta_s, sup_sur.CN_delta_s,
        h_sub,h_sup,Mach, pts_delta_s)
         
        Clift_delta_s   = results_delta_s.Clift   
        Cdrag_delta_s   = results_delta_s.Cdrag   
        CX_delta_s      = results_delta_s.CX      
        CY_delta_s      = results_delta_s.CY      
        CZ_delta_s      = results_delta_s.CZ      
        CL_delta_s      = results_delta_s.CL      
        CM_delta_s      = results_delta_s.CM      
        CN_delta_s      = results_delta_s.CN
        Clift_delta_s[delta_s==0.0] = 0    
        Cdrag_delta_s[delta_s==0.0] = 0  
        CX_delta_s[delta_s==0.0]    = 0        
        CY_delta_s[delta_s==0.0]    = 0        
        CZ_delta_s[delta_s==0.0]    = 0        
        CL_delta_s[delta_s==0.0]    = 0        
        CM_delta_s[delta_s==0.0]    = 0        
        CN_delta_s[delta_s==0.0]    = 0          
         
        conditions.static_stability.coefficients.lift                                += Clift_delta_s
        conditions.static_stability.coefficients.drag                                += Cdrag_delta_s
        conditions.static_stability.coefficients.X                                   += CX_delta_s 
        conditions.static_stability.coefficients.Y                                   += CY_delta_s
        conditions.static_stability.coefficients.Z                                   += CZ_delta_s
        conditions.static_stability.coefficients.L                                   += CL_delta_s
        conditions.static_stability.coefficients.M                                   += CM_delta_s
        conditions.static_stability.coefficients.N                                   += CN_delta_s

        conditions.control_surfaces.slat.static_stability.coefficients.lift          = Clift_delta_s         
        conditions.control_surfaces.slat.static_stability.coefficients.drag          = Cdrag_delta_s            
        conditions.control_surfaces.slat.static_stability.coefficients.X             = CX_delta_s             
        conditions.control_surfaces.slat.static_stability.coefficients.Y             = CY_delta_s            
        conditions.control_surfaces.slat.static_stability.coefficients.Z             = CZ_delta_s          
        conditions.control_surfaces.slat.static_stability.coefficients.L             = CL_delta_s          
        conditions.control_surfaces.slat.static_stability.coefficients.M             = CM_delta_s          
        conditions.control_surfaces.slat.static_stability.coefficients.N             = CN_delta_s                     
     
    
    conditions.static_stability.derivatives.Clift_alpha = compute_stability_derivative(sub_sur.dClift_dalpha ,trans_sur.dClift_dalpha ,sup_sur.dClift_dalpha ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CX_alpha    = compute_stability_derivative(sub_sur.dCX_dalpha    ,trans_sur.dCX_dalpha    ,sup_sur.dCX_dalpha    ,h_sub,h_sup,Mach)  
    conditions.static_stability.derivatives.CY_alpha    = compute_stability_derivative(sub_sur.dCY_dalpha    ,trans_sur.dCY_dalpha    ,sup_sur.dCY_dalpha    ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CZ_alpha    = compute_stability_derivative(sub_sur.dCZ_dalpha    ,trans_sur.dCZ_dalpha    ,sup_sur.dCZ_dalpha    ,h_sub,h_sup,Mach) 
    conditions.static_stability.derivatives.CL_alpha    = compute_stability_derivative(sub_sur.dCL_dalpha    ,trans_sur.dCL_dalpha    ,sup_sur.dCL_dalpha    ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CM_alpha    = compute_stability_derivative(sub_sur.dCM_dalpha    ,trans_sur.dCM_dalpha    ,sup_sur.dCM_dalpha    ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CN_alpha    = compute_stability_derivative(sub_sur.dCN_dalpha    ,trans_sur.dCN_dalpha    ,sup_sur.dCN_dalpha    ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.Clift_beta  = compute_stability_derivative(sub_sur.dClift_dbeta  ,trans_sur.dClift_dbeta  ,sup_sur.dClift_dbeta  ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CX_beta     = compute_stability_derivative(sub_sur.dCX_dbeta     ,trans_sur.dCX_dbeta     ,sup_sur.dCX_dbeta     ,h_sub,h_sup,Mach)  
    conditions.static_stability.derivatives.CY_beta     = compute_stability_derivative(sub_sur.dCY_dbeta     ,trans_sur.dCY_dbeta     ,sup_sur.dCY_dbeta     ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CZ_beta     = compute_stability_derivative(sub_sur.dCZ_dbeta     ,trans_sur.dCZ_dbeta     ,sup_sur.dCZ_dbeta     ,h_sub,h_sup,Mach) 
    conditions.static_stability.derivatives.CL_beta     = compute_stability_derivative(sub_sur.dCL_dbeta     ,trans_sur.dCL_dbeta     ,sup_sur.dCL_dbeta     ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CM_beta     = compute_stability_derivative(sub_sur.dCM_dbeta     ,trans_sur.dCM_dbeta     ,sup_sur.dCM_dbeta     ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CN_beta     = compute_stability_derivative(sub_sur.dCN_dbeta     ,trans_sur.dCN_dbeta     ,sup_sur.dCN_dbeta     ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.Clift_p     = compute_stability_derivative(sub_sur.dClift_dp     ,trans_sur.dClift_dp     ,sup_sur.dClift_dp     ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.Clift_q     = compute_stability_derivative(sub_sur.dClift_dq     ,trans_sur.dClift_dq     ,sup_sur.dClift_dq     ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.Clift_r     = compute_stability_derivative(sub_sur.dClift_dr     ,trans_sur.dClift_dr     ,sup_sur.dClift_dr     ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CX_u        = compute_stability_derivative(sub_sur.dCX_du        ,trans_sur.dCX_du        ,sup_sur.dCX_du        ,h_sub,h_sup,Mach)   
    conditions.static_stability.derivatives.CX_v        = compute_stability_derivative(sub_sur.dCX_dv        ,trans_sur.dCX_dv        ,sup_sur.dCX_dv        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CX_w        = compute_stability_derivative(sub_sur.dCX_dw        ,trans_sur.dCX_dw        ,sup_sur.dCX_dw        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CY_u        = compute_stability_derivative(sub_sur.dCY_du        ,trans_sur.dCY_du        ,sup_sur.dCY_du        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CY_v        = compute_stability_derivative(sub_sur.dCY_dv        ,trans_sur.dCY_dv        ,sup_sur.dCY_dv        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CY_w        = compute_stability_derivative(sub_sur.dCY_dw        ,trans_sur.dCY_dw        ,sup_sur.dCY_dw        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CZ_u        = compute_stability_derivative(sub_sur.dCZ_du        ,trans_sur.dCZ_du        ,sup_sur.dCZ_du        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CZ_v        = compute_stability_derivative(sub_sur.dCZ_dv        ,trans_sur.dCZ_dv        ,sup_sur.dCZ_dv        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CZ_w        = compute_stability_derivative(sub_sur.dCZ_dw        ,trans_sur.dCZ_dw        ,sup_sur.dCZ_dw        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CL_u        = compute_stability_derivative(sub_sur.dCL_du        ,trans_sur.dCL_du        ,sup_sur.dCL_du        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CL_v        = compute_stability_derivative(sub_sur.dCL_dv        ,trans_sur.dCL_dv        ,sup_sur.dCL_dv        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CL_w        = compute_stability_derivative(sub_sur.dCL_dw        ,trans_sur.dCL_dw        ,sup_sur.dCL_dw        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CM_u        = compute_stability_derivative(sub_sur.dCM_du        ,trans_sur.dCM_du        ,sup_sur.dCM_du        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CM_v        = compute_stability_derivative(sub_sur.dCM_dv        ,trans_sur.dCM_dv        ,sup_sur.dCM_dv        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CM_w        = compute_stability_derivative(sub_sur.dCM_dw        ,trans_sur.dCM_dw        ,sup_sur.dCM_dw        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CN_u        = compute_stability_derivative(sub_sur.dCN_du        ,trans_sur.dCN_du        ,sup_sur.dCN_du        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CN_v        = compute_stability_derivative(sub_sur.dCN_dv        ,trans_sur.dCN_dv        ,sup_sur.dCN_dv        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CN_w        = compute_stability_derivative(sub_sur.dCN_dw        ,trans_sur.dCN_dw        ,sup_sur.dCN_dw        ,h_sub,h_sup,Mach) 
    conditions.static_stability.derivatives.CX_p        = compute_stability_derivative(sub_sur.dCX_dp        ,trans_sur.dCX_dp        ,sup_sur.dCX_dp        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CX_q        = compute_stability_derivative(sub_sur.dCX_dq        ,trans_sur.dCX_dq        ,sup_sur.dCX_dq        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CX_r        = compute_stability_derivative(sub_sur.dCX_dr        ,trans_sur.dCX_dr        ,sup_sur.dCX_dr        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CY_p        = compute_stability_derivative(sub_sur.dCY_dp        ,trans_sur.dCY_dp        ,sup_sur.dCY_dp        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CY_q        = compute_stability_derivative(sub_sur.dCY_dq        ,trans_sur.dCY_dq        ,sup_sur.dCY_dq        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CY_r        = compute_stability_derivative(sub_sur.dCY_dr        ,trans_sur.dCY_dr        ,sup_sur.dCY_dr        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CZ_p        = compute_stability_derivative(sub_sur.dCZ_dp        ,trans_sur.dCZ_dp        ,sup_sur.dCZ_dp        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CZ_q        = compute_stability_derivative(sub_sur.dCZ_dq        ,trans_sur.dCZ_dq        ,sup_sur.dCZ_dq        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CZ_r        = compute_stability_derivative(sub_sur.dCZ_dr        ,trans_sur.dCZ_dr        ,sup_sur.dCZ_dr        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CL_p        = compute_stability_derivative(sub_sur.dCL_dp        ,trans_sur.dCL_dp        ,sup_sur.dCL_dp        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CL_q        = compute_stability_derivative(sub_sur.dCL_dq        ,trans_sur.dCL_dq        ,sup_sur.dCL_dq        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CL_r        = compute_stability_derivative(sub_sur.dCL_dr        ,trans_sur.dCL_dr        ,sup_sur.dCL_dr        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CM_p        = compute_stability_derivative(sub_sur.dCM_dp        ,trans_sur.dCM_dp        ,sup_sur.dCM_dp        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CM_q        = compute_stability_derivative(sub_sur.dCM_dq        ,trans_sur.dCM_dq        ,sup_sur.dCM_dq        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CM_r        = compute_stability_derivative(sub_sur.dCM_dr        ,trans_sur.dCM_dr        ,sup_sur.dCM_dr        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CN_p        = compute_stability_derivative(sub_sur.dCN_dp        ,trans_sur.dCN_dp        ,sup_sur.dCN_dp        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CN_q        = compute_stability_derivative(sub_sur.dCN_dq        ,trans_sur.dCN_dq        ,sup_sur.dCN_dq        ,h_sub,h_sup,Mach)
    conditions.static_stability.derivatives.CN_r        = compute_stability_derivative(sub_sur.dCN_dr        ,trans_sur.dCN_dr        ,sup_sur.dCN_dr        ,h_sub,h_sup,Mach) 

    for wing in vehicle.wings:   
        inviscid_wing_lifts = compute_coefficient(sub_sur.Clift_wing_alpha[wing.tag],trans_sur.Clift_wing_alpha[wing.tag],sup_sur.Cdrag_wing_alpha[wing.tag] ,h_sub,h_sup,Mach,pts_alpha)
        inviscid_wing_drags = compute_coefficient(sub_sur.Cdrag_wing_alpha[wing.tag],trans_sur.Cdrag_wing_alpha[wing.tag],sup_sur.Cdrag_wing_alpha[wing.tag] ,h_sub,h_sup,Mach,pts_alpha)
        # Pack 
        conditions.aerodynamics.coefficients.lift.induced.inviscid_wings[wing.tag]         =  inviscid_wing_lifts 
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
        
        Clift,Cdrag,CX,CY,CZ,CL,CM,CN,S_ref,b_ref,c_ref,X_ref,Y_ref ,Z_ref,Clift_wings,Cdrag_wings,AoA_wing_induced, Clift_spanwise,Cdrag_induced_spanwise,pressure_coefficient = call_VLM(conditions,settings,vehicle)
        
        # Dimensionalize the lift and drag for each wing 
        for wing in vehicle.wings: 
            conditions.aerodynamics.coefficients.lift.induced.inviscid_wings[wing.tag] = Clift_wings[wing.tag]
            conditions.aerodynamics.coefficients.lift.compressible_wings[wing.tag]     = Clift_wings[wing.tag]
            conditions.aerodynamics.coefficients.drag.induced.inviscid_wings[wing.tag] = Cdrag_wings[wing.tag] 
        conditions.aerodynamics.coefficients.lift.induced.spanwise     = Clift_spanwise
        conditions.aerodynamics.coefficients.drag.induced.spanwise     = Cdrag_induced_spanwise
        conditions.aerodynamics.coefficients.surface_pressure          = pressure_coefficient 
        conditions.aerodynamics.coefficients.lift.total                = Clift
        conditions.aerodynamics.coefficients.drag.induced.inviscid     = Cdrag
        
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
    
    Clift_0,Cdrag_0,CX_0,CY_0,CZ_0,CL_0,CM_0,CN_0,_,_,_,_,_ ,_,Clift_0_wings,Cdrag_0_wings,_,_,_, _= call_VLM(equilibrium_conditions,settings,vehicle)
    
    # Dimensionalize the lift and drag for each wing 
    for wing in vehicle.wings: 
        equilibrium_conditions.aerodynamics.coefficients.lift.induced.inviscid_wings[wing.tag] = Clift_0_wings[wing.tag]
        equilibrium_conditions.aerodynamics.coefficients.lift.compressible_wings[wing.tag]     = Clift_0_wings[wing.tag]
        equilibrium_conditions.aerodynamics.coefficients.drag.induced.inviscid_wings[wing.tag] = Cdrag_0_wings[wing.tag] 
    equilibrium_conditions.aerodynamics.coefficients.lift.total                                = Clift_0
    equilibrium_conditions.aerodynamics.coefficients.drag.induced.inviscid                     = Cdrag_0     

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
    Clift_alpha_prime,Cdrag_alpha_prime,CX_alpha_prime,CY_alpha_prime,CZ_alpha_prime,CL_alpha_prime,CM_alpha_prime,CN_alpha_prime,_,_,_,_,_ ,_,Clift_wings_alpha_prime,Cdrag_wings_alpha_prime,_,_, _, _= call_VLM(pertubation_conditions,settings,vehicle)    
     
    for wing in vehicle.wings: 
        pertubation_conditions.aerodynamics.coefficients.lift.induced.inviscid_wings[wing.tag]         = Clift_wings_alpha_prime[wing.tag]
        pertubation_conditions.aerodynamics.coefficients.lift.compressible_wings[wing.tag]     = Clift_wings_alpha_prime[wing.tag]
        pertubation_conditions.aerodynamics.coefficients.drag.induced.inviscid_wings[wing.tag] = Cdrag_wings_alpha_prime[wing.tag] 
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
    conditions.static_stability.derivatives.CY_alpha    = (CY_alpha_prime      - CY_0) / (delta_angle) # BUG IN VLM
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
    _,_,_,_,_,_,CM_alpha_cg_prime,_,_,_,_,_,_ ,_,_,_,_,_, _, _= call_VLM(pertubation_conditions,settings,vehicle_shifted_CG)    
  
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
    
    Clift_beta_prime,Cdrag_beta_prime,CX_beta_prime,CY_beta_prime,CZ_beta_prime,CL_beta_prime,CM_beta_prime,CN_beta_prime ,_,_,_,_,_ ,_,_,_,_,_, _, _= call_VLM(pertubation_conditions,settings,vehicle)   
 
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
       
    Clift_u_prime,Cdrag_u_prime,CX_u_prime,CY_u_prime,CZ_u_prime,CL_u_prime,CM_u_prime,CN_u_prime ,_,_,_,_,_,_,Clift_wings_u_prime,Cdrag_wings_u_prime,_,_, _, _= call_VLM(pertubation_conditions,settings,vehicle)

    # Dimensionalize the lift and drag for each wing 
    for wing in vehicle.wings: 
        pertubation_conditions.aerodynamics.coefficients.lift.induced.inviscid_wings[wing.tag]         = Clift_wings_u_prime[wing.tag]
        pertubation_conditions.aerodynamics.coefficients.lift.compressible_wings[wing.tag]     = Clift_wings_u_prime[wing.tag]
        pertubation_conditions.aerodynamics.coefficients.drag.induced.inviscid_wings[wing.tag] = Cdrag_wings_u_prime[wing.tag] 
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
    
    Clift_v_prime,Cdrag_v_prime,CX_v_prime,CY_v_prime,CZ_v_prime,CL_v_prime,CM_v_prime,CN_v_prime ,_,_,_,_,_,_,_,_,_,_, _, _= call_VLM(pertubation_conditions,settings,vehicle)   
 
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
    
    Clift_w_prime,Cdrag_w_prime,CX_w_prime,CY_w_prime,CZ_w_prime,CL_w_prime,CM_w_prime,CN_w_prime ,_,_,_,_,_,_,_,_,_,_, _, _= call_VLM(pertubation_conditions,settings,vehicle)   
 
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
    
    Clift_p_prime,Cdrag_p_prime,CX_p_prime,CY_p_prime,CZ_p_prime,CL_p_prime,CM_p_prime,CN_p_prime ,_,_,_,_,_,_,_,_,_,_, _, _= call_VLM(pertubation_conditions,settings,vehicle)   

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
    
    Clift_q_prime,Cdrag_q_prime,CX_q_prime,CY_q_prime,CZ_q_prime,CL_q_prime,CM_q_prime,CN_q_prime  ,_,_,_,_,_ ,_,Clift_wings_q_prime,Cdrag_wings_q_prime,_,_, _, _= call_VLM(pertubation_conditions,settings,vehicle)

    # Dimensionalize the lift and drag for each wing 
    for wing in vehicle.wings: 
        pertubation_conditions.aerodynamics.coefficients.lift.induced.inviscid_wings[wing.tag] = Clift_wings_q_prime[wing.tag]
        pertubation_conditions.aerodynamics.coefficients.lift.compressible_wings[wing.tag]     = Clift_wings_q_prime[wing.tag]
        pertubation_conditions.aerodynamics.coefficients.drag.induced.inviscid_wings[wing.tag] = Cdrag_wings_q_prime[wing.tag] 
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
    
    Clift_r_prime,Cdrag_r_prime,CX_r_prime,CY_r_prime,CZ_r_prime,CL_r_prime,CM_r_prime,CN_r_prime ,_,_,_,_,_,_,_,_,_,_,_, _= call_VLM(pertubation_conditions,settings,vehicle)   

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
                    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_,_= call_VLM(pertubation_conditions,settings,vehicle)    
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
                    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_,_= call_VLM(pertubation_conditions,settings,vehicle)    
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
                    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_,_= call_VLM(pertubation_conditions,settings,vehicle)    
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
                    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_,_ = call_VLM(pertubation_conditions,settings,vehicle)    
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
                    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_,_= call_VLM(pertubation_conditions,settings,vehicle)    
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

    # transonic 
    trans_coef  = np.atleast_2d(trans_sur_coef(pts)).T    

    # supersonic 
    sup_coef  = np.atleast_2d(sub_sur_coef(pts)).T             

    # apply  
    coef = h_sub(Mach)*sub_coef +   (1 - (h_sup(Mach) + h_sub(Mach)))*trans_coef  + h_sub(Mach)*sup_coef 

  
    return coef

def call_VLM(conditions,settings,vehicle):
    """Calculate aerodynamics coefficients inluding specific wing coefficients using the VLM
        
    Assumptions:
        None
        
    Source:
        None

    Args: 
        conditions : flight conditions     [unitless]
        settings   : VLM analysis settings [unitless]
        vehicle    : vehicle configuration [unitless] 
        
    Returns: 
        None  
    """
 
    Clift_wings         = Data()
    Cdrag_wings         = Data()
    AoA_wing_induced    = Data()
    
    results             = VLM(conditions,settings,vehicle)
    Clift               = results.CL       
    Cdrag               = results.CDi     
    Clift_w             = results.CL_wing        
    Cdrag_w             = results.CDi_wing       
    CX                  = results.CX       
    CY                  = results.CY       
    CZ                  = results.CZ       
    CL                  = results.CL_mom
    alpha_i             = results.alpha_i 
    cl_y                = results.cl_y     
    cdi_y               = results.cdi_y 
    CPi                 = results.CP 
    CM                  = results.CM       
    CN                  = results.CN  
    S_ref               = results.S_ref    
    b_ref               = results.b_ref    
    c_ref               = results.c_ref    
    X_ref               = results.X_ref    
    Y_ref               = results.Y_ref    
    Z_ref               = results.Z_ref 

    # Dimensionalize the lift and drag for each wing
    areas               = vehicle.vortex_distribution.wing_areas
    dim_wing_lifts      = Clift_w  * areas
    dim_wing_drags      = Cdrag_w * areas
    
    i = 0
    # Assign the lift and drag and non-dimensionalize
    for wing in vehicle.wings.values():
        ref = wing.areas.reference
        if wing.symmetric:
            Clift_wings[wing.tag]      = np.atleast_2d(np.sum(dim_wing_lifts[:,i:(i+2)],axis=1)).T/ref
            Cdrag_wings[wing.tag]      = np.atleast_2d(np.sum(dim_wing_drags[:,i:(i+2)],axis=1)).T/ref
            AoA_wing_induced[wing.tag] = np.concatenate((alpha_i[i],alpha_i[i+1]),axis=1)
            i+=1
        else:
            Clift_wings[wing.tag]      = np.atleast_2d(dim_wing_lifts[:,i]).T/ref
            Cdrag_wings[wing.tag]      = np.atleast_2d(dim_wing_drags[:,i]).T/ref
            AoA_wing_induced[wing.tag] = alpha_i[i]
        i+=1

    return Clift,Cdrag,CX,CY,CZ,CL,CM,CN, S_ref,b_ref,c_ref,X_ref,Y_ref ,Z_ref, Clift_wings,Cdrag_wings,AoA_wing_induced,cl_y,cdi_y,CPi  
