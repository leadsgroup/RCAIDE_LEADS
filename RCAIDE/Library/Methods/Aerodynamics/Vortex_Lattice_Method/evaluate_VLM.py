# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/evaluate_VLM.py
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
import RCAIDE 
from RCAIDE.Framework.Core                                               import Data, orientation_product 
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM       import VLM
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.control_surface_registry import lookup as cs_lookup, CONTROL_SURFACE_TYPES
from RCAIDE.Library.Methods.Utilities                                    import Cubic_Spline_Blender
from RCAIDE.Library.Mission.Common.Update                                import orientations
from RCAIDE.Library.Mission.Common.Unpack_Unknowns                       import orientation

# package imports
import numpy   as np
from copy      import  deepcopy 

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
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
    hsub_min         = aerodynamics.surrogates.subsonic_smoothing_min   
    hsub_max         = aerodynamics.surrogates.subsonic_smoothing_max   
    hsup_min         = aerodynamics.surrogates.supersonic_smoothing_min 
    hsup_max         = aerodynamics.surrogates.supersonic_smoothing_max 

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
                                         sub_sur.Clift_spanwise, trans_sur.Clift_spanwise, sup_sur.Clift_spanwise,
                                         h_sub,h_sup,Mach, pts_alpha)   
      
    conditions.aerodynamics.coefficients.lift.inviscid.total    = results_alpha.Clift
    conditions.aerodynamics.coefficients.drag.induced.inviscid  = results_alpha.Cdrag
    conditions.static_stability.coefficients.M                  = results_alpha.CM
    
    conditions.static_stability.coefficients.M_0 = compute_stability_derivative(sub_sur.CM_0    ,trans_sur.CM_0    ,sup_sur.CM_0    ,h_sub,h_sup,Mach) 
    conditions.aerodynamics.coefficients.lift.spanwise =  results_alpha.Clift_spanwise     
    
    # -----------------------------------------------------------------------------------------------------------------------
    # Query control surface surrogates if derivatives are not user defined
    # ----------------------------------------------------------------------------------------------------------------------- 
    if aerodynamics.stability_derivatives.CX_alpha == None:  
        conditions.static_stability.derivatives.CX_alpha    = compute_stability_derivative(sub_sur.dCX_dalpha    ,trans_sur.dCX_dalpha    ,sup_sur.dCX_dalpha    ,h_sub,h_sup,Mach)  
    else:
        conditions.static_stability.derivatives.CX_alpha    = aerodynamics.stability_derivatives.CX_alpha * ones_row
    
    if aerodynamics.stability_derivatives.CZ_alpha == None:
        conditions.static_stability.derivatives.CZ_alpha    = compute_stability_derivative(sub_sur.dCZ_dalpha    ,trans_sur.dCZ_dalpha    ,sup_sur.dCZ_dalpha    ,h_sub,h_sup,Mach) 
    else:
        conditions.static_stability.derivatives.CZ_alpha    = aerodynamics.stability_derivatives.CZ_alpha * ones_row
    
    if aerodynamics.stability_derivatives.CM_alpha == None:
        conditions.static_stability.derivatives.CM_alpha    = compute_stability_derivative(sub_sur.dCM_dalpha    ,trans_sur.dCM_dalpha    ,sup_sur.dCM_dalpha    ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CM_alpha    = aerodynamics.stability_derivatives.CM_alpha * ones_row 
        
    if aerodynamics.stability_derivatives.CY_beta == None:
        conditions.static_stability.derivatives.CY_beta     = compute_stability_derivative(sub_sur.dCY_dbeta     ,trans_sur.dCY_dbeta     ,sup_sur.dCY_dbeta     ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CY_beta     = aerodynamics.stability_derivatives.CY_beta * ones_row
    
    if aerodynamics.stability_derivatives.CL_beta == None:
        conditions.static_stability.derivatives.CL_beta     = compute_stability_derivative(sub_sur.dCL_dbeta     ,trans_sur.dCL_dbeta     ,sup_sur.dCL_dbeta     ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CL_beta     = aerodynamics.stability_derivatives.CL_beta * ones_row
    
    if aerodynamics.stability_derivatives.CN_beta == None:
        conditions.static_stability.derivatives.CN_beta     = compute_stability_derivative(sub_sur.dCN_dbeta     ,trans_sur.dCN_dbeta     ,sup_sur.dCN_dbeta     ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CN_beta     = aerodynamics.stability_derivatives.CN_beta * ones_row

    if aerodynamics.stability_derivatives.CX_u == None:
        conditions.static_stability.derivatives.CX_u        = compute_stability_derivative(sub_sur.dCX_du        ,trans_sur.dCX_du        ,sup_sur.dCX_du        ,h_sub,h_sup,Mach)   
    else:
        conditions.static_stability.derivatives.CX_u        = aerodynamics.stability_derivatives.CX_u * ones_row
    
    if aerodynamics.stability_derivatives.CZ_u == None:
        conditions.static_stability.derivatives.CZ_u        = compute_stability_derivative(sub_sur.dCZ_du        ,trans_sur.dCZ_du        ,sup_sur.dCZ_du        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CZ_u        = aerodynamics.stability_derivatives.CZ_u * ones_row
    
    if aerodynamics.stability_derivatives.CM_u == None:
        conditions.static_stability.derivatives.CM_u        = compute_stability_derivative(sub_sur.dCM_du        ,trans_sur.dCM_du        ,sup_sur.dCM_du        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CM_u        = aerodynamics.stability_derivatives.CM_u * ones_row
    
    if aerodynamics.stability_derivatives.CY_r == None:
        conditions.static_stability.derivatives.CY_r        = compute_stability_derivative(sub_sur.dCY_dr        ,trans_sur.dCY_dr        ,sup_sur.dCY_dr        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CY_r        = aerodynamics.stability_derivatives.CY_r * ones_row
    
    if aerodynamics.stability_derivatives.CZ_q == None:
        conditions.static_stability.derivatives.CZ_q        = compute_stability_derivative(sub_sur.dCZ_dq        ,trans_sur.dCZ_dq        ,sup_sur.dCZ_dq        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CZ_q        = aerodynamics.stability_derivatives.CZ_q*ones_row
    
    if aerodynamics.stability_derivatives.CL_p == None:
        conditions.static_stability.derivatives.CL_p        = compute_stability_derivative(sub_sur.dCL_dp        ,trans_sur.dCL_dp        ,sup_sur.dCL_dp        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CL_p        = aerodynamics.stability_derivatives.CL_p*ones_row
    
    if aerodynamics.stability_derivatives.CL_r == None:
        conditions.static_stability.derivatives.CL_r        = compute_stability_derivative(sub_sur.dCL_dr        ,trans_sur.dCL_dr        ,sup_sur.dCL_dr        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CL_r        = aerodynamics.stability_derivatives.CL_r * ones_row
    
    if aerodynamics.stability_derivatives.CM_q == None:
        conditions.static_stability.derivatives.CM_q        = compute_stability_derivative(sub_sur.dCM_dq        ,trans_sur.dCM_dq        ,sup_sur.dCM_dq        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CM_q        = aerodynamics.stability_derivatives.CM_q * ones_row
    
    if aerodynamics.stability_derivatives.CN_p == None:
        conditions.static_stability.derivatives.CN_p        = compute_stability_derivative(sub_sur.dCN_dp        ,trans_sur.dCN_dp        ,sup_sur.dCN_dp        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CN_p        = aerodynamics.stability_derivatives.CN_p * ones_row
    
    if aerodynamics.stability_derivatives.CN_r == None:
        conditions.static_stability.derivatives.CN_r        = compute_stability_derivative(sub_sur.dCN_dr        ,trans_sur.dCN_dr        ,sup_sur.dCN_dr        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.CN_r        = aerodynamics.stability_derivatives.CN_r * ones_row 
     
    if aerodynamics.stability_derivatives.Clift_alpha == None:
        conditions.static_stability.derivatives.Clift_alpha = compute_stability_derivative(sub_sur.dClift_dalpha        ,trans_sur.dClift_dalpha        ,sup_sur.dClift_dalpha        ,h_sub,h_sup,Mach)
    else:
        conditions.static_stability.derivatives.Clift_alpha = aerodynamics.stability_derivatives.Clift_alpha * ones_row 
     
    conditions.static_stability.coefficients.Y      = conditions.static_stability.derivatives.CY_beta * Beta
    conditions.static_stability.coefficients.L      = conditions.static_stability.derivatives.CL_beta * Beta 
    conditions.static_stability.coefficients.N      = conditions.static_stability.derivatives.CN_beta * Beta
 
    # -----------------------------------------------------------------------------------------------------------------------
    # Addition of Control Surface Effect
    # -----------------------------------------------------------------------------------------------------------------------
    # see control_surface_registry.py for why every control surface is handled through one generic path here.
    # Induced drag is added as an absolute value (it cannot reduce total induced drag regardless of deflection
    # direction), unlike the signed linear terms used for Y/L/N/M/lift. A compound surface (Flaperon, Elevon,
    # Ruddervator) contributes two rows sharing one `name` conditions block, so that block's diagnostic
    # coefficients are reset once per call and accumulated with += across its rows rather than overwritten.
    touched_names = set()
    for cls, letter, name, channel, flag, deflection_attr in CONTROL_SURFACE_TYPES:
        if not getattr(aerodynamics, flag):
            continue

        cs_conditions = getattr(conditions.control_surfaces, name)
        deflection    = getattr(cs_conditions, deflection_attr)

        if name not in touched_names:
            cs_conditions.static_stability.coefficients.Y = 0 * ones_row
            cs_conditions.static_stability.coefficients.L = 0 * ones_row
            cs_conditions.static_stability.coefficients.N = 0 * ones_row
            cs_conditions.static_stability.coefficients.M = 0 * ones_row
            touched_names.add(name)

        derivative = {}
        for coeff in ('CY', 'CL', 'CN', 'CM', 'Clift', 'Cdrag'):
            key      = coeff + '_delta_' + letter
            override = getattr(aerodynamics.stability_derivatives, key)
            if override is None:
                surrogate_key     = 'd' + coeff + '_ddelta_' + letter
                derivative[coeff] = compute_stability_derivative(getattr(sub_sur, surrogate_key), getattr(trans_sur, surrogate_key),
                                                                   getattr(sup_sur, surrogate_key), h_sub, h_sup, Mach)
            else:
                derivative[coeff] = override * ones_row
            # Cdrag_delta is always non-negative, same convention as trim_drag.py
            if coeff == 'Cdrag':
                derivative[coeff] = np.abs(derivative[coeff])
            setattr(conditions.static_stability.derivatives, key, derivative[coeff])

        conditions.static_stability.coefficients.Y                 += derivative['CY']    * deflection
        conditions.static_stability.coefficients.L                 += derivative['CL']    * deflection
        conditions.static_stability.coefficients.N                 += derivative['CN']    * deflection
        conditions.static_stability.coefficients.M                 += derivative['CM']    * deflection
        conditions.static_stability.coefficients.Z                 += derivative['Clift'] * deflection
        conditions.aerodynamics.coefficients.lift.inviscid.total   += derivative['Clift'] * deflection
        conditions.aerodynamics.coefficients.drag.induced.inviscid += derivative['Cdrag'] * np.abs(deflection)

        cs_conditions.static_stability.coefficients.Y += derivative['CY'] * deflection
        cs_conditions.static_stability.coefficients.L += derivative['CL'] * deflection
        cs_conditions.static_stability.coefficients.N += derivative['CN'] * deflection
        cs_conditions.static_stability.coefficients.M += derivative['CM'] * deflection

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
    alt           =  conditions.freestream.altitude
    g             =  conditions.freestream.gravity
    V             =  conditions.freestream.velocity
    MAC           = vehicle.reference_chord
    b             =  vehicle.reference_span

    # Only the baseline (undeflected-query) VLM solve below is aeroelastically
    # coupled. The stability-derivative perturbations further down this
    # function stay rigid - converging each of those too would multiply the
    # cost of every mission point several times over for a second-order
    # correction to an already-approximate finite-difference slope.
    aerostructural_analyses = getattr(state.analyses, 'aerostructures', None)
    coupled = (aerostructural_analyses is not None
               and aerostructural_analyses.settings.aeroelastic_coupling == 'coupled')
    if coupled:
        VLM_results, convergence = _evaluate_coupled_baseline(conditions, settings, vehicle, aerostructural_analyses)
        conditions.aerostructures.convergence = convergence
    else:
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
    conditions.aerodynamics.coefficients.lift.spanwise                = VLM_results.sectional_CLift
    conditions.aerodynamics.coefficients.drag.induced.wings           = VLM_results.CDrag_induced_wings
    conditions.aerodynamics.coefficients.drag.induced.spanwise        = VLM_results.sectional_CDrag_induced
    conditions.aerodynamics.coefficients.drag.induced.inviscid        = Cdrag
    conditions.aerodynamics.coefficients.differential_surface_pressure= VLM_results.CP
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
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.trim_drag(state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.total_drag(state,settings,vehicle)
    

    T_wind2inertial = conditions.frames.wind.transform_to_inertial   
    no_beta   = np.all(conditions.aerodynamics.angles.beta == 0)
    aileron   = getattr(conditions.control_surfaces, 'aileron', None)
    rudder    = getattr(conditions.control_surfaces, 'rudder', None)
    no_ail    = True if aileron is None else np.all(aileron.deflection == 0)
    no_rud    = True if rudder is None else np.all(rudder.deflection == 0)
    no_bank   = np.all(conditions.aerodynamics.angles.phi == 0)  

    if no_beta and no_ail and no_rud and no_bank:
        CY = CY * 0 
    conditions.static_stability.coefficients.X     = CX 
    conditions.static_stability.coefficients.Y     = CY 
    conditions.static_stability.coefficients.Z     = CZ 
    conditions.static_stability.coefficients.L     = CL 
    conditions.static_stability.coefficients.M     = CM  
    conditions.static_stability.coefficients.N     = CN      
    
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
    equilibrium_conditions =  create_conditions(n_cpts,alt,g,V,MAC,conditions.energy)
    VLM_results = VLM(equilibrium_conditions,settings,vehicle)
    CY_0     = VLM_results.CY
    CZ_0     = VLM_results.CZ
    CL_0     = VLM_results.CL
    CM_0     = VLM_results.CM
    CN_0     = VLM_results.CN
    # induced-only baseline; rate/control-surface derivatives below diff against raw induced drag, not total_drag
    Cdrag_induced_0 = VLM_results.CDrag_induced
     
    # store CM at 0 AoA
    conditions.static_stability.coefficients.M_0 =  CM_0
    
    # Dimensionalize the lift and drag for each wing   
    equilibrium_conditions.aerodynamics.coefficients.lift.inviscid.wings          = VLM_results.CLift_wings 
    equilibrium_conditions.aerodynamics.coefficients.lift.inviscid.total          = VLM_results.CLift
    equilibrium_conditions.aerodynamics.coefficients.lift.spanwise                = VLM_results.sectional_CLift
    equilibrium_conditions.aerodynamics.coefficients.drag.induced.wings           = VLM_results.CDrag_induced_wings
    equilibrium_conditions.aerodynamics.coefficients.drag.induced.spanwise        = VLM_results.sectional_CDrag_induced
    equilibrium_conditions.aerodynamics.coefficients.drag.induced.inviscid        = VLM_results.CDrag_induced
    equilibrium_conditions.aerodynamics.coefficients.differential_surface_pressure= VLM_results.CP
    equilibrium_conditions.aerodynamics.angles.induced                            = VLM_results.alpha_induced    
    equilibrium_conditions.aerodynamics.spanwise_stations                         = VLM_results.spanwise_stations    
    
    equilibrium_state                    = RCAIDE.Framework.Mission.Common.State()
    equilibrium_state.conditions         = equilibrium_conditions  
    equilibrium_segment                  = RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude()
    equilibrium_segment.conditions       = equilibrium_conditions
    equilibrium_segment.state.conditions = equilibrium_conditions
    orientation(equilibrium_segment)
    orientations(equilibrium_segment)

    RCAIDE.Library.Methods.Aerodynamics.Common.Lift.fuselage_correction(equilibrium_state,settings,vehicle)      
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
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.form_drag(equilibrium_state,settings,vehicle)   
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.trim_drag(equilibrium_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.total_drag(equilibrium_state,settings,vehicle)
    
    
    T_wind2inertial = equilibrium_conditions.frames.wind.transform_to_inertial 
    Cdrag_0         = equilibrium_state.conditions.aerodynamics.coefficients.drag.total
    Clift_0         = equilibrium_conditions.aerodynamics.coefficients.lift.total
    CX_0            = orientation_product(T_wind2inertial,Cdrag_0)[:,0][:,None]
     
    # --------------------------------------------------------------------------------------------      
    # Alpha Purtubation  
    # --------------------------------------------------------------------------------------------    
    pertubation_conditions   = create_conditions(n_cpts,alt,g,V,MAC,conditions.energy) 
    pertubation_conditions.aerodynamics.angles.alpha   += delta_angle
    
    VLM_results = VLM(pertubation_conditions,settings,vehicle)
    Clift_i_alpha_prime = VLM_results.CLift
    Cdrag_i_alpha_prime = VLM_results.CDrag_induced 
    CY_alpha_prime    = VLM_results.CY
    CZ_alpha_prime    = VLM_results.CZ
    CL_alpha_prime    = VLM_results.CL
    CM_alpha_prime    = VLM_results.CM
    CN_alpha_prime    = VLM_results.CN
 
    pertubation_conditions.aerodynamics.coefficients.lift.inviscid.total     = Clift_i_alpha_prime     
    pertubation_conditions.aerodynamics.coefficients.lift.inviscid.wings     = VLM_results.CLift_wings  
    pertubation_conditions.aerodynamics.coefficients.lift.spanwise           = VLM_results.sectional_CLift        
    pertubation_conditions.aerodynamics.coefficients.drag.induced.wings      = VLM_results.CDrag_induced_wings 
    pertubation_conditions.aerodynamics.coefficients.drag.induced.total      = Cdrag_i_alpha_prime
     
    perturbation_state                  = RCAIDE.Framework.Mission.Common.State()
    perturbation_state.conditions       = pertubation_conditions  
    perturbation_state                  = RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude()
    perturbation_state.conditions       = pertubation_conditions
    perturbation_state.state.conditions = pertubation_conditions
    orientation(perturbation_state)
    orientations(perturbation_state) 
    
    RCAIDE.Library.Methods.Aerodynamics.Common.Lift.fuselage_correction(perturbation_state,settings,vehicle)  
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
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.trim_drag(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.total_drag(perturbation_state,settings,vehicle) 

    T_wind2inertial   = pertubation_conditions.frames.wind.transform_to_inertial 
    Cdrag_visc_prime  = perturbation_state.conditions.aerodynamics.coefficients.drag.total
    Clift_visc_prime  = perturbation_state.conditions.aerodynamics.coefficients.lift.total
    CX_visc_prime     = orientation_product(T_wind2inertial,Cdrag_visc_prime)[:,0][:,None] 
    
    conditions.static_stability.derivatives.Clift_alpha = (Clift_visc_prime    - Clift_0) / (delta_angle)
    conditions.static_stability.derivatives.Cdrag_alpha = (Cdrag_visc_prime    - Cdrag_0) / (delta_angle)  
    conditions.static_stability.derivatives.CX_alpha    = (CX_visc_prime       - CX_0) / (delta_angle)   
    conditions.static_stability.derivatives.CY_alpha    = (CY_alpha_prime      - CY_0) / (delta_angle)  
    conditions.static_stability.derivatives.CZ_alpha    = (CZ_alpha_prime      - CZ_0) / (delta_angle) 
    conditions.static_stability.derivatives.CL_alpha    = (CL_alpha_prime      - CL_0) / (delta_angle)  
    conditions.static_stability.derivatives.CM_alpha    = (CM_alpha_prime      - CM_0) / (delta_angle)  
    conditions.static_stability.derivatives.CN_alpha    = (CN_alpha_prime      - CN_0) / (delta_angle)  
    
    # --------------------------------------------------------------------------------------------      
    # Beta Purtubation  
    # --------------------------------------------------------------------------------------------   
    pertubation_conditions =  create_conditions(n_cpts,alt,g,V,MAC,conditions.energy) 
    pertubation_conditions.aerodynamics.angles.beta         += delta_angle  

    VLM_results = VLM(pertubation_conditions,settings,vehicle)
    Clift_beta_prime = VLM_results.CLift
    Cdrag_beta_prime = VLM_results.CDrag_induced
    CX_beta_prime    = VLM_results.CX
    CY_beta_prime    = VLM_results.CY
    CZ_beta_prime    = VLM_results.CZ
    CL_beta_prime    = VLM_results.CL
    CM_beta_prime    = VLM_results.CM
    CN_beta_prime    = VLM_results.CN
    
    conditions.static_stability.derivatives.Clift_beta =   (Clift_beta_prime   - Clift_0) / (delta_angle)
    conditions.static_stability.derivatives.Cdrag_beta =   (Cdrag_beta_prime   - Cdrag_induced_0) / (delta_angle)
    conditions.static_stability.derivatives.CX_beta    =   (CX_beta_prime      - CX_0) / (delta_angle)  
    conditions.static_stability.derivatives.CY_beta    =   (CY_beta_prime      - CY_0) / (delta_angle) 
    conditions.static_stability.derivatives.CZ_beta    =   (CZ_beta_prime      - CZ_0) / (delta_angle) 
    conditions.static_stability.derivatives.CL_beta    =   (CL_beta_prime      - CL_0) / (delta_angle)   
    conditions.static_stability.derivatives.CM_beta    =   (CM_beta_prime      - CM_0) / (delta_angle)  
    conditions.static_stability.derivatives.CN_beta    =   (CN_beta_prime      - CN_0) / (delta_angle) 

    # --------------------------------------------------------------------------------------------      
    # U-Velocity Pertubation 
    # --------------------------------------------------------------------------------------------
    perturbation_state                                           = RCAIDE.Framework.Mission.Common.State()
    pertubation_conditions                                       = create_conditions(n_cpts,alt,g,V,MAC,conditions.energy)  
    pertubation_conditions.frames.inertial.velocity_vector[:,0]  += delta_speed 
    pertubation_conditions.freestream.velocity            [:,0]  += delta_speed 
    pertubation_conditions.freestream.mach_number                = np.linalg.norm(pertubation_conditions.frames.inertial.velocity_vector, axis=1)[:,None] /  equilibrium_conditions.freestream.speed_of_sound 
    pertubation_conditions.freestream.reynolds_number            = pertubation_conditions.freestream.density * pertubation_conditions.freestream.velocity * wing.chords.mean_aerodynamic/equilibrium_conditions.freestream.dynamic_viscosity   
    pertubation_conditions.freestream.dynamic_pressure           = 0.5 * pertubation_conditions.freestream.density * np.sum( pertubation_conditions.freestream.velocity**2, axis=1)[:,None] 
        
    VLM_results = VLM(pertubation_conditions,settings,vehicle)
    Clift_i_u_prime = VLM_results.CLift
    Cdrag_i_u_prime = VLM_results.CDrag_induced
    CX_u_prime    = VLM_results.CX
    CY_u_prime    = VLM_results.CY
    CZ_u_prime    = VLM_results.CZ
    CL_u_prime    = VLM_results.CL
    CM_u_prime    = VLM_results.CM
    CN_u_prime    = VLM_results.CN 

    # Dimensionalize the lift and drag for each wing  
    pertubation_conditions.aerodynamics.coefficients.lift.inviscid.wings = VLM_results.CLift_wings          
    pertubation_conditions.aerodynamics.coefficients.lift.inviscid.total = Clift_i_u_prime
    pertubation_conditions.aerodynamics.coefficients.drag.induced.total  = Cdrag_i_u_prime
    pertubation_conditions.aerodynamics.coefficients.drag.induced.wings  = VLM_results.CDrag_induced_wings
    pertubation_conditions.aerodynamics.coefficients.lift.spanwise       = VLM_results.sectional_CLift

    perturbation_state                  = RCAIDE.Framework.Mission.Common.State()
    perturbation_state.conditions       = pertubation_conditions  
    perturbation_state                  = RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude()
    perturbation_state.conditions       = pertubation_conditions
    perturbation_state.state.conditions = pertubation_conditions
    orientation(perturbation_state)
    orientations(perturbation_state)
    
    RCAIDE.Library.Methods.Aerodynamics.Common.Lift.fuselage_correction(perturbation_state,settings,vehicle)  
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
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.trim_drag(perturbation_state,settings,vehicle)
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.total_drag(perturbation_state,settings,vehicle) 

    T_wind2inertial   = pertubation_conditions.frames.wind.transform_to_inertial 
    Cdrag_visc_prime  = perturbation_state.conditions.aerodynamics.coefficients.drag.total
    Clift_visc_prime  = perturbation_state.conditions.aerodynamics.coefficients.lift.total
    CX_visc_prime     = orientation_product(T_wind2inertial,Cdrag_visc_prime)[:,0][:,None]        
 
    conditions.static_stability.derivatives.Clift_u = (Clift_visc_prime   - Clift_0) / (delta_speed)
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
    pertubation_conditions =  create_conditions(n_cpts,alt,g,V,MAC,conditions.energy) 
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
    conditions.static_stability.derivatives.Cdrag_v = (Cdrag_v_prime   - Cdrag_induced_0) / (delta_speed)
    conditions.static_stability.derivatives.CX_v    = (CX_v_prime      - CX_0) / (delta_speed)  
    conditions.static_stability.derivatives.CY_v    = (CY_v_prime      - CY_0) / (delta_speed) 
    conditions.static_stability.derivatives.CZ_v    = (CZ_v_prime      - CZ_0) / (delta_speed) 
    conditions.static_stability.derivatives.CL_v    = (CL_v_prime      - CL_0) / (delta_speed)  
    conditions.static_stability.derivatives.CM_v    = (CM_v_prime      - CM_0) / (delta_speed)  
    conditions.static_stability.derivatives.CN_v    = (CN_v_prime      - CN_0) / (delta_speed)        

    # --------------------------------------------------------------------------------------------      
    # W-Velocity Pertubation 
    # --------------------------------------------------------------------------------------------  
    pertubation_conditions =  create_conditions(n_cpts,alt,g,V,MAC,conditions.energy)    
    pertubation_conditions.frames.inertial.velocity_vector[:,2]  += delta_speed 
    pertubation_conditions.freestream.velocity                   = np.linalg.norm(pertubation_conditions.frames.inertial.velocity_vector, axis=1)[:,None]     
    pertubation_conditions.freestream.mach_number                = pertubation_conditions.freestream.velocity / pertubation_conditions.freestream.speed_of_sound   
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
    conditions.static_stability.derivatives.Cdrag_w  = (Cdrag_w_prime   - Cdrag_induced_0) / (delta_speed)
    conditions.static_stability.derivatives.CX_w     = (CX_w_prime      - CX_0) / (delta_speed)  
    conditions.static_stability.derivatives.CY_w     = (CY_w_prime      - CY_0) / (delta_speed) 
    conditions.static_stability.derivatives.CZ_w     = (CZ_w_prime      - CZ_0) / (delta_speed) 
    conditions.static_stability.derivatives.CL_w     = (CL_w_prime      - CL_0) / (delta_speed)  
    conditions.static_stability.derivatives.CM_w     = (CM_w_prime      - CM_0) / (delta_speed)  
    conditions.static_stability.derivatives.CN_w     = (CN_w_prime      - CN_0) / (delta_speed)
    

    # --------------------------------------------------------------------------------------------      
    # Roll Rate (p) Purtubation
    # --------------------------------------------------------------------------------------------  
    pertubation_conditions =  create_conditions(n_cpts,alt,g,V,MAC,conditions.energy) 
    pertubation_conditions.static_stability.roll_rate[:,0] = delta_rate  
    pertubation_conditions.static_stability.pitch_rate[:,0]= 0 
    pertubation_conditions.static_stability.yaw_rate[:,0]  = 0 
    p_hat =  delta_rate * b / (2 * V)
    
    VLM_results   = VLM(pertubation_conditions,settings,vehicle)
    Clift_p_prime = VLM_results.CLift
    Cdrag_p_prime = VLM_results.CDrag_induced
    CX_p_prime    = VLM_results.CX
    CY_p_prime    = VLM_results.CY
    CZ_p_prime    = VLM_results.CZ
    CL_p_prime    = VLM_results.CL
    CM_p_prime    = VLM_results.CM
    CN_p_prime    = VLM_results.CN
    
    conditions.static_stability.derivatives.Clift_p  =  (Clift_p_prime   - Clift_0) / (p_hat)
    conditions.static_stability.derivatives.Cdrag_p  =  (Cdrag_p_prime   - Cdrag_induced_0) / (p_hat)
    conditions.static_stability.derivatives.CX_p     =  (CX_p_prime      - CX_0)    / (p_hat)  
    conditions.static_stability.derivatives.CY_p     =  (CY_p_prime      - CY_0)    / (p_hat) 
    conditions.static_stability.derivatives.CZ_p     =  (CZ_p_prime      - CZ_0)    / (p_hat) 
    conditions.static_stability.derivatives.CL_p     =  (CL_p_prime      - CL_0)    / (p_hat)  
    conditions.static_stability.derivatives.CM_p     =  (CM_p_prime      - CM_0)    / (p_hat)  
    conditions.static_stability.derivatives.CN_p     = -(CN_p_prime      - CN_0)    / (p_hat)

    # ---------------------------------------------------------------------------------------------------      
    # Pitch Rate (q) Purtubation
    # ---------------------------------------------------------------------------------------------------    
    perturbation_state     = RCAIDE.Framework.Mission.Common.State()
    pertubation_conditions = create_conditions(n_cpts,alt,g,V,MAC,conditions.energy) 
    pertubation_conditions.static_stability.roll_rate[:,0]  = 0 
    pertubation_conditions.static_stability.pitch_rate[:,0] = delta_rate 
    pertubation_conditions.static_stability.yaw_rate[:,0]   = 0 
    q_hat = MAC * delta_rate / (2 * V)
     
    VLM_results     = VLM(pertubation_conditions,settings,vehicle)
    Clift_q_prime   = VLM_results.CLift
    Cdrag_q_prime   = VLM_results.CDrag_induced
    CX_q_prime      = VLM_results.CX
    CY_q_prime      = VLM_results.CY
    CZ_q_prime      = VLM_results.CZ
    CL_q_prime      = VLM_results.CL
    CM_q_prime      = VLM_results.CM
    CN_q_prime      = VLM_results.CN
    
    conditions.static_stability.derivatives.Clift_q  = (Clift_q_prime   - Clift_0) / (q_hat)
    conditions.static_stability.derivatives.Cdrag_q  = (Cdrag_q_prime   - Cdrag_induced_0) / (q_hat)
    conditions.static_stability.derivatives.CX_q     = (CX_q_prime      - CX_0)    / (q_hat)
    conditions.static_stability.derivatives.CY_q     = (CY_q_prime      - CY_0)    / (q_hat)  
    conditions.static_stability.derivatives.CZ_q     = (CZ_q_prime      - CZ_0)    / (q_hat)
    conditions.static_stability.derivatives.CL_q     = (CL_q_prime      - CL_0)    / (q_hat)  
    conditions.static_stability.derivatives.CM_q     = (CM_q_prime      - CM_0)    / (q_hat)  
    conditions.static_stability.derivatives.CN_q     = (CN_q_prime      - CN_0)    / (q_hat)   

    # ---------------------------------------------------------------------------------------------------      
    # Yaw Rate (r) Purtubation
    # ---------------------------------------------------------------------------------------------------     
    pertubation_conditions =  create_conditions(n_cpts,alt,g,V,MAC,conditions.energy)
    pertubation_conditions.static_stability.roll_rate[:,0]   = 0   
    pertubation_conditions.static_stability.pitch_rate[:,0]  = 0 
    pertubation_conditions.static_stability.yaw_rate[:,0]    = delta_rate
    r_hat =  delta_rate * b / (2 * V)
    
    VLM_results   = VLM(pertubation_conditions,settings,vehicle)
    Clift_r_prime = VLM_results.CLift
    Cdrag_r_prime = VLM_results.CDrag_induced
    CX_r_prime    = VLM_results.CX
    CY_r_prime    = VLM_results.CY
    CZ_r_prime    = VLM_results.CZ
    CL_r_prime    = VLM_results.CL
    CM_r_prime    = VLM_results.CM
    CN_r_prime    = VLM_results.CN
     
    conditions.static_stability.derivatives.Clift_r  =  (Clift_r_prime   - Clift_0) / (r_hat)
    conditions.static_stability.derivatives.Cdrag_r  =  (Cdrag_r_prime   - Cdrag_induced_0) / (r_hat)
    conditions.static_stability.derivatives.CX_r     =  (CX_r_prime      - CX_0)    / (r_hat)  
    conditions.static_stability.derivatives.CY_r     =  (CY_r_prime      - CY_0)    / (r_hat) 
    conditions.static_stability.derivatives.CZ_r     =  (CZ_r_prime      - CZ_0)    / (r_hat) 
    conditions.static_stability.derivatives.CL_r     =  (CL_r_prime      - CL_0)    / (r_hat) 
    conditions.static_stability.derivatives.CM_r     =  (CM_r_prime      - CM_0)    / (r_hat)  
    conditions.static_stability.derivatives.CN_r     =  (CN_r_prime      - CN_0)    / (r_hat) 
 
    # see control_surface_registry.py for why every control surface is handled through one generic path here
    baseline_coefficients = {'Clift': Clift_0, 'Cdrag': Cdrag_induced_0, 'CX': CX_0, 'CY': CY_0,
                              'CZ': CZ_0, 'CL': CL_0, 'CM': CM_0, 'CN': CN_0}

    for wing in vehicle.wings:
        for control_surface in wing.control_surfaces:
            for letter, name, channel, flag, deflection_attr in cs_lookup(control_surface):
                pertubation_conditions = create_conditions(n_cpts,alt,g,V,MAC,conditions.energy)
                original_deflection    = getattr(control_surface, deflection_attr)
                # perturb relative to the current deflection, not an absolute set-point
                setattr(control_surface, deflection_attr, original_deflection + delta_ctrl_surf)
                VLM_results             = VLM(pertubation_conditions,settings,vehicle)
                setattr(control_surface, deflection_attr, original_deflection)

                perturbed_coefficients = {'Clift': VLM_results.CLift, 'Cdrag': VLM_results.CDrag_induced,
                                           'CX': VLM_results.CX, 'CY': VLM_results.CY, 'CZ': VLM_results.CZ,
                                           'CL': VLM_results.CL, 'CM': VLM_results.CM, 'CN': VLM_results.CN}

                for coeff, prime in perturbed_coefficients.items():
                    key = coeff + '_delta_' + letter
                    derivative_value = (prime - baseline_coefficients[coeff]) / delta_ctrl_surf
                    # Cdrag_delta is always non-negative, same convention as trim_drag.py
                    if coeff == 'Cdrag':
                        derivative_value = np.abs(derivative_value)
                    conditions.static_stability.derivatives[key] = derivative_value
    return


def _stack_vortex_distribution_rows(row_vds):
    """Recombine single-row settings.vortex_distribution snapshots (one per
    per-row VLM call in the coupled baseline path) into one multi-row Data,
    matching the (n_cpts, N) shape VLM() normally produces for a full batch.
    """
    stacked = Data()
    for key in row_vds[0].keys():
        stacked[key] = np.vstack([row[key] for row in row_vds])
    return stacked


def _evaluate_coupled_baseline(full_conditions, settings, vehicle, aerostructural_analyses, tol=1e-4, max_iter=15):
    """Per-control-point VLM<->FEA convergence for the baseline (no-surrogate)
    aerodynamic evaluation only - see the call site in evaluate_no_surrogate
    for why the stability-derivative perturbations are excluded.

    Returns (VLM_results, convergence) where VLM_results carries just the
    fields evaluate_no_surrogate reads from the baseline call, and
    convergence.delta is a (n_cpts, max_iter) NaN-padded relative-deflection
    trace (see _converge_aeroelastic).
    """
    from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates import _converge_aeroelastic
    from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.generate_vortex_distribution import generate_vortex_distribution

    num_cases  = len(full_conditions.aerodynamics.angles.alpha)
    jig_vd     = None
    conv_delta = np.full((num_cases, max_iter), np.nan)

    result_fields = ('CLift','CDrag_induced','CX','CY','CZ','CL','CM','CN',
                      'CLift_wings','sectional_CLift','CDrag_induced_wings',
                      'sectional_CDrag_induced','CP','alpha_induced','spanwise_stations')
    stacked_results = {f: [] for f in result_fields}
    vd_rows         = []

    for i in range(num_cases):
        conditions = RCAIDE.Framework.Mission.Common.Results()
        conditions.freestream.mach_number                = np.atleast_2d(full_conditions.freestream.mach_number[i,:])
        conditions.freestream.velocity                   = np.atleast_2d(full_conditions.freestream.velocity[i,:])
        conditions.freestream.density                    = np.atleast_2d(full_conditions.freestream.density[i,:])
        conditions.freestream.dynamic_pressure            = np.atleast_2d(full_conditions.freestream.dynamic_pressure[i,:])
        # FEA() expects gravitational_acceleration; mission conditions carry
        # the same quantity as freestream.gravity.
        conditions.freestream.gravitational_acceleration  = np.atleast_2d(full_conditions.freestream.gravity[i,:])
        conditions.aerodynamics.angles.alpha              = np.atleast_2d(full_conditions.aerodynamics.angles.alpha[i,:])
        conditions.aerodynamics.angles.beta               = np.atleast_2d(full_conditions.aerodynamics.angles.beta[i,:])
        conditions.static_stability.pitch_rate            = np.atleast_2d(full_conditions.static_stability.pitch_rate[i,:])
        conditions.static_stability.roll_rate             = np.atleast_2d(full_conditions.static_stability.roll_rate[i,:])
        conditions.static_stability.yaw_rate              = np.atleast_2d(full_conditions.static_stability.yaw_rate[i,:])
        conditions.aerostructures = Data()
        if 'weights' in full_conditions:
            conditions.weights = Data()
            conditions.weights.components = Data()
            conditions.weights.components.mass = Data()
            for tag in full_conditions.weights.components.mass.keys():
                conditions.weights.components.mass[tag] = np.atleast_2d(
                    full_conditions.weights.components.mass[tag][i,:])

        if jig_vd is None:
            jig_vd = generate_vortex_distribution(conditions, settings, vehicle)

        VLM_i, _, history_i = _converge_aeroelastic(
            conditions, settings, vehicle, aerostructural_analyses, jig_vd, tol, max_iter)

        conv_delta[i, :len(history_i)] = history_i
        for f in result_fields:
            stacked_results[f].append(VLM_i[f])
        # settings.vortex_distribution is overwritten by the next row's VLM
        # call, so snapshot this row's deformed mesh before it's clobbered.
        vd_rows.append(deepcopy(settings.vortex_distribution))

    VLM_results = Data()
    for f in result_fields:
        VLM_results[f] = np.vstack(stacked_results[f])

    settings.vortex_distribution = _stack_vortex_distribution_rows(vd_rows)

    return VLM_results, Data(delta=conv_delta)


def create_conditions(n_cpts,altitude,g,V,MAC,energy_conditions):
    
    atmosphere                                                         = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data                                                          = atmosphere.compute_values(altitude =altitude)  
    equilibrium_conditions                                             = RCAIDE.Framework.Mission.Common.Results()
    equilibrium_conditions.expand_rows(n_cpts,override=False)
    equilibrium_conditions.energy                                      = deepcopy(energy_conditions)
    equilibrium_conditions.freestream.density[:,0]                     = atmo_data.density[:,0]
    equilibrium_conditions.freestream.gravity[:,0]                     = g[:,0]
    equilibrium_conditions.freestream.speed_of_sound[:,0]              = atmo_data.speed_of_sound[:,0]
    equilibrium_conditions.freestream.dynamic_viscosity[:,0]           = atmo_data.dynamic_viscosity[:,0]
    equilibrium_conditions.aerodynamics.angles.alpha[:,0]              = 1E-12
    equilibrium_conditions.freestream.temperature[:,0]                 = atmo_data.temperature[:,0]
    equilibrium_conditions.freestream.velocity[:,0]                    = V[:,0]    
    equilibrium_conditions.frames.inertial.velocity_vector[:,0]        = equilibrium_conditions.freestream.velocity[:,0]
    equilibrium_conditions.freestream.mach_number                      = equilibrium_conditions.freestream.velocity/equilibrium_conditions.freestream.speed_of_sound
    equilibrium_conditions.freestream.dynamic_pressure                 = 0.5 * equilibrium_conditions.freestream.density *  (equilibrium_conditions.freestream.velocity ** 2)
    equilibrium_conditions.freestream.reynolds_number                  = equilibrium_conditions.freestream.density * equilibrium_conditions.freestream.velocity * MAC/ equilibrium_conditions.freestream.dynamic_viscosity  

    return equilibrium_conditions

def compute_stability_derivative(sub_sur,trans_sur,sup_sur,h_sub,h_sup,Mach):
    if trans_sur ==  None and  sup_sur == None:
        derivative = h_sub(Mach)*sub_sur(Mach) 
        return derivative
        
    derivative = h_sub(Mach)*sub_sur(Mach) +   (1 - (h_sup(Mach) + h_sub(Mach)))*trans_sur(Mach)  + h_sup(Mach)*sup_sur(Mach) 
    return derivative



def compute_coefficients(sub_sur_Clift,sub_sur_Cdrag,sub_sur_CX,sub_sur_CY,sub_sur_CZ,sub_sur_CL,sub_sur_CM,sub_sur_CN,
                         trans_sur_Clift,trans_sur_Cdrag,trans_sur_CX,trans_sur_CY,trans_sur_CZ,trans_sur_CL,trans_sur_CM,trans_sur_CN,
                         sup_sur_Clift,sup_sur_Cdrag,sup_sur_CX,sup_sur_CY,sup_sur_CZ,sup_sur_CL,sup_sur_CM,sup_sur_CN, sub_sur_cl_spanwise, trans_sur_cl_spanwise, sup_sur_cl_spanwise,
                         h_sub,h_sup,Mach, pts): 
    

     #  subsonic 
    sub_Clift          = np.atleast_2d(sub_sur_Clift(pts)).T  
    sub_Cdrag          = np.atleast_2d(sub_sur_Cdrag(pts)).T  
    sub_CX             = np.atleast_2d(sub_sur_CX(pts)).T 
    sub_CY             = np.atleast_2d(sub_sur_CY(pts)).T     
    sub_CZ             = np.atleast_2d(sub_sur_CZ(pts)).T     
    sub_CL             = np.atleast_2d(sub_sur_CL(pts)).T     
    sub_CM             = np.atleast_2d(sub_sur_CM(pts)).T     
    sub_CN             = np.atleast_2d(sub_sur_CN(pts)).T
    sub_Clift_y        = sub_sur_cl_spanwise(pts) 
    
    
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
        results.Clift_spanwise = h_sub(Mach)*sub_Clift_y
        
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
    trans_Clift_y = trans_sur_cl_spanwise(pts) 

    # supersonic 
    sup_Clift     = np.atleast_2d(sup_sur_Clift(pts)).T  
    sup_Cdrag     = np.atleast_2d(sup_sur_Cdrag(pts)).T  
    sup_CX        = np.atleast_2d(sup_sur_CX(pts)).T 
    sup_CY        = np.atleast_2d(sup_sur_CY(pts)).T     
    sup_CZ        = np.atleast_2d(sup_sur_CZ(pts)).T     
    sup_CL        = np.atleast_2d(sup_sur_CL(pts)).T     
    sup_CM        = np.atleast_2d(sup_sur_CM(pts)).T     
    sup_CN        = np.atleast_2d(sup_sur_CN(pts)).T   
    sup_Clift_y   = sup_sur_cl_spanwise(pts)          

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
    results.Clift_spanwise = h_sub(Mach)*sub_Clift_y    + (1 - (h_sup(Mach) + h_sub(Mach)))*trans_Clift_y     + h_sup(Mach)*sup_Clift_y

    return results


