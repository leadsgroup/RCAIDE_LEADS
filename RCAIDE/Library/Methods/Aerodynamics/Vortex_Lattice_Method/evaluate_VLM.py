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
    # Same generic override-or-compute pattern as the control-surface loop below: a surrogate
    # value unless the user supplied an override in aerodynamics.stability_derivatives.
    for name in ('CX_alpha','CZ_alpha','CM_alpha','CY_beta','CL_beta','CN_beta',
                 'CX_u','CZ_u','CM_u','CY_r','CZ_q','CL_p','CL_r','CM_q','CN_p','CN_r',
                 'Clift_alpha'):
        coeff, deriv  = name.rsplit('_', 1)
        surrogate_key = f'd{coeff}_d{deriv}'
        override      = getattr(aerodynamics.stability_derivatives, name)
        if override == None:
            value = compute_stability_derivative(getattr(sub_sur, surrogate_key), getattr(trans_sur, surrogate_key),
                                                  getattr(sup_sur, surrogate_key), h_sub, h_sup, Mach)
        else:
            value = override * ones_row
        setattr(conditions.static_stability.derivatives, name, value)

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

    apply_drag_corrections(equilibrium_state,settings,vehicle)

    T_wind2inertial = equilibrium_conditions.frames.wind.transform_to_inertial
    Cdrag_0         = equilibrium_state.conditions.aerodynamics.coefficients.drag.total
    Clift_0         = equilibrium_conditions.aerodynamics.coefficients.lift.total
    CX_0            = orientation_product(T_wind2inertial,Cdrag_0)[:,0][:,None]

    # CY/CZ/CL/CM/CN baselines are always the induced-only values from the equilibrium VLM()
    # call above (never re-derived through drag corrections); Clift/Cdrag/CX mix viscous and
    # induced baselines depending on perturbation type -- see assign_moment_derivatives().
    baseline_coefficients = {'Clift': Clift_0, 'Cdrag': Cdrag_induced_0, 'CX': CX_0, 'CY': CY_0,
                              'CZ': CZ_0, 'CL': CL_0, 'CM': CM_0, 'CN': CN_0}

    # --------------------------------------------------------------------------------------------      
    # Alpha Purtubation  
    # --------------------------------------------------------------------------------------------    
    pertubation_conditions   = create_conditions(n_cpts,alt,g,V,MAC,conditions.energy) 
    pertubation_conditions.aerodynamics.angles.alpha   += delta_angle
    
    VLM_results = VLM(pertubation_conditions,settings,vehicle)

    pertubation_conditions.aerodynamics.coefficients.lift.inviscid.total     = VLM_results.CLift
    pertubation_conditions.aerodynamics.coefficients.lift.inviscid.wings     = VLM_results.CLift_wings
    pertubation_conditions.aerodynamics.coefficients.lift.spanwise           = VLM_results.sectional_CLift
    pertubation_conditions.aerodynamics.coefficients.drag.induced.wings      = VLM_results.CDrag_induced_wings
    pertubation_conditions.aerodynamics.coefficients.drag.induced.total      = VLM_results.CDrag_induced

    Clift_visc_prime, Cdrag_visc_prime, CX_visc_prime = compute_viscous_prime(pertubation_conditions,settings,vehicle)

    conditions.static_stability.derivatives.Clift_alpha = (Clift_visc_prime - Clift_0) / (delta_angle)
    conditions.static_stability.derivatives.Cdrag_alpha = (Cdrag_visc_prime - Cdrag_0) / (delta_angle)
    conditions.static_stability.derivatives.CX_alpha    = (CX_visc_prime    - CX_0)    / (delta_angle)
    assign_moment_derivatives(conditions, 'alpha', VLM_results, baseline_coefficients, delta_angle)

    # --------------------------------------------------------------------------------------------      
    # Beta Purtubation  
    # --------------------------------------------------------------------------------------------   
    pertubation_conditions =  create_conditions(n_cpts,alt,g,V,MAC,conditions.energy) 
    pertubation_conditions.aerodynamics.angles.beta         += delta_angle  

    VLM_results = VLM(pertubation_conditions,settings,vehicle)
    conditions.static_stability.derivatives.Clift_beta = (VLM_results.CLift        - Clift_0)        / (delta_angle)
    conditions.static_stability.derivatives.Cdrag_beta = (VLM_results.CDrag_induced - Cdrag_induced_0) / (delta_angle)
    conditions.static_stability.derivatives.CX_beta    = (VLM_results.CX           - CX_0)            / (delta_angle)
    assign_moment_derivatives(conditions, 'beta', VLM_results, baseline_coefficients, delta_angle)

    # --------------------------------------------------------------------------------------------      
    # U-Velocity Pertubation
    # --------------------------------------------------------------------------------------------
    # Reproduces the wing that apply_drag_corrections()'s "for wing in vehicle.wings" loop
    # (called for the alpha perturbation above, via compute_viscous_prime) used to leave
    # behind as an ordinary leaked loop variable, before that loop was factored into its own
    # function scope -- same value, just no longer an accidental side effect.
    wing = list(vehicle.wings)[-1]
    pertubation_conditions                                       = create_conditions(n_cpts,alt,g,V,MAC,conditions.energy)
    pertubation_conditions.frames.inertial.velocity_vector[:,0]  += delta_speed
    pertubation_conditions.freestream.velocity            [:,0]  += delta_speed
    pertubation_conditions.freestream.mach_number                = np.linalg.norm(pertubation_conditions.frames.inertial.velocity_vector, axis=1)[:,None] /  equilibrium_conditions.freestream.speed_of_sound
    pertubation_conditions.freestream.reynolds_number            = pertubation_conditions.freestream.density * pertubation_conditions.freestream.velocity * wing.chords.mean_aerodynamic/equilibrium_conditions.freestream.dynamic_viscosity
    pertubation_conditions.freestream.dynamic_pressure           = 0.5 * pertubation_conditions.freestream.density * np.sum( pertubation_conditions.freestream.velocity**2, axis=1)[:,None]

    VLM_results = VLM(pertubation_conditions,settings,vehicle)

    pertubation_conditions.aerodynamics.coefficients.lift.inviscid.wings = VLM_results.CLift_wings
    pertubation_conditions.aerodynamics.coefficients.lift.inviscid.total = VLM_results.CLift
    pertubation_conditions.aerodynamics.coefficients.drag.induced.total  = VLM_results.CDrag_induced
    pertubation_conditions.aerodynamics.coefficients.drag.induced.wings  = VLM_results.CDrag_induced_wings
    pertubation_conditions.aerodynamics.coefficients.lift.spanwise       = VLM_results.sectional_CLift

    Clift_visc_prime, Cdrag_visc_prime, CX_visc_prime = compute_viscous_prime(pertubation_conditions,settings,vehicle)

    conditions.static_stability.derivatives.Clift_u = (Clift_visc_prime - Clift_0) / (delta_speed)
    conditions.static_stability.derivatives.Cdrag_u = (Cdrag_visc_prime - Cdrag_0) / (delta_speed)
    conditions.static_stability.derivatives.CX_u    = (CX_visc_prime    - CX_0)    / (delta_speed)
    assign_moment_derivatives(conditions, 'u', VLM_results, baseline_coefficients, delta_speed)

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
    conditions.static_stability.derivatives.Clift_v = (VLM_results.CLift        - Clift_0)        / (delta_speed)
    conditions.static_stability.derivatives.Cdrag_v = (VLM_results.CDrag_induced - Cdrag_induced_0) / (delta_speed)
    conditions.static_stability.derivatives.CX_v    = (VLM_results.CX           - CX_0)            / (delta_speed)
    assign_moment_derivatives(conditions, 'v', VLM_results, baseline_coefficients, delta_speed)

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
    conditions.static_stability.derivatives.Clift_w = (VLM_results.CLift        - Clift_0)        / (delta_speed)
    conditions.static_stability.derivatives.Cdrag_w = (VLM_results.CDrag_induced - Cdrag_induced_0) / (delta_speed)
    conditions.static_stability.derivatives.CX_w    = (VLM_results.CX           - CX_0)            / (delta_speed)
    assign_moment_derivatives(conditions, 'w', VLM_results, baseline_coefficients, delta_speed)
    

    # --------------------------------------------------------------------------------------------      
    # Roll Rate (p) Purtubation
    # --------------------------------------------------------------------------------------------  
    pertubation_conditions =  create_conditions(n_cpts,alt,g,V,MAC,conditions.energy) 
    pertubation_conditions.static_stability.roll_rate[:,0] = delta_rate  
    pertubation_conditions.static_stability.pitch_rate[:,0]= 0 
    pertubation_conditions.static_stability.yaw_rate[:,0]  = 0 
    p_hat =  delta_rate * b / (2 * V)
    
    VLM_results   = VLM(pertubation_conditions,settings,vehicle)
    conditions.static_stability.derivatives.Clift_p = (VLM_results.CLift        - Clift_0)        / (p_hat)
    conditions.static_stability.derivatives.Cdrag_p = (VLM_results.CDrag_induced - Cdrag_induced_0) / (p_hat)
    conditions.static_stability.derivatives.CX_p    = (VLM_results.CX           - CX_0)            / (p_hat)
    # CN_p carries a sign flip the other 7 perturbations in this function don't -- preserved
    # exactly as it was (not investigated here; out of scope for this refactor).
    assign_moment_derivatives(conditions, 'p', VLM_results, baseline_coefficients, p_hat, negate_CN=True)

    # ---------------------------------------------------------------------------------------------------      
    # Pitch Rate (q) Purtubation
    # ---------------------------------------------------------------------------------------------------    
    pertubation_conditions = create_conditions(n_cpts,alt,g,V,MAC,conditions.energy)
    pertubation_conditions.static_stability.roll_rate[:,0]  = 0
    pertubation_conditions.static_stability.pitch_rate[:,0] = delta_rate
    pertubation_conditions.static_stability.yaw_rate[:,0]   = 0
    q_hat = MAC * delta_rate / (2 * V)

    VLM_results = VLM(pertubation_conditions,settings,vehicle)
    conditions.static_stability.derivatives.Clift_q = (VLM_results.CLift        - Clift_0)        / (q_hat)
    conditions.static_stability.derivatives.Cdrag_q = (VLM_results.CDrag_induced - Cdrag_induced_0) / (q_hat)
    conditions.static_stability.derivatives.CX_q    = (VLM_results.CX           - CX_0)            / (q_hat)
    assign_moment_derivatives(conditions, 'q', VLM_results, baseline_coefficients, q_hat)

    # ---------------------------------------------------------------------------------------------------      
    # Yaw Rate (r) Purtubation
    # ---------------------------------------------------------------------------------------------------     
    pertubation_conditions =  create_conditions(n_cpts,alt,g,V,MAC,conditions.energy)
    pertubation_conditions.static_stability.roll_rate[:,0]   = 0   
    pertubation_conditions.static_stability.pitch_rate[:,0]  = 0 
    pertubation_conditions.static_stability.yaw_rate[:,0]    = delta_rate
    r_hat =  delta_rate * b / (2 * V)
    
    VLM_results = VLM(pertubation_conditions,settings,vehicle)
    conditions.static_stability.derivatives.Clift_r = (VLM_results.CLift        - Clift_0)        / (r_hat)
    conditions.static_stability.derivatives.Cdrag_r = (VLM_results.CDrag_induced - Cdrag_induced_0) / (r_hat)
    conditions.static_stability.derivatives.CX_r    = (VLM_results.CX           - CX_0)            / (r_hat)
    assign_moment_derivatives(conditions, 'r', VLM_results, baseline_coefficients, r_hat)

    # see control_surface_registry.py for why every control surface is handled through one generic path here
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

def apply_drag_corrections(state, settings, vehicle):
    """Runs the standard parasite/induced/misc drag buildup chain used throughout
    evaluate_no_surrogate() -- factored out since it's called identically (same
    functions, same order) on 4 different states there."""
    RCAIDE.Library.Methods.Aerodynamics.Common.Lift.fuselage_correction(state,settings,vehicle)
    for wing in vehicle.wings:
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

def compute_viscous_prime(pertubation_conditions, settings, vehicle):
    """Builds a State/Segment around an already-VLM()-evaluated pertubation_conditions,
    runs orientation + apply_drag_corrections, and returns the resulting viscous/total
    (Clift, Cdrag, CX) -- the "prime" values the alpha and u-velocity perturbations in
    evaluate_no_surrogate() need (unlike beta/v/w/p/q/r, which reuse their own
    perturbation's already-induced-only VLM_results directly for every derivative, see
    assign_moment_derivatives)."""
    perturbation_state                  = RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude()
    perturbation_state.conditions       = pertubation_conditions
    perturbation_state.state.conditions = pertubation_conditions
    orientation(perturbation_state)
    orientations(perturbation_state)

    apply_drag_corrections(perturbation_state, settings, vehicle)

    T_wind2inertial  = pertubation_conditions.frames.wind.transform_to_inertial
    Cdrag_visc_prime = perturbation_state.conditions.aerodynamics.coefficients.drag.total
    Clift_visc_prime = perturbation_state.conditions.aerodynamics.coefficients.lift.total
    CX_visc_prime    = orientation_product(T_wind2inertial, Cdrag_visc_prime)[:,0][:,None]
    return Clift_visc_prime, Cdrag_visc_prime, CX_visc_prime

def assign_moment_derivatives(conditions, suffix, VLM_results, baseline_coefficients, denom, negate_CN=False):
    """Computes and stores the CY/CZ/CL/CM/CN stability derivatives for one perturbation
    (suffix e.g. 'beta', 'p', 'u') -- the part of every perturbation block in
    evaluate_no_surrogate() that's identical regardless of perturbation type: always the
    perturbation's own (induced-only) VLM_results against the induced-only baseline.
    Clift/Cdrag/CX aren't included here since whether their baseline is viscous or
    induced-only differs by perturbation type -- each call site assigns those 3 directly."""
    d = conditions.static_stability.derivatives
    d['CY_' + suffix] = (VLM_results.CY - baseline_coefficients['CY']) / denom
    d['CZ_' + suffix] = (VLM_results.CZ - baseline_coefficients['CZ']) / denom
    d['CL_' + suffix] = (VLM_results.CL - baseline_coefficients['CL']) / denom
    d['CM_' + suffix] = (VLM_results.CM - baseline_coefficients['CM']) / denom
    cn = (VLM_results.CN - baseline_coefficients['CN']) / denom
    d['CN_' + suffix] = -cn if negate_CN else cn

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


