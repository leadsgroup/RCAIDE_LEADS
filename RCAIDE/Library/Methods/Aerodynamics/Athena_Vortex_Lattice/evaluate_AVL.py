# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/evaluate_AVL_surrogate.py
#  
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports   
from RCAIDE.Framework.Core import  Data, Units   
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.run_AVL_analysis  import run_AVL_analysis  

# package imports
import numpy   as np  

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ---------------------------------------------------------------------------------------------------------------------- 
def evaluate_AVL_surrogate(state,settings,vehicle):
    """Evaluates surrogates forces and moments using built surrogates 
    
    Assumptions:
        None
        
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
    conditions          = state.conditions
    aerodynamics        = state.analyses.aerodynamics  
    Mach                = conditions.freestream.mach_number
    AoA                 = conditions.aerodynamics.angles.alpha
    lift_model          = aerodynamics.surrogates.Clift_alpha  
    lift_y_model        = aerodynamics.surrogates.Clift_spanwise           
    drag_model          = aerodynamics.surrogates.Cdrag_induced_alpha            
    moment_model        = aerodynamics.surrogates.CM_alpha
    e_model             = aerodynamics.surrogates.span_efficincy       
    Cm_alpha_model      = aerodynamics.surrogates.dCM_dalpha 
    Cn_beta_model       = aerodynamics.surrogates.dCN_dbeta       
    neutral_point_model = aerodynamics.surrogates.neutral_point               
    cg                  = vehicle.mass_properties.center_of_gravity[0]
    MAC                 = vehicle.wings.main_wing.chords.mean_aerodynamic
  
    pts   = np.hstack((AoA,Mach))     
    conditions.aerodynamics.coefficients.lift.inviscid.total          = np.atleast_2d(lift_model(pts)).T  
    conditions.aerodynamics.coefficients.drag.induced.inviscid        = np.atleast_2d(drag_model(pts)).T  
    conditions.aerodynamics.span_efficiency                           = np.atleast_2d(e_model(pts)).T   
    conditions.static_stability.derivatives.CM_alpha                  = np.atleast_2d(Cm_alpha_model(pts)).T  
    conditions.static_stability.derivatives.CN_beta                   = np.atleast_2d(Cn_beta_model(pts)).T  
    conditions.static_stability.neutral_point                         = np.atleast_2d(neutral_point_model(pts)).T  
    conditions.aerodynamics.coefficients.lift.spanwise                = np.atleast_2d(lift_y_model(pts))       
    conditions.static_stability.coefficients.M                        = np.atleast_2d(moment_model(pts)).T
    
    # control surface increments, linearized about the training deflection (AVL derivatives are per degree)
    letters = {'flap':'f', 'slat':'s', 'aileron':'a', 'elevator':'e', 'rudder':'r'}
    for cs in set(settings.control_surface_tags):
        if cs not in conditions.control_surfaces:
            continue
        letter     = letters[cs]
        deflection = (conditions.control_surfaces[cs].deflection - aerodynamics.training_deflections[cs]) / Units.degrees
        CM_delta   = np.atleast_2d(aerodynamics.surrogates['dCM_ddelta_' + letter](pts)).T
        lift_delta = np.atleast_2d(aerodynamics.surrogates['dClift_ddelta_' + letter](pts)).T
        drag_delta = np.abs(np.atleast_2d(aerodynamics.surrogates['dCdrag_induced_ddelta_' + letter](pts)).T)
        conditions.static_stability.coefficients.M                 += CM_delta   * deflection
        conditions.aerodynamics.coefficients.lift.inviscid.total   += lift_delta * deflection
        conditions.aerodynamics.coefficients.drag.induced.inviscid += drag_delta * np.abs(deflection)
    
    conditions.static_stability.static_margin                         = (conditions.static_stability.neutral_point - cg)/MAC     
    aerodynamics.settings.span_efficiency                             = conditions.aerodynamics.span_efficiency   
    return


def evaluate_AVL_no_surrogate(state,settings,vehicle):
    """Evaluates forces and moments directly using VLM.
    
    Assumptions:
        None
        
    Source:
        None

    Args:
        aerodynamics    : AVL analysis  [unitless]
        state           : flight conditions     [unitless] 
        vehicle         : vehicle configuration [unitless] 
        
    Returns: 
        None  
    """          

    # unpack 
    conditions     = state.conditions
    aerodynamics   = state.analyses.aerodynamics   
    V              = conditions.freestream.velocity
    b_ref          = vehicle.wings.main_wing.spans.projected
    c_ref          = vehicle.wings.main_wing.chords.mean_aerodynamic

    # AVL inputs: run at the segment's lift coefficient only when one is prescribed, otherwise at alpha
    if getattr(state, 'lift_coefficient', None) is None:
        conditions.aerodynamics.coefficients.lift.inviscid.total = None
    conditions.static_stability.coefficients.roll  = conditions.static_stability.roll_rate * b_ref / (2 * V)
    conditions.static_stability.coefficients.pitch = conditions.static_stability.pitch_rate * c_ref / (2 * V)
    
    run_AVL_analysis(aerodynamics,conditions, vehicle)
                       
    return

 
 