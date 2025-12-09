# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/evaluate_AVL_surrogate.py
#  
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports   
from RCAIDE.Framework.Core import  Data   
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
    lift_model          = aerodynamics.surrogates.lift_coefficient            
    drag_model          = aerodynamics.surrogates.drag_coefficient            
    e_model             = aerodynamics.surrogates.span_efficiency_factor      
    moment_model        = aerodynamics.surrogates.moment_coefficient          
    Cm_alpha_model      = aerodynamics.surrogates.Cm_alpha_moment_coefficient 
    Cn_beta_model       = aerodynamics.surrogates.Cn_beta_moment_coefficient       
    neutral_point_model = aerodynamics.surrogates.neutral_point               
    cg                  = vehicle.mass_properties.center_of_gravity[0]
    MAC                 = vehicle.wings.main_wing.chords.mean_aerodynamic
  
    pts   = np.hstack((AoA,Mach))     
    conditions.aerodynamics.coefficients.lift.inviscid.total          = np.atleast_2d(lift_model(pts)).T  
    conditions.aerodynamics.coefficients.drag.induced.inviscid        = np.atleast_2d(drag_model(pts)).T  
    conditions.aerodynamics.span_efficiency                           = np.atleast_2d(e_model(pts)).T  
    conditions.control_surfaces.slat.static_stability.coefficients.M  = np.atleast_2d(moment_model(pts)).T  
    conditions.static_stability.derivatives.CM_alpha                  = np.atleast_2d(Cm_alpha_model(pts)).T  
    conditions.static_stability.derivatives.CN_beta                   = np.atleast_2d(Cn_beta_model(pts)).T  
    conditions.static_stability.neutral_point                         = np.atleast_2d(neutral_point_model(pts)).T    
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
    run_AVL_analysis(aerodynamics,conditions, vehicle)
                       
    return

 
 