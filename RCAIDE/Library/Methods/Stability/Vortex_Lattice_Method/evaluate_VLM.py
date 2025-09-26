# RCAIDE/Library/Methods/Stability/Vortex_Lattice_Method/evaluate_VLM.py
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports   
from RCAIDE.Framework.Core     import Data    
from RCAIDE.Library.Methods.Stability.Common.update_center_of_gravity import update_center_of_gravity

# package imports
import numpy   as np

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ---------------------------------------------------------------------------------------------------------------------- 
def evaluate(state,settings,vehicle):
    """Evaluates static margin and neutral point using built surrogates 
    
    Assumptions: 
        
    Source:
        None

    Args:
        stability    : VLM analysis  [unitless]
        state        : flight conditions     [unitless]
        settings     : VLM analysis settings [unitless]
        vehicle      : vehicle configuration [unitless] 
        
    Returns: 
        None  
    """ 
   
    # --------------------------------------------------------------------------
    # unpack 
    # --------------------------------------------------------------------------
    conditions    = state.conditions 
    AoA           = conditions.aerodynamics.angles.alpha  
    
    if settings.update_center_of_gravity:
        CG = update_center_of_gravity(vehicle, conditions)
    else:
        CG = np.ones_like(AoA) * vehicle.mass_properties.center_of_gravity[0][0] 
 
    # --------------------------------------------------------------------------------------------      
    # Vehicle Properties 
    # --------------------------------------------------------------------------------------------      
    c_ref         = vehicle.reference_chord   
    NP            = vehicle.neutral_point
    
    # --------------------------------------------------------------------------------------------      
    # Store Results 
    # --------------------------------------------------------------------------------------------      
    conditions.static_stability.center_of_gravity      = CG       
    conditions.static_stability.neutral_point[:,0]     = NP 
    conditions.static_stability.static_margin          = (NP - CG) / c_ref      
        
    return  