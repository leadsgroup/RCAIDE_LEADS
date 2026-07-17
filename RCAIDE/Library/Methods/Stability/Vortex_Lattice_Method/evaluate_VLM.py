# RCAIDE/Library/Methods/Stability/Vortex_Lattice_Method/evaluate_VLM.py
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

import numpy as np

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
    # update static margin 
    c_ref                                                = vehicle.reference_chord   
    state.conditions.static_stability.static_margin      = np.atleast_2d((vehicle.neutral_point  - state.conditions.weights.vehicle.global_center_of_gravity[:,0]) / c_ref).T
    state.conditions.static_stability.neutral_point[:,0] = vehicle.neutral_point  
    return  