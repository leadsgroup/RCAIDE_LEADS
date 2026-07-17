# RCAIDE/Library/Methods/Aerodynamics/Athena_Vortex_Lattice/build_VLM_surrogates.py
#
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# package imports 
from scipy.interpolate                                           import RegularGridInterpolator 

# ----------------------------------------------------------------------------------------------------------------------
#  build_AVL_surrogates
# ---------------------------------------------------------------------------------------------------------------------- 
def build_AVL_surrogates(aerodynamics):
    """Build a surrogate using sample evaluation results.
    
    Assumptions:
        None
        
    Source:
        None

    Args:
        aerodynamics       : VLM analysis          [unitless] 
        
    Returns: 
        None  
    """
    surrogates  = aerodynamics.surrogates
    training    = aerodynamics.training  
    AoA_data    = training.angle_of_attack
    mach_data   = training.Mach
    
    skip_list = ['angle_of_attack', 'Mach']
    for key in training.keys():
        if key in skip_list : 
            pass
        else:
            surrogates[key]  = RegularGridInterpolator((AoA_data, mach_data), training[key] ,method='linear',    bounds_error=False, fill_value=None)           
        
    return 