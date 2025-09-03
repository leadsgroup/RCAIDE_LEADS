# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/build_SU2_surrogates.py
#  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core import  Data 

# package imports 
from scipy.interpolate    import RegularGridInterpolator 
from scipy import interpolate

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ----------------------------------------------------------------------------------------------------------------------   
def build_SU2_surrogates(aerodynamics):
    """Build a surrogate using sample evaluation results.
    
    Assumptions:
        None
        
    Source:
        None

    Args:
        aerodynamics       : SU2 analysis          [unitless] 
        
    Returns: 
        None  
    """
    surrogates = aerodynamics.surrogates
    training   = aerodynamics.training  
    vehicle    = aerodynamics.vehicle  
    
    # unpack data
    surrogates     = Data()
    mach_data      = training.Mach 
    AoA_data       = aerodynamics.training.angle_of_attack
    
    # coefficients  
    surrogates.Clift_alpha       = RegularGridInterpolator((AoA_data,mach_data),training.Clift_alpha  ,method = 'linear',   bounds_error=False, fill_value=None)     
    surrogates.Cdrag_alpha       = RegularGridInterpolator((AoA_data,mach_data),training.Cdrag_alpha  ,method = 'linear',   bounds_error=False, fill_value=None)     
    surrogates.CM_alpha          = RegularGridInterpolator((AoA_data ,mach_data),training.CM_alpha    ,method = 'linear',   bounds_error=False, fill_value=None)

    surrogates.Clift_wing_alpha = Data()
    surrogates.Cdrag_induced_wing_alpha = Data() 
    for wing in  vehicle.wings: 
        surrogates.Clift_wing_alpha[wing.tag] = RegularGridInterpolator((AoA_data ,mach_data),training.Clift_wing_alpha[wing.tag],method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.Cdrag_induced_wing_alpha[wing.tag] = RegularGridInterpolator((AoA_data ,mach_data),training.Cdrag_induced_wing_alpha[wing.tag],method = 'linear',   bounds_error=False, fill_value=None) 
     
    # stability derivatives 
    surrogates.dClift_dalpha    = interpolate.interp1d(mach_data, training.dClift_dalpha, kind='linear', bounds_error=False, fill_value='extrapolate')     
    surrogates.dCM_dalpha       = interpolate.interp1d(mach_data,training.dCM_dalpha, kind='linear', bounds_error=False, fill_value='extrapolate')         
      
    aerodynamics.surrogates = surrogates
    return #surrogates
 
 
