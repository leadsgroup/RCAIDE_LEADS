# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/build_VLM_surrogates.py
#  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core import  Data
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.control_surface_registry import CONTROL_SURFACE_TYPES

# package imports
from scipy.interpolate   import RegularGridInterpolator
from scipy import interpolate

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ----------------------------------------------------------------------------------------------------------------------   
def build_VLM_surrogates(aerodynamics, vehicle,aerostructural_analyses=None):
    """
    Build surrogate models for aerodynamic coefficients using VLM analysis results.
    
    This function creates interpolation-based surrogate models for various aerodynamic 
    coefficients across different flight regimes (subsonic, transonic, supersonic).
    
    Parameters
    ----------
    aerodynamics : Data
        VLM analysis data structure containing training data and vehicle information
            - training : Data
                Training data with aerodynamic coefficients at different conditions
            - vehicle : Data
                Vehicle configuration data
            - surrogates : Data
                Container to store the created surrogate models
    
    Returns
    -------
    None
        Results are stored in the aerodynamics.surrogates data structure
    
    Notes
    -----
    The function creates separate surrogate models for subsonic, transonic, and 
    supersonic regimes. For supersonic and transonic regimes, surrogates are only 
    built if sufficient data points are available (more than 2 Mach points).
    
    The surrogate models use interpolation to predict aerodynamic coefficients 
    at arbitrary flight conditions within the training data range.
    
    **Theory**
    
    The function uses regular grid interpolation for 2D data (e.g., coefficient vs 
    angle of attack and Mach number) and 1D interpolation for stability derivatives.
    
    **Related Functions:**
    
    build_surrogate : Creates individual surrogate models for a specific flight regime
    no_surrogate : Creates placeholder surrogate structures when data is insufficient
    """
    surrogates = aerodynamics.surrogates
    training   = aerodynamics.training 
    Mach       = aerodynamics.training.Mach 
    sub_len    = int(sum(Mach<1.))  
    sup_Mach   = Mach[sub_len:]
    
    surrogates.subsonic    =  build_surrogate(aerodynamics, training.subsonic, vehicle)
    
    # only build supersonic surrogates if necessary
    if len(sup_Mach) > 2: 
        surrogates.supersonic  =  build_surrogate(aerodynamics, training.supersonic, vehicle)
        surrogates.transonic   =  build_surrogate(aerodynamics, training.transonic, vehicle)
    else: 
        surrogates.supersonic  =  no_surrogate(aerodynamics, training.supersonic, vehicle)
        surrogates.transonic   =  no_surrogate(aerodynamics, training.transonic, vehicle)        
        
    return

def build_surrogate(aerodynamics, training, vehicle):
    
    # unpack data
    surrogates     = Data()
    mach_data      = training.Mach
    AoA_data       = aerodynamics.training.angle_of_attack     
    Beta_data      = aerodynamics.training.sideslip_angle
    
    # Pack the outputs
    surrogates.Clift_alpha        = RegularGridInterpolator((AoA_data ,mach_data),training.Clift_alpha        ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Cdrag_induced_alpha= RegularGridInterpolator((AoA_data ,mach_data),training.Cdrag_induced_alpha,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CM_alpha           = RegularGridInterpolator((AoA_data ,mach_data),training.CM_alpha           ,method = 'linear',   bounds_error=False, fill_value=None)  
    surrogates.CX_alpha           = RegularGridInterpolator((AoA_data ,mach_data),training.CX_alpha           ,method = 'linear',   bounds_error=False, fill_value=None)  
    surrogates.CZ_alpha           = RegularGridInterpolator((AoA_data ,mach_data),training.CZ_alpha           ,method = 'linear',   bounds_error=False, fill_value=None)
    surrogates.CY_alpha           = RegularGridInterpolator((AoA_data ,mach_data),training.CY_alpha           ,method = 'linear',   bounds_error=False, fill_value=None)
    surrogates.CL_alpha           = RegularGridInterpolator((AoA_data ,mach_data),training.CL_alpha           ,method = 'linear',   bounds_error=False, fill_value=None)
    surrogates.CN_alpha           = RegularGridInterpolator((AoA_data ,mach_data),training.CN_alpha           ,method = 'linear',   bounds_error=False, fill_value=None)  
    surrogates.Clift_spanwise     = RegularGridInterpolator((AoA_data, mach_data),training.Clift_spanwise      ,method='linear',    bounds_error=False, fill_value=None)      

    surrogates.Clift_beta         = RegularGridInterpolator((Beta_data ,mach_data),training.Clift_beta        ,method = 'linear',   bounds_error=False, fill_value=None)   
    surrogates.Cdrag_induced_beta = RegularGridInterpolator((Beta_data ,mach_data),training.Cdrag_induced_beta,method = 'linear',   bounds_error=False, fill_value=None)    
    surrogates.CX_beta            = RegularGridInterpolator((Beta_data ,mach_data),training.CX_beta           ,method = 'linear',   bounds_error=False, fill_value=None)    
    surrogates.CZ_beta            = RegularGridInterpolator((Beta_data ,mach_data),training.CZ_beta           ,method = 'linear',   bounds_error=False, fill_value=None)    
    surrogates.CY_beta            = RegularGridInterpolator((Beta_data ,mach_data),training.CY_beta           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CL_beta            = RegularGridInterpolator((Beta_data ,mach_data),training.CL_beta           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CN_beta            = RegularGridInterpolator((Beta_data ,mach_data),training.CN_beta           ,method = 'linear',   bounds_error=False, fill_value=None)  
    surrogates.CM_beta            = RegularGridInterpolator((Beta_data ,mach_data),training.CM_beta           ,method = 'linear',   bounds_error=False, fill_value=None) 

    # Use interpolat.interp1d below
    surrogates.CM_0             = interpolate.interp1d(mach_data, training.CM_0, kind='linear', bounds_error=False, fill_value='extrapolate')         
    surrogates.dClift_dalpha    = interpolate.interp1d(mach_data, training.dClift_dalpha, kind='linear', bounds_error=False, fill_value='extrapolate')      
    surrogates.dCX_dalpha       = interpolate.interp1d(mach_data, training.dCX_dalpha, kind='linear', bounds_error=False, fill_value='extrapolate')      
    surrogates.dCX_du           = interpolate.interp1d(mach_data, training.dCX_du, kind='linear', bounds_error=False, fill_value='extrapolate')      
    
    surrogates.dCY_dbeta        = interpolate.interp1d(mach_data,training.dCY_dbeta, kind='linear', bounds_error=False, fill_value='extrapolate')  
    surrogates.dCY_dr           = interpolate.interp1d(mach_data,training.dCY_dr, kind='linear', bounds_error=False, fill_value='extrapolate')      
    
    surrogates.dCZ_dalpha       = interpolate.interp1d(mach_data,training.dCZ_dalpha, kind='linear', bounds_error=False, fill_value='extrapolate')      
    surrogates.dCZ_du           = interpolate.interp1d(mach_data,training.dCZ_du, kind='linear', bounds_error=False, fill_value='extrapolate')      
    surrogates.dCZ_dq           = interpolate.interp1d(mach_data,training.dCZ_dq, kind='linear', bounds_error=False, fill_value='extrapolate')      
    
    surrogates.dCL_dbeta        = interpolate.interp1d(mach_data,training.dCL_dbeta, kind='linear', bounds_error=False, fill_value='extrapolate')  
    surrogates.dCL_dp           = interpolate.interp1d(mach_data,training.dCL_dp, kind='linear', bounds_error=False, fill_value='extrapolate')      
    surrogates.dCL_dr           = interpolate.interp1d(mach_data,training.dCL_dr, kind='linear', bounds_error=False, fill_value='extrapolate')      
    
    surrogates.dCM_dalpha       = interpolate.interp1d(mach_data,training.dCM_dalpha, kind='linear', bounds_error=False, fill_value='extrapolate')      
    surrogates.dCM_du           = interpolate.interp1d(mach_data,training.dCM_du, kind='linear', bounds_error=False, fill_value='extrapolate')      
    surrogates.dCM_dq           = interpolate.interp1d(mach_data,training.dCM_dq, kind='linear', bounds_error=False, fill_value='extrapolate')      
    
    surrogates.dCN_dbeta        = interpolate.interp1d(mach_data,training.dCN_dbeta, kind='linear', bounds_error=False, fill_value='extrapolate')  
    surrogates.dCN_dp           = interpolate.interp1d(mach_data,training.dCN_dp, kind='linear', bounds_error=False, fill_value='extrapolate')      
    surrogates.dCN_dr           = interpolate.interp1d(mach_data,training.dCN_dr, kind='linear', bounds_error=False, fill_value='extrapolate')      
   

    # see control_surface_registry.py for why every control surface is handled through one generic path here
    for cls, letter, name, channel, flag, deflection_attr in CONTROL_SURFACE_TYPES:
        if getattr(aerodynamics, flag):
            for coeff in ('Clift', 'Cdrag', 'CX', 'CY', 'CZ', 'CL', 'CM', 'CN'):
                key = 'd' + coeff + '_ddelta_' + letter
                surrogates[key] = interpolate.interp1d(mach_data, training[key], kind='linear', bounds_error=False, fill_value='extrapolate')

    return surrogates
 
 
def no_surrogate(aerodynamics, training, vehicle):
    
    # unpack data
    surrogates     = Data()    
     
    # Pack the outputs     
    surrogates.Clift_alpha            = None     
    surrogates.Clift_beta             = None
    surrogates.Clift_spanwise         = None
    surrogates.Cdrag_induced_alpha    = None     
    surrogates.Cdrag_induced_beta     = None 
    surrogates.CX_alpha               = None    
    surrogates.CX_beta                = None 
    surrogates.CY_alpha               = None    
    surrogates.CY_beta                = None  
    surrogates.CZ_alpha               = None    
    surrogates.CZ_beta                = None 
    surrogates.CL_alpha               = None    
    surrogates.CL_beta                = None 
    surrogates.CM_alpha               = None    
    surrogates.CM_beta                = None 
    surrogates.CN_alpha               = None    
    surrogates.CN_beta                = None    
    surrogates.CM_0                   = None  
    
    surrogates.dClift_dalpha          = None      
    surrogates.dCX_dalpha             = None      
    surrogates.dCX_du                 = None      
    surrogates.dCY_dbeta              = None  
    surrogates.dCY_dr                 = None  
    surrogates.dCZ_dalpha             = None      
    surrogates.dCZ_du                 = None      
    surrogates.dCZ_dq                 = None      
    surrogates.dCL_dbeta              = None  
    surrogates.dCL_dp                 = None      
    surrogates.dCL_dr                 = None  
    surrogates.dCM_dalpha             = None      
    surrogates.dCM_du                 = None      
    surrogates.dCM_dq                 = None      
    surrogates.dCN_dbeta              = None  
    surrogates.dCN_dp                 = None      
    surrogates.dCN_dr                 = None 

    for cls, letter, name, channel, flag, deflection_attr in CONTROL_SURFACE_TYPES:
        if getattr(aerodynamics, flag):
            for coeff in ('Clift', 'Cdrag', 'CX', 'CY', 'CZ', 'CL', 'CM', 'CN'):
                surrogates['d' + coeff + '_ddelta_' + letter] = None

    return surrogates