# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/build_VLM_surrogates.py
#  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core import  Data 

# package imports 
from scipy.interpolate                                           import RegularGridInterpolator
from scipy import interpolate

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ----------------------------------------------------------------------------------------------------------------------   
def build_VLM_surrogates(aerodynamics):
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
    surrogates = aerodynamics.surrogates
    training   = aerodynamics.training 
    Mach       = aerodynamics.training.Mach 
    training   = aerodynamics.training  
    sub_len    = int(sum(Mach<1.))  
    sup_Mach   = Mach[sub_len:]
    
    surrogates.subsonic    =  build_surrogate(aerodynamics, training.subsonic)
    
    # only build supersonic surrogates if necessary
    if len(sup_Mach) > 2: 
        surrogates.supersonic  =  build_surrogate(aerodynamics, training.supersonic)
        surrogates.transonic   =  build_surrogate(aerodynamics, training.transonic)
    else: 
        surrogates.supersonic  =  no_surrogate(aerodynamics, training.supersonic)
        surrogates.transonic   =  no_surrogate(aerodynamics, training.transonic)        
        
    return

def build_surrogate(aerodynamics, training):
    
    # unpack data
    surrogates     = Data()
    mach_data      = training.Mach
    vehicle        = aerodynamics.vehicle
    AoA_data       = aerodynamics.training.angle_of_attack     
    Beta_data      = aerodynamics.training.sideslip_angle     
    
    surrogates.Clift_wing_alpha = Data()
    surrogates.Cdrag_wing_alpha = Data() 
    for wing in  vehicle.wings: 
        surrogates.Clift_wing_alpha[wing.tag] = RegularGridInterpolator((AoA_data ,mach_data),training.Clift_wing_alpha[wing.tag],method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.Cdrag_wing_alpha[wing.tag] = RegularGridInterpolator((AoA_data ,mach_data),training.Cdrag_wing_alpha[wing.tag],method = 'linear',   bounds_error=False, fill_value=None) 
     
    # Pack the outputs
    surrogates.Clift_alpha    = RegularGridInterpolator((AoA_data ,mach_data),training.Clift_alpha        ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Cdrag_alpha    = RegularGridInterpolator((AoA_data ,mach_data),training.Cdrag_alpha        ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CM_alpha       = RegularGridInterpolator((AoA_data ,mach_data),training.CM_alpha       ,method = 'linear',   bounds_error=False, fill_value=None)  
    surrogates.CX_alpha       = RegularGridInterpolator((AoA_data ,mach_data),training.CX_alpha       ,method = 'linear',   bounds_error=False, fill_value=None)  
    surrogates.CZ_alpha       = RegularGridInterpolator((AoA_data ,mach_data),training.CZ_alpha       ,method = 'linear',   bounds_error=False, fill_value=None)
    surrogates.CY_alpha       = RegularGridInterpolator((AoA_data ,mach_data),training.CY_alpha       ,method = 'linear',   bounds_error=False, fill_value=None)
    surrogates.CL_alpha       = RegularGridInterpolator((AoA_data ,mach_data),training.CL_alpha       ,method = 'linear',   bounds_error=False, fill_value=None)
    surrogates.CN_alpha       = RegularGridInterpolator((AoA_data ,mach_data),training.CN_alpha       ,method = 'linear',   bounds_error=False, fill_value=None)       

    surrogates.Clift_beta     = RegularGridInterpolator((Beta_data ,mach_data),training.Clift_beta        ,method = 'linear',   bounds_error=False, fill_value=None)   
    surrogates.Cdrag_beta     = RegularGridInterpolator((Beta_data ,mach_data),training.Cdrag_beta        ,method = 'linear',   bounds_error=False, fill_value=None)    
    surrogates.CX_beta        = RegularGridInterpolator((Beta_data ,mach_data),training.CX_beta        ,method = 'linear',   bounds_error=False, fill_value=None)    
    surrogates.CZ_beta        = RegularGridInterpolator((Beta_data ,mach_data),training.CZ_beta        ,method = 'linear',   bounds_error=False, fill_value=None)    
    surrogates.CY_beta        = RegularGridInterpolator((Beta_data ,mach_data),training.CY_beta        ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CL_beta        = RegularGridInterpolator((Beta_data ,mach_data),training.CL_beta        ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CN_beta        = RegularGridInterpolator((Beta_data ,mach_data),training.CN_beta        ,method = 'linear',   bounds_error=False, fill_value=None)  
    surrogates.CM_beta        = RegularGridInterpolator((Beta_data ,mach_data),training.CM_beta        ,method = 'linear',   bounds_error=False, fill_value=None) 


    # Use interpolat.interp1d below
    surrogates.CM_0             =  interpolate.interp1d(mach_data, training.CM_0, kind='linear', bounds_error=False, fill_value='extrapolate')      
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
   

    if aerodynamics.aileron_flag: 
        surrogates.dCY_ddelta_a     = interpolate.interp1d(mach_data,training.dCY_ddelta_a        , kind = 'linear',   bounds_error=False, fill_value='extrapolate')  
        surrogates.dCL_ddelta_a     = interpolate.interp1d(mach_data,training.dCL_ddelta_a        , kind = 'linear',   bounds_error=False, fill_value='extrapolate') 
        surrogates.dCN_ddelta_a     = interpolate.interp1d(mach_data,training.dCN_ddelta_a        , kind = 'linear',   bounds_error=False, fill_value='extrapolate')             
    
    if aerodynamics.elevator_flag: 
        surrogates.dCM_ddelta_e     = interpolate.interp1d(mach_data,training.dCM_ddelta_e     ,kind = 'linear',   bounds_error=False, fill_value='extrapolate') 
        surrogates.dClift_ddelta_e  = interpolate.interp1d(mach_data,training.dClift_ddelta_e  ,kind = 'linear',   bounds_error=False, fill_value='extrapolate') 
        
    
    if aerodynamics.rudder_flag: 
        surrogates.dCY_ddelta_r     = interpolate.interp1d(mach_data,training.dCY_ddelta_r       ,kind = 'linear',   bounds_error=False, fill_value='extrapolate')   
        surrogates.dCN_ddelta_r     = interpolate.interp1d(mach_data,training.dCN_ddelta_r       ,kind = 'linear',   bounds_error=False, fill_value='extrapolate')    
        surrogates.dCL_ddelta_r     = interpolate.interp1d(mach_data,training.dCL_ddelta_r       ,kind = 'linear',   bounds_error=False, fill_value='extrapolate')    

    if aerodynamics.flap_flag:
        surrogates.dCM_ddelta_f     = interpolate.interp1d(mach_data,training.dCM_ddelta_f     ,kind = 'linear',   bounds_error=False, fill_value='extrapolate') 
    
    if aerodynamics.slat_flag: 
        surrogates.dCY_ddelta_s     = interpolate.interp1d(mach_data,training.dCY_ddelta_s       ,kind = 'linear',   bounds_error=False, fill_value='extrapolate')   
        surrogates.dCL_ddelta_s     = interpolate.interp1d(mach_data,training.dCL_ddelta_s       ,kind = 'linear',   bounds_error=False, fill_value='extrapolate')  
        surrogates.dCN_ddelta_s     = interpolate.interp1d(mach_data,training.dCN_ddelta_s       ,kind = 'linear',   bounds_error=False, fill_value='extrapolate')             
    
    if aerodynamics.flap_flag:
        surrogates.dCM_ddelta_f     = interpolate.interp1d(mach_data,training.dCM_ddelta_f     ,kind = 'linear',   bounds_error=False, fill_value='extrapolate') 
    
    return surrogates
 
 
def no_surrogate(aerodynamics, training):
    
    # unpack data
    surrogates     = Data() 
    vehicle        = aerodynamics.vehicle 
    
    surrogates.Clift_wing_alpha = Data()
    surrogates.Cdrag_wing_alpha = Data() 
    for wing in  vehicle.wings: 
        surrogates.Clift_wing_alpha[wing.tag] =None 
        surrogates.Cdrag_wing_alpha[wing.tag] =None 
     
    # Pack the outputs     
    surrogates.Clift_alpha    = None     
    surrogates.Clift_beta     = None  
    surrogates.Cdrag_alpha    = None     
    surrogates.Cdrag_beta     = None 
    surrogates.CX_alpha       = None    
    surrogates.CX_beta        = None 
    surrogates.CY_alpha       = None    
    surrogates.CY_beta        = None  
    surrogates.CZ_alpha       = None    
    surrogates.CZ_beta        = None 
    surrogates.CL_alpha       = None    
    surrogates.CL_beta        = None 
    surrogates.CM_alpha       = None    
    surrogates.CM_beta        = None 
    surrogates.CN_alpha       = None    
    surrogates.CN_beta        = None    
    surrogates.CM_0           = None
    
    surrogates.dClift_dalpha    = None      
    surrogates.dCX_dalpha       = None      
    surrogates.dCX_du           = None      
    surrogates.dCY_dbeta        = None  
    surrogates.dCY_dr           = None  
    surrogates.dCZ_dalpha       = None      
    surrogates.dCZ_du           = None      
    surrogates.dCZ_dq           = None      
    surrogates.dCL_dbeta        = None  
    surrogates.dCL_dp           = None      
    surrogates.dCL_dr           = None  
    surrogates.dCM_dalpha       = None      
    surrogates.dCM_du           = None      
    surrogates.dCM_dq           = None      
    surrogates.dCN_dbeta        = None  
    surrogates.dCN_dp           = None      
    surrogates.dCN_dr           = None
   

    if aerodynamics.aileron_flag:  
        surrogates.dCY_ddelta_a     = None 
        surrogates.dCL_ddelta_a     = None 
        surrogates.dCN_ddelta_a     = None        
    
    if aerodynamics.elevator_flag: 
        surrogates.Clift_delta_e    = None 
        surrogates.dCM_ddelta_e     = None 
        surrogates.dClift_ddelta_e  = None 
    
    
    if aerodynamics.rudder_flag:  
        surrogates.dCY_ddelta_r     = None 
        surrogates.dCL_ddelta_r     = None 
        surrogates.dCN_ddelta_r     = None
    
    if aerodynamics.flap_flag:
        surrogates.Clift_delta_f    = None 
        surrogates.dCM_ddelta_f     = None  
   
    return surrogates 