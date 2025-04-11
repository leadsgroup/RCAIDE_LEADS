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
    
    surrogates.Clift_wing_alpha = Data()
    surrogates.Cdrag_wing_alpha = Data() 
    for wing in  vehicle.wings: 
        surrogates.Clift_wing_alpha[wing.tag] = RegularGridInterpolator((AoA_data ,mach_data),training.Clift_wing_alpha[wing.tag],method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.Cdrag_wing_alpha[wing.tag] = RegularGridInterpolator((AoA_data ,mach_data),training.Cdrag_wing_alpha[wing.tag],method = 'linear',   bounds_error=False, fill_value=None) 
     
    # Pack the outputs
    surrogates.CM_alpha       = interpolate.interp1d(mach_data,training.CM_alpha       ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.CY_beta        = interpolate.interp1d(mach_data,training.CY_beta        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.CL_beta        = interpolate.interp1d(mach_data,training.CL_beta        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.CN_beta        = interpolate.interp1d(mach_data,training.CN_beta        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      


    surrogates.dCX_dalpha       = interpolate.interp1d(mach_data,training.dCX_dalpha       ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCX_du           = interpolate.interp1d(mach_data,training.dCX_du           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    
    surrogates.dCY_dbeta        = interpolate.interp1d(mach_data,training.dCY_dbeta        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")  
    surrogates.dCY_dr           = interpolate.interp1d(mach_data,training.dCY_dr           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    
    surrogates.dCZ_dalpha       = interpolate.interp1d(mach_data,training.dCZ_dalpha       ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCZ_du           = interpolate.interp1d(mach_data,training.dCZ_du           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCZ_dq           = interpolate.interp1d(mach_data,training.dCZ_dq           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    
    surrogates.dCL_dbeta        = interpolate.interp1d(mach_data,training.dCL_dbeta        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")  
    surrogates.dCL_dp           = interpolate.interp1d(mach_data,training.dCL_dp           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCL_dr           = interpolate.interp1d(mach_data,training.dCL_dr           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    
    surrogates.dCM_dalpha       = interpolate.interp1d(mach_data,training.dCM_dalpha       ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCM_du           = interpolate.interp1d(mach_data,training.dCM_du           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCM_dq           = interpolate.interp1d(mach_data,training.dCM_dq           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    
    surrogates.dCN_dbeta        = interpolate.interp1d(mach_data,training.dCN_dbeta        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")  
    surrogates.dCN_dp           = interpolate.interp1d(mach_data,training.dCN_dp           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCN_dr           = interpolate.interp1d(mach_data,training.dCN_dr           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")
   

    if aerodynamics.aileron_flag: 
        surrogates.dCY_ddelta_a     = interpolate.interp1d(mach_data,training.dCY_ddelta_a        , kind = 'linear',   bounds_error=False, fill_value="extrapolate")  
        surrogates.dCL_ddelta_a     = interpolate.interp1d(mach_data,training.dCL_ddelta_a        , kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCN_ddelta_a     = interpolate.interp1d(mach_data,training.dCN_ddelta_a        , kind = 'linear',   bounds_error=False, fill_value="extrapolate")             
    
    if aerodynamics.elevator_flag: 
        surrogates.dCM_ddelta_e     = interpolate.interp1d(mach_data,training.dCM_ddelta_e     ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate") 
        
    
    if aerodynamics.rudder_flag: 
        surrogates.dCY_ddelta_r     = interpolate.interp1d(mach_data,training.dCY_ddelta_r       ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")   
        surrogates.dCL_ddelta_r     = interpolate.interp1d(mach_data,training.dCL_ddelta_r       ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")  
        surrogates.dCN_ddelta_r     = interpolate.interp1d(mach_data,training.dCN_ddelta_r       ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")    
    
    if aerodynamics.flap_flag:
        surrogates.dCM_ddelta_f     = interpolate.interp1d(mach_data,training.dCM_ddelta_f     ,kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
    
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
    surrogates.Clift_u        = None     
    surrogates.Clift_v        = None     
    surrogates.Clift_w        = None     
    surrogates.Clift_p        = None     
    surrogates.Clift_q        = None     
    surrogates.Clift_r        = None    
    surrogates.Cdrag_alpha    = None     
    surrogates.Cdrag_beta     = None
    surrogates.Cdrag_u        = None    
    surrogates.Cdrag_v        = None    
    surrogates.Cdrag_w        = None    
    surrogates.Cdrag_p        = None    
    surrogates.Cdrag_q        = None    
    surrogates.Cdrag_r        = None 
    surrogates.CX_alpha       = None    
    surrogates.CX_beta        = None
    surrogates.CX_u           = None    
    surrogates.CX_v           = None    
    surrogates.CX_w           = None    
    surrogates.CX_p           = None    
    surrogates.CX_q           = None    
    surrogates.CX_r           = None  
    surrogates.CY_alpha       = None    
    surrogates.CY_beta        = None
    surrogates.CY_u           = None    
    surrogates.CY_v           = None    
    surrogates.CY_w           = None    
    surrogates.CY_p           = None    
    surrogates.CY_q           = None    
    surrogates.CY_r           = None   
    surrogates.CZ_alpha       = None    
    surrogates.CZ_beta        = None
    surrogates.CZ_u           = None    
    surrogates.CZ_v           = None    
    surrogates.CZ_w           = None    
    surrogates.CZ_p           = None    
    surrogates.CZ_q           = None    
    surrogates.CZ_r           = None
    surrogates.CL_alpha       = None    
    surrogates.CL_beta        = None
    surrogates.CL_u           = None    
    surrogates.CL_v           = None    
    surrogates.CL_w           = None    
    surrogates.CL_p           = None    
    surrogates.CL_q           = None    
    surrogates.CL_r           = None  
    surrogates.CM_alpha       = None    
    surrogates.CM_beta        = None
    surrogates.CM_u           = None    
    surrogates.CM_v           = None    
    surrogates.CM_w           = None    
    surrogates.CM_p           = None    
    surrogates.CM_q           = None    
    surrogates.CM_r           = None  
    surrogates.CN_alpha       = None    
    surrogates.CN_beta        = None
    surrogates.CN_u           = None    
    surrogates.CN_v           = None    
    surrogates.CN_w           = None    
    surrogates.CN_p           = None    
    surrogates.CN_q           = None    
    surrogates.CN_r           = None
    
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
        surrogates.Clift_delta_a    = None
        surrogates.Cdrag_delta_a    = None
        surrogates.CX_delta_a       = None
        surrogates.CY_delta_a       = None
        surrogates.CZ_delta_a       = None
        surrogates.CL_delta_a       = None
        surrogates.CM_delta_a       = None
        surrogates.CN_delta_a       = None
        surrogates.dClift_ddelta_a  = None
        surrogates.dCdrag_ddelta_a  = None
        surrogates.dCX_ddelta_a     = None
        surrogates.dCY_ddelta_a     = None
        surrogates.dCZ_ddelta_a     = None
        surrogates.dCL_ddelta_a     = None
        surrogates.dCM_ddelta_a     = None
        surrogates.dCN_ddelta_a     = None        
    
    if aerodynamics.elevator_flag: 
        surrogates.Clift_delta_e    = None
        surrogates.Cdrag_delta_e    = None
        surrogates.CX_delta_e       = None
        surrogates.CY_delta_e       = None
        surrogates.CZ_delta_e       = None
        surrogates.CL_delta_e       = None
        surrogates.CM_delta_e       = None
        surrogates.CN_delta_e       = None
        surrogates.dClift_ddelta_e  = None
        surrogates.dCdrag_ddelta_e  = None
        surrogates.dCX_ddelta_e     = None
        surrogates.dCY_ddelta_e     = None
        surrogates.dCZ_ddelta_e     = None
        surrogates.dCL_ddelta_e     = None
        surrogates.dCM_ddelta_e     = None
        surrogates.dCN_ddelta_e     = None
    
    
    if aerodynamics.rudder_flag: 
        surrogates.Clift_delta_r    = None  
        surrogates.Cdrag_delta_r    = None
        surrogates.CX_delta_r       = None
        surrogates.CY_delta_r       = None
        surrogates.CZ_delta_r       = None
        surrogates.CL_delta_r       = None
        surrogates.CM_delta_r       = None
        surrogates.CN_delta_r       = None
        surrogates.dClift_ddelta_r  = None
        surrogates.dCdrag_ddelta_r  = None
        surrogates.dCX_ddelta_r     = None
        surrogates.dCY_ddelta_r     = None
        surrogates.dCZ_ddelta_r     = None
        surrogates.dCL_ddelta_r     = None
        surrogates.dCM_ddelta_r     = None
        surrogates.dCN_ddelta_r     = None
    
    if aerodynamics.flap_flag:
        surrogates.Clift_delta_f    = None
        surrogates.Cdrag_delta_f    = None
        surrogates.CX_delta_f       = None
        surrogates.CY_delta_f       = None
        surrogates.CZ_delta_f       = None
        surrogates.CL_delta_f       = None
        surrogates.CM_delta_f       = None
        surrogates.CN_delta_f       = None
        surrogates.dClift_ddelta_f  = None
        surrogates.dCdrag_ddelta_f  = None
        surrogates.dCX_ddelta_f     = None
        surrogates.dCY_ddelta_f     = None
        surrogates.dCZ_ddelta_f     = None
        surrogates.dCL_ddelta_f     = None
        surrogates.dCM_ddelta_f     = None
        surrogates.dCN_ddelta_f     = None
    
    if aerodynamics.slat_flag: 
        surrogates.Clift_delta_s    = None       
        surrogates.Cdrag_delta_s    = None
        surrogates.CX_delta_s       = None
        surrogates.CY_delta_s       = None
        surrogates.CZ_delta_s       = None
        surrogates.CL_delta_s       = None
        surrogates.CM_delta_s       = None
        surrogates.CN_delta_s       = None
        surrogates.dClift_ddelta_s  = None
        surrogates.dCdrag_ddelta_s  = None
        surrogates.dCX_ddelta_s     = None
        surrogates.dCY_ddelta_s     = None
        surrogates.dCZ_ddelta_s     = None
        surrogates.dCL_ddelta_s     = None
        surrogates.dCM_ddelta_s     = None
        surrogates.dCN_ddelta_s     = None
   
    return surrogates 