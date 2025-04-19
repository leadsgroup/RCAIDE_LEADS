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
    surrogates =  aerodynamics.surrogates
    training   =  aerodynamics.training 
    surrogates.subsonic    =  build_surrogate(aerodynamics, training.subsonic) 
    surrogates.supersonic  =  build_surrogate(aerodynamics, training.supersonic)
    surrogates.transonic   =  build_surrogate(aerodynamics, training.transonic)
    return

def build_surrogate(aerodynamics, training):
    
    # unpack data
    surrogates     = Data()
    mach_data      = training.Mach
    vehicle        = aerodynamics.vehicle
    AoA_data       = aerodynamics.training.angle_of_attack           
    Beta_data      = aerodynamics.training.sideslip_angle  
    u_data         = aerodynamics.training.u
    v_data         = aerodynamics.training.v
    w_data         = aerodynamics.training.w
    p_data         = aerodynamics.training.roll_rate
    q_data         = aerodynamics.training.pitch_rate
    r_data         = aerodynamics.training.yaw_rate
    aileron_data   = aerodynamics.training.aileron_deflection           
    elevator_data  = aerodynamics.training.elevator_deflection          
    rudder_data    = aerodynamics.training.rudder_deflection            
    flap_data      = aerodynamics.training.flap_deflection              
    slat_data      = aerodynamics.training.slat_deflection
    
    surrogates.Clift_wing_alpha = Data()
    surrogates.Cdrag_wing_alpha = Data() 
    for wing in  vehicle.wings: 
        surrogates.Clift_wing_alpha[wing.tag] = RegularGridInterpolator((AoA_data ,mach_data),training.Clift_wing_alpha[wing.tag],method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.Cdrag_wing_alpha[wing.tag] = RegularGridInterpolator((AoA_data ,mach_data),training.Cdrag_wing_alpha[wing.tag],method = 'linear',   bounds_error=False, fill_value=None) 
     
    # Pack the outputs     
    surrogates.Clift_alpha       = RegularGridInterpolator((AoA_data ,mach_data),training.Clift_alpha        ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Clift_beta        = RegularGridInterpolator((Beta_data,mach_data),training.Clift_beta         ,method = 'linear',   bounds_error=False, fill_value=None) 
    surrogates.Clift_u           = RegularGridInterpolator((u_data,mach_data),training.Clift_u               ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Clift_v           = RegularGridInterpolator((v_data,mach_data),training.Clift_v               ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Clift_w           = RegularGridInterpolator((w_data,mach_data),training.Clift_w               ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Clift_p           = RegularGridInterpolator((p_data,mach_data),training.Clift_p               ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Clift_q           = RegularGridInterpolator((q_data,mach_data),training.Clift_q               ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Clift_r           = RegularGridInterpolator((r_data,mach_data),training.Clift_r               ,method = 'linear',   bounds_error=False, fill_value=None)      
    
    surrogates.Cdrag_alpha       = RegularGridInterpolator((AoA_data,mach_data),training.Cdrag_alpha         ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Cdrag_beta        = RegularGridInterpolator((Beta_data,mach_data),training.Cdrag_beta         ,method = 'linear',   bounds_error=False, fill_value=None)
    surrogates.Cdrag_u        = RegularGridInterpolator((u_data,mach_data),training.Cdrag_u        ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Cdrag_v        = RegularGridInterpolator((v_data,mach_data),training.Cdrag_v        ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Cdrag_w        = RegularGridInterpolator((w_data,mach_data),training.Cdrag_w        ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Cdrag_p        = RegularGridInterpolator((p_data,mach_data),training.Cdrag_p        ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Cdrag_q        = RegularGridInterpolator((q_data,mach_data),training.Cdrag_q        ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.Cdrag_r        = RegularGridInterpolator((r_data,mach_data),training.Cdrag_r        ,method = 'linear',   bounds_error=False, fill_value=None)      
    
    surrogates.CX_alpha       = RegularGridInterpolator((AoA_data,mach_data),training.CX_alpha     ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CX_beta        = RegularGridInterpolator((Beta_data,mach_data),training.CX_beta     ,method = 'linear',   bounds_error=False, fill_value=None) 
    surrogates.CX_u           = RegularGridInterpolator((u_data,mach_data),training.CX_u           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CX_v           = RegularGridInterpolator((v_data,mach_data),training.CX_v           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CX_w           = RegularGridInterpolator((w_data,mach_data),training.CX_w           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CX_p           = RegularGridInterpolator((p_data,mach_data),training.CX_p           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CX_q           = RegularGridInterpolator((q_data,mach_data),training.CX_q           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CX_r           = RegularGridInterpolator((r_data,mach_data),training.CX_r           ,method = 'linear',   bounds_error=False, fill_value=None)      
    
    surrogates.CY_alpha       = RegularGridInterpolator((AoA_data,mach_data),training.CY_alpha     ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CY_beta        = RegularGridInterpolator((Beta_data,mach_data),training.CY_beta     ,method = 'linear',   bounds_error=False, fill_value=None)
    surrogates.CY_u           = RegularGridInterpolator((u_data,mach_data),training.CY_u           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CY_v           = RegularGridInterpolator((v_data,mach_data),training.CY_v           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CY_w           = RegularGridInterpolator((w_data,mach_data),training.CY_w           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CY_p           = RegularGridInterpolator((p_data,mach_data),training.CY_p           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CY_q           = RegularGridInterpolator((q_data,mach_data),training.CY_q           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CY_r           = RegularGridInterpolator((r_data,mach_data),training.CY_r           ,method = 'linear',   bounds_error=False, fill_value=None)      
    
    surrogates.CZ_alpha       = RegularGridInterpolator((AoA_data,mach_data),training.CZ_alpha     ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CZ_beta        = RegularGridInterpolator((Beta_data,mach_data),training.CZ_beta     ,method = 'linear',   bounds_error=False, fill_value=None)
    surrogates.CZ_u           = RegularGridInterpolator((u_data,mach_data),training.CZ_u           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CZ_v           = RegularGridInterpolator((v_data,mach_data),training.CZ_v           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CZ_w           = RegularGridInterpolator((w_data,mach_data),training.CZ_w           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CZ_p           = RegularGridInterpolator((p_data,mach_data),training.CZ_p           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CZ_q           = RegularGridInterpolator((q_data,mach_data),training.CZ_q           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CZ_r           = RegularGridInterpolator((r_data,mach_data),training.CZ_r           ,method = 'linear',   bounds_error=False, fill_value=None)      
    
    surrogates.CL_alpha       = RegularGridInterpolator((AoA_data,mach_data),training.CL_alpha     ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CL_beta        = RegularGridInterpolator((Beta_data,mach_data),training.CL_beta     ,method = 'linear',   bounds_error=False, fill_value=None)
    surrogates.CL_u           = RegularGridInterpolator((u_data,mach_data),training.CL_u           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CL_v           = RegularGridInterpolator((v_data,mach_data),training.CL_v           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CL_w           = RegularGridInterpolator((w_data,mach_data),training.CL_w           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CL_p           = RegularGridInterpolator((p_data,mach_data),training.CL_p           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CL_q           = RegularGridInterpolator((q_data,mach_data),training.CL_q           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CL_r           = RegularGridInterpolator((r_data,mach_data),training.CL_r           ,method = 'linear',   bounds_error=False, fill_value=None)      
    
    surrogates.CM_alpha       = RegularGridInterpolator((AoA_data,mach_data),training.CM_alpha     ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CM_beta        = RegularGridInterpolator((Beta_data,mach_data),training.CM_beta     ,method = 'linear',   bounds_error=False, fill_value=None)
    surrogates.CM_u           = RegularGridInterpolator((u_data,mach_data),training.CM_u           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CM_v           = RegularGridInterpolator((v_data,mach_data),training.CM_v           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CM_w           = RegularGridInterpolator((w_data,mach_data),training.CM_w           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CM_p           = RegularGridInterpolator((p_data,mach_data),training.CM_p           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CM_q           = RegularGridInterpolator((q_data,mach_data),training.CM_q           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CM_r           = RegularGridInterpolator((r_data,mach_data),training.CM_r           ,method = 'linear',   bounds_error=False, fill_value=None)      
    
    surrogates.CN_alpha       = RegularGridInterpolator((AoA_data,mach_data),training.CN_alpha     ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CN_beta        = RegularGridInterpolator((Beta_data,mach_data),training.CN_beta     ,method = 'linear',   bounds_error=False, fill_value=None)
    surrogates.CN_u           = RegularGridInterpolator((u_data,mach_data),training.CN_u           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CN_v           = RegularGridInterpolator((v_data,mach_data),training.CN_v           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CN_w           = RegularGridInterpolator((w_data,mach_data),training.CN_w           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CN_p           = RegularGridInterpolator((p_data,mach_data),training.CN_p           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CN_q           = RegularGridInterpolator((q_data,mach_data),training.CN_q           ,method = 'linear',   bounds_error=False, fill_value=None)      
    surrogates.CN_r           = RegularGridInterpolator((r_data,mach_data),training.CN_r           ,method = 'linear',   bounds_error=False, fill_value=None)
    
    surrogates.dClift_dalpha    = interpolate.interp1d(mach_data,training.dClift_dalpha    ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dClift_dbeta     = interpolate.interp1d(mach_data,training.dClift_dbeta     ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate") 
    surrogates.dClift_du        = interpolate.interp1d(mach_data,training.dClift_du        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dClift_dv        = interpolate.interp1d(mach_data,training.dClift_dv        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dClift_dw        = interpolate.interp1d(mach_data,training.dClift_dw        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dClift_dp        = interpolate.interp1d(mach_data,training.dClift_dp        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dClift_dq        = interpolate.interp1d(mach_data,training.dClift_dq        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dClift_dr        = interpolate.interp1d(mach_data,training.dClift_dr        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    
    surrogates.dCdrag_dalpha    = interpolate.interp1d(mach_data,training.dCdrag_dalpha    ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCdrag_dbeta     = interpolate.interp1d(mach_data,training.dCdrag_dbeta     ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")   
    surrogates.dCdrag_du        = interpolate.interp1d(mach_data,training.dCdrag_du        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCdrag_dv        = interpolate.interp1d(mach_data,training.dCdrag_dv        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCdrag_dw        = interpolate.interp1d(mach_data,training.dCdrag_dw        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCdrag_dp        = interpolate.interp1d(mach_data,training.dCdrag_dp        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCdrag_dq        = interpolate.interp1d(mach_data,training.dCdrag_dq        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCdrag_dr        = interpolate.interp1d(mach_data,training.dCdrag_dr        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    
    surrogates.dCX_dalpha       = interpolate.interp1d(mach_data,training.dCX_dalpha       ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCX_dbeta        = interpolate.interp1d(mach_data,training.dCX_dbeta        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")  
    surrogates.dCX_du           = interpolate.interp1d(mach_data,training.dCX_du           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCX_dv           = interpolate.interp1d(mach_data,training.dCX_dv           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCX_dw           = interpolate.interp1d(mach_data,training.dCX_dw           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCX_dp           = interpolate.interp1d(mach_data,training.dCX_dp           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCX_dq           = interpolate.interp1d(mach_data,training.dCX_dq           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCX_dr           = interpolate.interp1d(mach_data,training.dCX_dr           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    
    surrogates.dCY_dalpha       = interpolate.interp1d(mach_data,training.dCY_dalpha       ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCY_dbeta        = interpolate.interp1d(mach_data,training.dCY_dbeta        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")  
    surrogates.dCY_du           = interpolate.interp1d(mach_data,training.dCY_du           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCY_dv           = interpolate.interp1d(mach_data,training.dCY_dv           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCY_dw           = interpolate.interp1d(mach_data,training.dCY_dw           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCY_dp           = interpolate.interp1d(mach_data,training.dCY_dp           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCY_dq           = interpolate.interp1d(mach_data,training.dCY_dq           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCY_dr           = interpolate.interp1d(mach_data,training.dCY_dr           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    
    surrogates.dCZ_dalpha       = interpolate.interp1d(mach_data,training.dCZ_dalpha       ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCZ_dbeta        = interpolate.interp1d(mach_data,training.dCZ_dbeta        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")  
    surrogates.dCZ_du           = interpolate.interp1d(mach_data,training.dCZ_du           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCZ_dv           = interpolate.interp1d(mach_data,training.dCZ_dv           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCZ_dw           = interpolate.interp1d(mach_data,training.dCZ_dw           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCZ_dp           = interpolate.interp1d(mach_data,training.dCZ_dp           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCZ_dq           = interpolate.interp1d(mach_data,training.dCZ_dq           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCZ_dr           = interpolate.interp1d(mach_data,training.dCZ_dr           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    
    surrogates.dCL_dalpha       = interpolate.interp1d(mach_data,training.dCL_dalpha       ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCL_dbeta        = interpolate.interp1d(mach_data,training.dCL_dbeta        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")  
    surrogates.dCL_du           = interpolate.interp1d(mach_data,training.dCL_du           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCL_dv           = interpolate.interp1d(mach_data,training.dCL_dv           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCL_dw           = interpolate.interp1d(mach_data,training.dCL_dw           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCL_dp           = interpolate.interp1d(mach_data,training.dCL_dp           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCL_dq           = interpolate.interp1d(mach_data,training.dCL_dq           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCL_dr           = interpolate.interp1d(mach_data,training.dCL_dr           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    
    surrogates.dCM_dalpha       = interpolate.interp1d(mach_data,training.dCM_dalpha       ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCM_dbeta        = interpolate.interp1d(mach_data,training.dCM_dbeta        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")  
    surrogates.dCM_du           = interpolate.interp1d(mach_data,training.dCM_du           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCM_dv           = interpolate.interp1d(mach_data,training.dCM_dv           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCM_dw           = interpolate.interp1d(mach_data,training.dCM_dw           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCM_dp           = interpolate.interp1d(mach_data,training.dCM_dp           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCM_dq           = interpolate.interp1d(mach_data,training.dCM_dq           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCM_dr           = interpolate.interp1d(mach_data,training.dCM_dr           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    
    surrogates.dCN_dalpha       = interpolate.interp1d(mach_data,training.dCN_dalpha       ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCN_dbeta        = interpolate.interp1d(mach_data,training.dCN_dbeta        ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")  
    surrogates.dCN_du           = interpolate.interp1d(mach_data,training.dCN_du           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCN_dv           = interpolate.interp1d(mach_data,training.dCN_dv           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCN_dw           = interpolate.interp1d(mach_data,training.dCN_dw           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCN_dp           = interpolate.interp1d(mach_data,training.dCN_dp           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCN_dq           = interpolate.interp1d(mach_data,training.dCN_dq           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")      
    surrogates.dCN_dr           = interpolate.interp1d(mach_data,training.dCN_dr           ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")
   

    if aerodynamics.aileron_flag: 
        surrogates.Clift_delta_a    = RegularGridInterpolator((aileron_data,mach_data),training.Clift_delta_a  ,method = 'linear',   bounds_error=False, fill_value=None)
        surrogates.Cdrag_delta_a    = RegularGridInterpolator((aileron_data,mach_data),training.Cdrag_delta_a  ,method = 'linear',   bounds_error=False, fill_value=None)
        surrogates.CX_delta_a       = RegularGridInterpolator((aileron_data,mach_data),training.CX_delta_a  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CY_delta_a       = RegularGridInterpolator((aileron_data,mach_data),training.CY_delta_a  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CZ_delta_a       = RegularGridInterpolator((aileron_data,mach_data),training.CZ_delta_a  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CL_delta_a       = RegularGridInterpolator((aileron_data,mach_data),training.CL_delta_a  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CM_delta_a       = RegularGridInterpolator((aileron_data,mach_data),training.CM_delta_a  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CN_delta_a       = RegularGridInterpolator((aileron_data,mach_data),training.CN_delta_a  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.dClift_ddelta_a  = interpolate.interp1d(mach_data,training.dClift_ddelta_a     , kind = 'linear',   bounds_error=False, fill_value="extrapolate")
        surrogates.dCdrag_ddelta_a  = interpolate.interp1d(mach_data,training.dCdrag_ddelta_a     , kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCX_ddelta_a     = interpolate.interp1d(mach_data,training.dCX_ddelta_a        , kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCY_ddelta_a     = interpolate.interp1d(mach_data,training.dCY_ddelta_a        , kind = 'linear',   bounds_error=False, fill_value="extrapolate")  
        surrogates.dCZ_ddelta_a     = interpolate.interp1d(mach_data,training.dCZ_ddelta_a        , kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCL_ddelta_a     = interpolate.interp1d(mach_data,training.dCL_ddelta_a        , kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCM_ddelta_a     = interpolate.interp1d(mach_data,training.dCM_ddelta_a        , kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCN_ddelta_a     = interpolate.interp1d(mach_data,training.dCN_ddelta_a        , kind = 'linear',   bounds_error=False, fill_value="extrapolate")             
    
    if aerodynamics.elevator_flag: 
        surrogates.Clift_delta_e    = RegularGridInterpolator((elevator_data,mach_data),training.Clift_delta_e  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.Cdrag_delta_e    = RegularGridInterpolator((elevator_data,mach_data),training.Cdrag_delta_e  ,method = 'linear',   bounds_error=False, fill_value=None)
        surrogates.CX_delta_e       = RegularGridInterpolator((elevator_data,mach_data),training.CX_delta_e  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CY_delta_e       = RegularGridInterpolator((elevator_data,mach_data),training.CY_delta_e  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CZ_delta_e       = RegularGridInterpolator((elevator_data,mach_data),training.CZ_delta_e  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CL_delta_e       = RegularGridInterpolator((elevator_data,mach_data),training.CL_delta_e  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CM_delta_e       = RegularGridInterpolator((elevator_data,mach_data),training.CM_delta_e  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CN_delta_e       = RegularGridInterpolator((elevator_data,mach_data),training.CN_delta_e  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.dClift_ddelta_e  = interpolate.interp1d(mach_data,training.dClift_ddelta_e  ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")
        surrogates.dCdrag_ddelta_e  = interpolate.interp1d(mach_data,training.dCdrag_ddelta_e  ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate") 
        surrogates.dCX_ddelta_e     = interpolate.interp1d(mach_data,training.dCX_ddelta_e     ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate") 
        surrogates.dCY_ddelta_e     = interpolate.interp1d(mach_data,training.dCY_ddelta_e     ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")  
        surrogates.dCZ_ddelta_e     = interpolate.interp1d(mach_data,training.dCZ_ddelta_e     ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate") 
        surrogates.dCL_ddelta_e     = interpolate.interp1d(mach_data,training.dCL_ddelta_e     ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate") 
        surrogates.dCM_ddelta_e     = interpolate.interp1d(mach_data,training.dCM_ddelta_e     ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate") 
        surrogates.dCN_ddelta_e     = interpolate.interp1d(mach_data,training.dCN_ddelta_e     ,kind = 'linear',   bounds_error=False, fill_value= "extrapolate")   
    
    
    if aerodynamics.rudder_flag: 
        surrogates.Clift_delta_r    = RegularGridInterpolator((rudder_data,mach_data),training.Clift_delta_r  ,method = 'linear',   bounds_error=False, fill_value=None)  
        surrogates.Cdrag_delta_r    = RegularGridInterpolator((rudder_data,mach_data),training.Cdrag_delta_r  ,method = 'linear',   bounds_error=False, fill_value=None)
        surrogates.CX_delta_r       = RegularGridInterpolator((rudder_data,mach_data),training.CX_delta_r  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CY_delta_r       = RegularGridInterpolator((rudder_data,mach_data),training.CY_delta_r  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CZ_delta_r       = RegularGridInterpolator((rudder_data,mach_data),training.CZ_delta_r  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CL_delta_r       = RegularGridInterpolator((rudder_data,mach_data),training.CL_delta_r  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CM_delta_r       = RegularGridInterpolator((rudder_data,mach_data),training.CM_delta_r  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CN_delta_r       = RegularGridInterpolator((rudder_data,mach_data),training.CN_delta_r  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.dClift_ddelta_r  = interpolate.interp1d(mach_data,training.dClift_ddelta_r    ,kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCdrag_ddelta_r  = interpolate.interp1d(mach_data,training.dCdrag_ddelta_r    ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")  
        surrogates.dCX_ddelta_r     = interpolate.interp1d(mach_data,training.dCX_ddelta_r       ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")  
        surrogates.dCY_ddelta_r     = interpolate.interp1d(mach_data,training.dCY_ddelta_r       ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")   
        surrogates.dCZ_ddelta_r     = interpolate.interp1d(mach_data,training.dCZ_ddelta_r       ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")  
        surrogates.dCL_ddelta_r     = interpolate.interp1d(mach_data,training.dCL_ddelta_r       ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")  
        surrogates.dCM_ddelta_r     = interpolate.interp1d(mach_data,training.dCM_ddelta_r       ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")  
        surrogates.dCN_ddelta_r     = interpolate.interp1d(mach_data,training.dCN_ddelta_r       ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")    
    
    if aerodynamics.flap_flag:
        surrogates.Clift_delta_f    = RegularGridInterpolator((flap_data,mach_data),training.Clift_delta_f  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.Cdrag_delta_f    = RegularGridInterpolator((flap_data,mach_data),training.Cdrag_delta_f  ,method = 'linear',   bounds_error=False, fill_value=None)
        surrogates.CX_delta_f       = RegularGridInterpolator((flap_data,mach_data),training.CX_delta_f  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CY_delta_f       = RegularGridInterpolator((flap_data,mach_data),training.CY_delta_f  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CZ_delta_f       = RegularGridInterpolator((flap_data,mach_data),training.CZ_delta_f  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CL_delta_f       = RegularGridInterpolator((flap_data,mach_data),training.CL_delta_f  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CM_delta_f       = RegularGridInterpolator((flap_data,mach_data),training.CM_delta_f  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CN_delta_f       = RegularGridInterpolator((flap_data,mach_data),training.CN_delta_f  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.dClift_ddelta_f  = interpolate.interp1d(mach_data,training.dClift_ddelta_f  ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")
        surrogates.dCdrag_ddelta_f  = interpolate.interp1d(mach_data,training.dCdrag_ddelta_f  ,kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCX_ddelta_f     = interpolate.interp1d(mach_data,training.dCX_ddelta_f     ,kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCY_ddelta_f     = interpolate.interp1d(mach_data,training.dCY_ddelta_f     ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")  
        surrogates.dCZ_ddelta_f     = interpolate.interp1d(mach_data,training.dCZ_ddelta_f     ,kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCL_ddelta_f     = interpolate.interp1d(mach_data,training.dCL_ddelta_f     ,kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCM_ddelta_f     = interpolate.interp1d(mach_data,training.dCM_ddelta_f     ,kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCN_ddelta_f     = interpolate.interp1d(mach_data,training.dCN_ddelta_f     ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")   
    
    if aerodynamics.slat_flag: 
        surrogates.Clift_delta_s    = RegularGridInterpolator((slat_data,mach_data),training.Clift_delta_s  ,method = 'linear',   bounds_error=False, fill_value=None)         
        surrogates.Cdrag_delta_s    = RegularGridInterpolator((slat_data,mach_data),training.Cdrag_delta_s  ,method = 'linear',   bounds_error=False, fill_value=None)
        surrogates.CX_delta_s       = RegularGridInterpolator((slat_data,mach_data),training.CX_delta_s  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CY_delta_s       = RegularGridInterpolator((slat_data,mach_data),training.CY_delta_s  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CZ_delta_s       = RegularGridInterpolator((slat_data,mach_data),training.CZ_delta_s  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CL_delta_s       = RegularGridInterpolator((slat_data,mach_data),training.CL_delta_s  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CM_delta_s       = RegularGridInterpolator((slat_data,mach_data),training.CM_delta_s  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.CN_delta_s       = RegularGridInterpolator((slat_data,mach_data),training.CN_delta_s  ,method = 'linear',   bounds_error=False, fill_value=None) 
        surrogates.dClift_ddelta_s  = interpolate.interp1d(mach_data,training.dClift_ddelta_s  ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")  
        surrogates.dCdrag_ddelta_s  = interpolate.interp1d(mach_data,training.dCdrag_ddelta_s  ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")   
        surrogates.dCX_ddelta_s     = interpolate.interp1d(mach_data,training.dCX_ddelta_s     ,kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCY_ddelta_s     = interpolate.interp1d(mach_data,training.dCY_ddelta_s     ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")  
        surrogates.dCZ_ddelta_s     = interpolate.interp1d(mach_data,training.dCZ_ddelta_s     ,kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCL_ddelta_s     = interpolate.interp1d(mach_data,training.dCL_ddelta_s     ,kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCM_ddelta_s     = interpolate.interp1d(mach_data,training.dCM_ddelta_s     ,kind = 'linear',   bounds_error=False, fill_value="extrapolate") 
        surrogates.dCN_ddelta_s     = interpolate.interp1d(mach_data,training.dCN_ddelta_s     ,kind = 'linear',   bounds_error=False, fill_value="extrapolate")   
   
    return surrogates
 