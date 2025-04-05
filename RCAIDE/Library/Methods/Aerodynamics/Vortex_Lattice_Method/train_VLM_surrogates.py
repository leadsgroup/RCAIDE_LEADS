# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/train_VLM_surrogates.py
#  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
import RCAIDE 
from RCAIDE.Framework.Core import  Data 
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM import  VLM
# package imports
import numpy  as np
from copy     import  deepcopy

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ---------------------------------------------------------------------------------------------------------------------- 
def train_VLM_surrogates(aerodynamics):
    """Call methods to run VLM for sample point evaluation. 
    
    Assumptions:
        CY_beta multiplied by 2, verified against literature and AVL 
        CL_p multiplied by -2, verified against literature and AVL 
        CM_q multiplied by 10, verified against literature and AVL 
        CN_p multiplied by -3, verified against literature and AVL 
        CN_r multiplied by 3, verified against literature and AVL 
        CL_delta_a multiplied by -1, verified against literature and AVL 
        CLift_delta_e multiplied by 0.5, verified against literature and AVL 
        CY_alpha multiplied by 0, based on theory 
        CL_beta multiplied by -1, verified against literature and AVL 
        p derivatives multiplied by -10, verified against literature and AVL 
        r derivatives multiplied by -10, verified against literature and AVL 
        Rudder derivatives multiplied by -1, verified against literature  
        Aileron derivatives multiplied by -1, verified against literature
        
    Source:
        None

    Args:
        aerodynamics       : VLM analysis          [unitless] 
        
    Returns: 
        None    
    """
 
    Mach          = aerodynamics.training.Mach 
    training      = aerodynamics.training  
    sub_len       = int(sum(Mach<1.))  
    sub_Mach      = Mach[:sub_len] 
    sup_Mach      = Mach[sub_len:] 
    
    training.subsonic    =  train_model(aerodynamics, sub_Mach)  
    training.supersonic  =  train_model(aerodynamics, sup_Mach)
    training.transonic   =  train_trasonic_model(aerodynamics, training.subsonic,training.supersonic,sub_Mach, sup_Mach) 
    return 
    
def train_model(aerodynamics, Mach): 
    """Sub function that call methods to run VLM for sample point evaluation. 
    
    Assumptions:
        None
        
    Source:
        None

    Args:
        aerodynamics       : VLM analysis          [unitless] 
        
    Returns: 
        None    
    """    

    vehicle        = aerodynamics.vehicle
    settings       = aerodynamics.settings
    AoA            = aerodynamics.training.angle_of_attack                  
    Beta           = aerodynamics.training.sideslip_angle
    training       = Data()
    training.Mach  = Mach 
    
    # loop through wings to determine what control surfaces are present 
    for wing in aerodynamics.vehicle.wings: 
        for control_surface in wing.control_surfaces:
            control_surface.deflection  =  0.0
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron:  
                delta_a                    = aerodynamics.training.aileron_deflection
                len_d_a                    = len(delta_a)
                aerodynamics.aileron_flag  = True 
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator:  
                delta_e                    = aerodynamics.training.elevator_deflection
                len_d_e                    = len(delta_e)   
                aerodynamics.elevator_flag = True 
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder:   
                delta_r                    = aerodynamics.training.rudder_deflection
                aerodynamics.rudder_flag   = True
                len_d_r                    = len(delta_r)  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Slat:  
                delta_s                = aerodynamics.training.slat_deflection
                len_d_s                = len(delta_s)   
                aerodynamics.slat_flag = True 
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap: 
                delta_f                     = aerodynamics.training.rudder_deflection
                len_d_f                     = len(delta_f)  
                aerodynamics.flap_flag      = True 
             
    u              = aerodynamics.training.u
    v              = aerodynamics.training.v
    w              = aerodynamics.training.w
    pitch_rate     = aerodynamics.training.pitch_rate
    roll_rate      = aerodynamics.training.roll_rate
    yaw_rate       = aerodynamics.training.yaw_rate  
    len_Mach       = len(Mach)        
    len_AoA        = len(AoA)  
    len_Beta       = len(Beta)
    len_u          = len(u)
    len_v          = len(v)
    len_w          = len(w)
    len_q          = len(pitch_rate)
    len_p          = len(roll_rate) 
    len_r          = len(yaw_rate) 
    
    # --------------------------------------------------------------------------------------------------------------
    # Alpha
    # --------------------------------------------------------------------------------------------------------------
    
    # Setup new array shapes for vectorization 
    # stakcing 9x9 matrices into one horizontal line(81)  
    AoAs       = np.atleast_2d(np.tile(AoA,len_Mach).T.flatten()).T 
    Machs      = np.atleast_2d(np.repeat(Mach,len_AoA)).T        
    
    # reset conditions  
    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
    conditions.freestream.mach_number               = Machs
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs)*AoAs 
    
    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res, S_ref,b_ref,c_ref,X_ref,Y_ref ,Z_ref, Clift_wing_res, Cdrag_wing_res,_,_,_, _= call_VLM(conditions,settings,vehicle)
    
    Clift_alpha   = np.reshape(Clift_res,(len_Mach,len_AoA)).T 
    Cdrag_alpha   = np.reshape(Cdrag_res,(len_Mach,len_AoA)).T 
    CX_alpha      = np.reshape(CX_res,(len_Mach,len_AoA)).T 
    CY_alpha      = np.reshape(CY_res,(len_Mach,len_AoA)).T 
    CZ_alpha      = np.reshape(CZ_res,(len_Mach,len_AoA)).T 
    CL_alpha      = np.reshape(CL_res,(len_Mach,len_AoA)).T 
    CM_alpha      = np.reshape(CM_res,(len_Mach,len_AoA)).T 
    CN_alpha      = np.reshape(CN_res,(len_Mach,len_AoA)).T  
    
    # Angle of Attack at 0 Degrees 
    Clift_alpha_0   =  np.tile(Clift_alpha[2][None,:],(2,1))
    Cdrag_alpha_0   =  np.tile(Cdrag_alpha[2][None,:],(2,1))
    CX_alpha_0      =  np.tile(CX_alpha[2][None,:],(2, 1)) 
    CY_alpha_0      =  0 * np.tile(CY_alpha[2][None,:],(2, 1)) 
    CZ_alpha_0      =  np.tile(CZ_alpha[2][None,:],(2, 1)) 
    CL_alpha_0      =  0 * np.tile(CL_alpha[2][None,:],(2, 1)) 
    CM_alpha_0      =  np.tile(CM_alpha[2][None,:],(2, 1)) 
    CN_alpha_0      =  0 * np.tile(CN_alpha[2][None,:],(2, 1))  

    aerodynamics.reference_values.S_ref = S_ref
    aerodynamics.reference_values.b_ref = b_ref
    aerodynamics.reference_values.c_ref = c_ref
    aerodynamics.reference_values.X_ref = X_ref
    aerodynamics.reference_values.Y_ref = Y_ref
    aerodynamics.reference_values.Z_ref = Z_ref
    aerodynamics.reference_values.aspect_ratio = (b_ref ** 2) / S_ref
    
    Clift_wing_alpha = Data()
    Cdrag_wing_alpha = Data() 
    for wing in vehicle.wings: 
        Clift_wing_alpha[wing.tag] = np.reshape(Clift_wing_res[wing.tag],(len_Mach,len_AoA)).T    
        Cdrag_wing_alpha[wing.tag] = np.reshape(Cdrag_wing_res[wing.tag],(len_Mach,len_AoA)).T  
 
     
    # --------------------------------------------------------------------------------------------------------------
    # Beta 
    # --------------------------------------------------------------------------------------------------------------
    Betas         = np.atleast_2d(np.tile(Beta,len_Mach).T.flatten()).T 
    Machs         = np.atleast_2d(np.repeat(Mach,len_Beta)).T        

    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
    conditions.expand_rows(rows= len(Machs))
    conditions.freestream.mach_number               = Machs 
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
    conditions.aerodynamics.angles.beta             = np.ones_like(Machs)*Betas   
    
    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_, _= call_VLM(conditions,settings,vehicle)
    
    Clift_beta =    np.reshape(Clift_res,(len_Mach,len_Beta)).T - Clift_alpha_0
    Cdrag_beta =    np.reshape(Cdrag_res,(len_Mach,len_Beta)).T - Cdrag_alpha_0                                
    CX_beta    =    np.reshape(CX_res,(len_Mach,len_Beta)).T    - CX_alpha_0   
    CY_beta    =    np.reshape(CY_res,(len_Mach,len_Beta)).T    - CY_alpha_0   
    CZ_beta    =    np.reshape(CZ_res,(len_Mach,len_Beta)).T    - CZ_alpha_0   
    CL_beta    = - (np.reshape(CL_res,(len_Mach,len_Beta)).T    - CL_alpha_0) 
    CM_beta    =    np.reshape(CM_res,(len_Mach,len_Beta)).T    - CM_alpha_0   
    CN_beta    =    np.reshape(CN_res,(len_Mach,len_Beta)).T    - CN_alpha_0 
 
    # -------------------------------------------------------      
    # Velocity u 
    # -------------------------------------------------------
    u_s     = np.atleast_2d(np.tile(u, len_Mach).T.flatten()).T 
    Machs   = np.atleast_2d(np.repeat(Mach,len_u)).T                   
    conditions                                      = RCAIDE.Framework.Mission.Common.Results() 
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12 
    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
    conditions.freestream.mach_number               = Machs + Machs*u_s 
    
    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_, _= call_VLM(conditions,settings,vehicle)
    
    Clift_u     = np.reshape(Clift_res,(len_Mach,len_u)).T - Clift_alpha_0
    Cdrag_u     = np.reshape(Cdrag_res,(len_Mach,len_u)).T - Cdrag_alpha_0
    CX_u        = np.reshape(CX_res,(len_Mach,len_u)).T    - CX_alpha_0   
    CY_u        = np.reshape(CY_res,(len_Mach,len_u)).T    - CY_alpha_0   
    CZ_u        = np.reshape(CZ_res,(len_Mach,len_u)).T    - CZ_alpha_0   
    CL_u        = np.reshape(CL_res,(len_Mach,len_u)).T    - CL_alpha_0   
    CM_u        = np.reshape(CM_res,(len_Mach,len_u)).T    - CM_alpha_0   
    CN_u        = np.reshape(CN_res,(len_Mach,len_u)).T    - CN_alpha_0   
    
    # -------------------------------------------------------               
    # Velocity v 
    # -------------------------------------------------------
    v_s     = np.atleast_2d(np.tile(v, len_Mach).T.flatten()).T 
    Machs         = np.atleast_2d(np.repeat(Mach,len_v)).T    

    conditions                                      = RCAIDE.Framework.Mission.Common.Results()  
    conditions.freestream.mach_number               = Machs
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs)       
    
    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_, _= call_VLM(conditions,settings,vehicle)
    
    Clift_v     = np.reshape(Clift_res,(len_Mach,len_v)).T - Clift_alpha_0
    Cdrag_v     = np.reshape(Cdrag_res,(len_Mach,len_v)).T - Cdrag_alpha_0
    CX_v        = np.reshape(CX_res,(len_Mach,len_v)).T    - CX_alpha_0   
    CY_v        = np.reshape(CY_res,(len_Mach,len_v)).T    - CY_alpha_0   
    CZ_v        = np.reshape(CZ_res,(len_Mach,len_v)).T    - CZ_alpha_0   
    CL_v        = np.reshape(CL_res,(len_Mach,len_v)).T    - CL_alpha_0   
    CM_v        = np.reshape(CM_res,(len_Mach,len_v)).T    - CM_alpha_0   
    CN_v        = np.reshape(CN_res,(len_Mach,len_v)).T    - CN_alpha_0   
    
    # -------------------------------------------------------               
    # Velocity w 
    # -------------------------------------------------------
    w_s     = np.atleast_2d(np.tile(w, len_Mach).T.flatten()).T 
    Machs   = np.atleast_2d(np.repeat(Mach,len_w)).T
     
    conditions                                      = RCAIDE.Framework.Mission.Common.Results()  
    conditions.freestream.mach_number               = Machs 
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
    
    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_, _= call_VLM(conditions,settings,vehicle)
    
    Clift_w     = np.reshape(Clift_res,(len_Mach,len_w)).T - Clift_alpha_0
    Cdrag_w     = np.reshape(Cdrag_res,(len_Mach,len_w)).T - Cdrag_alpha_0
    CX_w        = np.reshape(CX_res,(len_Mach,len_w)).T    - CX_alpha_0   
    CY_w        = np.reshape(CY_res,(len_Mach,len_w)).T    - CY_alpha_0   
    CZ_w        = np.reshape(CZ_res,(len_Mach,len_w)).T    - CZ_alpha_0   
    CL_w        = np.reshape(CL_res,(len_Mach,len_w)).T    - CL_alpha_0   
    CM_w        = np.reshape(CM_res,(len_Mach,len_w)).T    - CM_alpha_0   
    CN_w        = np.reshape(CN_res,(len_Mach,len_w)).T    - CN_alpha_0           
                    
    # -------------------------------------------------------               
    # Pitch Rate 
    # -------------------------------------------------------
    q_s     = np.atleast_2d(np.tile(pitch_rate, len_Mach).T.flatten()).T 
    Machs   = np.atleast_2d(np.repeat(Mach,len_q)).T

    conditions                                      = RCAIDE.Framework.Mission.Common.Results() 
    conditions.freestream.mach_number               = Machs 
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
    conditions.static_stability.pitch_rate          = np.ones_like(Machs)*q_s     
    conditions.freestream.velocity                  = Machs * 343 # speed of sound   
    
    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_, _= call_VLM(conditions,settings,vehicle)
    
    Clift_q     = np.reshape(Clift_res,(len_Mach,len_q)).T - Clift_alpha_0
    Cdrag_q     = np.reshape(Cdrag_res,(len_Mach,len_q)).T - Cdrag_alpha_0
    CX_q        = np.reshape(CX_res,(len_Mach,len_q)).T    - CX_alpha_0   
    CY_q        = np.reshape(CY_res,(len_Mach,len_q)).T    - CY_alpha_0   
    CZ_q        = np.reshape(CZ_res,(len_Mach,len_q)).T    - CZ_alpha_0   
    CL_q        = np.reshape(CL_res,(len_Mach,len_q)).T    - CL_alpha_0   
    CM_q        = np.reshape(CM_res,(len_Mach,len_q)).T    - CM_alpha_0   
    CN_q        = np.reshape(CN_res,(len_Mach,len_q)).T    - CN_alpha_0   

    # -------------------------------------------------------               
    # Roll  Rate 
    # -------------------------------------------------------    
    p_s     = np.atleast_2d(np.tile(roll_rate, len_Mach).T.flatten()).T 
    Machs         = np.atleast_2d(np.repeat(Mach,len_p)).T

    conditions                                      = RCAIDE.Framework.Mission.Common.Results() 
    conditions.freestream.mach_number               = Machs  
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12 
    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
    conditions.static_stability.roll_rate           = np.ones_like(Machs)*p_s 
    conditions.freestream.velocity                  = Machs * 343 # speed of sound           
        
    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_, _= call_VLM(conditions,settings,vehicle)  
        
    Clift_p     = -10*(np.reshape(Clift_res,(len_Mach,len_p)).T - Clift_alpha_0)
    Cdrag_p     = -10*(np.reshape(Cdrag_res,(len_Mach,len_p)).T - Cdrag_alpha_0)
    CX_p        = -10*(np.reshape(CX_res,(len_Mach,len_p)).T    - CX_alpha_0   )
    CY_p        = -10*(np.reshape(CY_res,(len_Mach,len_p)).T    - CY_alpha_0   )
    CZ_p        = -10*(np.reshape(CZ_res,(len_Mach,len_p)).T    - CZ_alpha_0   )
    CL_p        = (np.reshape(CL_res,(len_Mach,len_p)).T    - CL_alpha_0   )
    CM_p        = -10*(np.reshape(CM_res,(len_Mach,len_p)).T    - CM_alpha_0   )
    CN_p        = (np.reshape(CN_res,(len_Mach,len_p)).T    - CN_alpha_0   )   

    # -------------------------------------------------------               
    # Yaw Rate 
    # -------------------------------------------------------        
    r_s     = np.atleast_2d(np.tile(yaw_rate, len_Mach).T.flatten()).T 
    Machs         = np.atleast_2d(np.repeat(Mach,len_r)).T

    conditions                                      = RCAIDE.Framework.Mission.Common.Results() 
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs)*1E-2 
    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
    conditions.freestream.mach_number               = Machs 
    conditions.static_stability.yaw_rate            = np.ones_like(Machs)*r_s
    conditions.freestream.velocity                  = Machs * 343
    
    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_, _= call_VLM(conditions,settings,vehicle)
    
    Clift_r     = 10*(np.reshape(Clift_res,(len_Mach,len_r)).T - Clift_alpha_0)
    Cdrag_r     = 10*(np.reshape(Cdrag_res,(len_Mach,len_r)).T - Cdrag_alpha_0)
    CX_r        = 10*(np.reshape(CX_res,(len_Mach,len_r)).T    - CX_alpha_0   )
    CY_r        = 10*(np.reshape(CY_res,(len_Mach,len_r)).T    - CY_alpha_0   )
    CZ_r        = 10*(np.reshape(CZ_res,(len_Mach,len_r)).T    - CZ_alpha_0   )
    CL_r        = (np.reshape(CL_res,(len_Mach,len_r)).T    - CL_alpha_0   )
    CM_r        = 10*(np.reshape(CM_res,(len_Mach,len_r)).T    - CM_alpha_0   )
    CN_r        = (np.reshape(CN_res,(len_Mach,len_r)).T    - CN_alpha_0   ) 
        
    # STABILITY COEFFICIENTS  
    training.Clift_alpha       = Clift_alpha 
    training.Clift_wing_alpha  = Clift_wing_alpha
    training.Clift_beta        = Clift_beta
    training.Clift_u           = Clift_u       
    training.Clift_v           = Clift_v       
    training.Clift_w           = Clift_w       
    training.Clift_p           = Clift_p       
    training.Clift_q           = Clift_q       
    training.Clift_r           = Clift_r   

    training.Cdrag_alpha       = Cdrag_alpha   
    training.Cdrag_wing_alpha  = Cdrag_wing_alpha  
    training.Cdrag_beta        = Cdrag_beta 
    training.Cdrag_u           = Cdrag_u       
    training.Cdrag_v           = Cdrag_v       
    training.Cdrag_w           = Cdrag_w       
    training.Cdrag_p           = Cdrag_p        
    training.Cdrag_q           = Cdrag_q       
    training.Cdrag_r           = Cdrag_r   

    training.CX_alpha          = CX_alpha      
    training.CX_beta           = CX_beta 
    training.CX_u              = CX_u          
    training.CX_v              = CX_v          
    training.CX_w              = CX_w          
    training.CX_p              = CX_p           
    training.CX_q              = CX_q          
    training.CX_r              = CX_r   

    training.CY_alpha          = CY_alpha      
    training.CY_beta           = CY_beta            
    training.CY_u              = CY_u          
    training.CY_v              = CY_v          
    training.CY_w              = CY_w          
    training.CY_p              = CY_p            
    training.CY_q              = CY_q           
    training.CY_r              = CY_r      

    training.CZ_alpha          = CZ_alpha      
    training.CZ_beta           = CZ_beta
    training.CZ_u              = CZ_u          
    training.CZ_v              = CZ_v          
    training.CZ_w              = CZ_w          
    training.CZ_p              = CZ_p           
    training.CZ_q              = CZ_q          
    training.CZ_r              = CZ_r    

    training.CL_alpha          = CL_alpha      
    training.CL_beta           = CL_beta          
    training.CL_u              = CL_u          
    training.CL_v              = CL_v          
    training.CL_w              = CL_w          
    training.CL_p              = CL_p           
    training.CL_q              = CL_q          
    training.CL_r              = CL_r      

    training.CM_alpha          = CM_alpha      
    training.CM_beta           = CM_beta            
    training.CM_u              = CM_u          
    training.CM_v              = CM_v          
    training.CM_w              = CM_w          
    training.CM_p              = CM_p             
    training.CM_q              = CM_q            
    training.CM_r              = CM_r    

    training.CN_alpha          = CN_alpha      
    training.CN_beta           = CN_beta        
    training.CN_u              = CN_u          
    training.CN_v              = CN_v          
    training.CN_w              = CN_w          
    training.CN_p              = CN_p            
    training.CN_q              = CN_q           
    training.CN_r              = CN_r
      
    # STABILITY DERIVATIVES 
    training.dClift_dalpha = (Clift_alpha[0,:] - Clift_alpha[1,:]) / (AoA[0] - AoA[1])
    training.dClift_dbeta = (Clift_beta[0,:] - Clift_beta[1,:]) / (Beta[0] - Beta[1]) 
    training.dClift_du = (Clift_u[0,:] - Clift_u[1,:]) / (u[0] - u[1])            
    training.dClift_dv = (Clift_v[0,:] - Clift_v[1,:]) / (v[0] - v[1])          
    training.dClift_dw = (Clift_w[0,:] - Clift_w[1,:]) / (w[0] - w[1])         
    training.dClift_dp = (Clift_p[0,:] - Clift_p[1,:]) / (roll_rate[0]-roll_rate[1])            
    training.dClift_dq = (Clift_q[0,:] - Clift_q[1,:]) / (pitch_rate[0]-pitch_rate[1])        
    training.dClift_dr = (Clift_r[0,:] - Clift_r[1,:]) / (yaw_rate[0]-yaw_rate[1])     

    training.dCdrag_dalpha = (Cdrag_alpha[0,:] - Cdrag_alpha[1,:]) / (AoA[0] - AoA[1])    
    training.dCdrag_dbeta = (Cdrag_beta[0,:] - Cdrag_beta[1,:]) / (Beta[0] - Beta[1])
    training.dCdrag_du = (Cdrag_u[0,:] - Cdrag_u[1,:]) / (u[0] - u[1])                     
    training.dCdrag_dv = (Cdrag_v[0,:] - Cdrag_v[1,:]) / (v[0] - v[1])                   
    training.dCdrag_dw = (Cdrag_w[0,:] - Cdrag_w[1,:]) / (w[0] - w[1])                  
    training.dCdrag_dp = (Cdrag_p[0,:] - Cdrag_p[1,:]) / (roll_rate[0]-roll_rate[1])             
    training.dCdrag_dq = (Cdrag_q[0,:] - Cdrag_q[1,:]) / (pitch_rate[0]-pitch_rate[1])         
    training.dCdrag_dr = (Cdrag_r[0,:] - Cdrag_r[1,:]) / (yaw_rate[0]-yaw_rate[1])   

    training.dCX_dalpha = (CX_alpha[0,:] - CX_alpha[1,:]) / (AoA[0] - AoA[1])            
    training.dCX_dbeta = (CX_beta[0,:] - CX_beta[1,:]) / (Beta[0] - Beta[1])     
    training.dCX_du = (CX_u[0,:] - CX_u[1,:]) / (u[0] - u[1])                                 
    training.dCX_dv = (CX_v[0,:] - CX_v[1,:]) / (v[0] - v[1])                               
    training.dCX_dw = (CX_w[0,:] - CX_w[1,:]) / (w[0] - w[1])                              
    training.dCX_dp = (CX_p[0,:] - CX_p[1,:]) / (roll_rate[0]-roll_rate[1])                
    training.dCX_dq = (CX_q[0,:] - CX_q[1,:]) / (pitch_rate[0]-pitch_rate[1])            
    training.dCX_dr = (CX_r[0,:] - CX_r[1,:]) / (yaw_rate[0]-yaw_rate[1])     

    training.dCY_dalpha = (CY_alpha[0,:] - CY_alpha[1,:]) / (AoA[0] - AoA[1])         
    training.dCY_dbeta = 2*((CY_beta[0,:] - CY_beta[1,:]) / (Beta[0] - Beta[1]))
    training.dCY_du = (CY_u[0,:] - CY_u[1,:]) / (u[0] - u[1])                                             
    training.dCY_dv = (CY_v[0,:] - CY_v[1,:]) / (v[0] - v[1])                                           
    training.dCY_dw = (CY_w[0,:] - CY_w[1,:]) / (w[0] - w[1])                                          
    training.dCY_dp = (CY_p[0,:] - CY_p[1,:]) / (roll_rate[0]-roll_rate[1])                 
    training.dCY_dq = (CY_q[0,:] - CY_q[1,:]) / (pitch_rate[0]-pitch_rate[1])             
    training.dCY_dr = (CY_r[0,:] - CY_r[1,:]) / (yaw_rate[0]-yaw_rate[1])       

    training.dCZ_dalpha = (CZ_alpha[0,:] - CZ_alpha[1,:]) / (AoA[0] - AoA[1])             
    training.dCZ_dbeta = (CZ_beta[0,:] - CZ_beta[1,:]) / (Beta[0] - Beta[1])                
    training.dCZ_du = (CZ_u[0,:] - CZ_u[1,:]) / (u[0] - u[1])                                              
    training.dCZ_dv = (CZ_v[0,:] - CZ_v[1,:]) / (v[0] - v[1])                                              
    training.dCZ_dw = (CZ_w[0,:] - CZ_w[1,:]) / (w[0] - w[1])                                              
    training.dCZ_dp = (CZ_p[0,:] - CZ_p[1,:]) / (roll_rate[0]-roll_rate[1])                
    training.dCZ_dq = (CZ_q[0,:] - CZ_q[1,:]) / (pitch_rate[0]-pitch_rate[1])            
    training.dCZ_dr = (CZ_r[0,:] - CZ_r[1,:]) / (yaw_rate[0]-yaw_rate[1])       

    training.dCL_dalpha = (CL_alpha[0,:] - CL_alpha[1,:]) / (AoA[0] - AoA[1])         
    training.dCL_dbeta = ((CL_beta[0,:] - CL_beta[1,:]) / (Beta[0] - Beta[1]))                
    training.dCL_du = (CL_u[0,:] - CL_u[1,:]) / (u[0] - u[1])                                              
    training.dCL_dv = (CL_v[0,:] - CL_v[1,:]) / (v[0] - v[1])                                              
    training.dCL_dw = (CL_w[0,:] - CL_w[1,:]) / (w[0] - w[1])                                              
    training.dCL_dp = -2*((CL_p[0,:] - CL_p[1,:]) / (roll_rate[0]-roll_rate[1]))                
    training.dCL_dq = (CL_q[0,:] - CL_q[1,:]) / (pitch_rate[0]-pitch_rate[1])            
    training.dCL_dr = (CL_r[0,:] - CL_r[1,:]) / (yaw_rate[0]-yaw_rate[1])    

    training.dCM_dalpha = (CM_alpha[0,:] - CM_alpha[1,:]) / (AoA[0] - AoA[1])          
    training.dCM_dbeta = (CM_beta[0,:] - CM_beta[1,:]) / (Beta[0] - Beta[1])  
    training.dCM_du = (CM_u[0,:] - CM_u[1,:]) / (u[0] - u[1])                                               
    training.dCM_dv = (CM_v[0,:] - CM_v[1,:]) / (v[0] - v[1])                                               
    training.dCM_dw = (CM_w[0,:] - CM_w[1,:]) / (w[0] - w[1])                                               
    training.dCM_dp = (CM_p[0,:] - CM_p[1,:]) / (roll_rate[0]-roll_rate[1])                 
    training.dCM_dq = 10*((CM_q[0,:] - CM_q[1,:]) / (pitch_rate[0]-pitch_rate[1]))            
    training.dCM_dr = (CM_r[0,:] - CM_r[1,:]) / (yaw_rate[0]-yaw_rate[1])        

    training.dCN_dalpha = (CN_alpha[0,:] - CN_alpha[1,:]) / (AoA[0] - AoA[1])          
    training.dCN_dbeta = (CN_beta[0,:] - CN_beta[1,:]) / (Beta[0] - Beta[1]) 
    training.dCN_du = (CN_u[0,:] - CN_u[1,:]) / (u[0] - u[1])                                               
    training.dCN_dv = (CN_v[0,:] - CN_v[1,:]) / (v[0] - v[1])                                               
    training.dCN_dw = (CN_w[0,:] - CN_w[1,:]) / (w[0] - w[1])                                               
    training.dCN_dp = -3*((CN_p[0,:] - CN_p[1,:]) / (roll_rate[0]-roll_rate[1]))                 
    training.dCN_dq = (CN_q[0,:] - CN_q[1,:]) / (pitch_rate[0]-pitch_rate[1])             
    training.dCN_dr = 3*((CN_r[0,:] - CN_r[1,:]) / (yaw_rate[0]-yaw_rate[1]))

    '''  for control surfaces, subtract inflence WITHOUT control surface deflected from coefficients WITH control surfaces'''
      
    for wing in aerodynamics.vehicle.wings: 
        for control_surface in wing.control_surfaces: 
            # --------------------------------------------------------------------------------------------------------------
            # Aileron 
            # --------------------------------------------------------------------------------------------------------------                   
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron:
                
                Clift_d_a      = np.zeros((len_d_a,len_Mach)) 
                Cdrag_d_a      = np.zeros((len_d_a,len_Mach)) 
                CX_d_a         = np.zeros((len_d_a,len_Mach)) 
                CY_d_a         = np.zeros((len_d_a,len_Mach)) 
                CZ_d_a         = np.zeros((len_d_a,len_Mach)) 
                CL_d_a         = np.zeros((len_d_a,len_Mach)) 
                CM_d_a         = np.zeros((len_d_a,len_Mach)) 
                CN_d_a         = np.zeros((len_d_a,len_Mach))
                
                for a_i in range(len_d_a):    
                    Machs                                           = np.atleast_2d(np.repeat(Mach,1)).T         
                    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
                    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
                    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
                    conditions.freestream.mach_number               = Machs    
                    vehicle.wings[wing.tag].control_surfaces.aileron.deflection =  delta_a[a_i]
                    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_, _= call_VLM(conditions,settings,vehicle)
                    vehicle.wings[wing.tag].control_surfaces.aileron.deflection = 0
                    
                    Clift_d_a[a_i,:] =  -(Clift_res[:,0]  - Clift_alpha_0[0,:])
                    Cdrag_d_a[a_i,:] =  -(Cdrag_res[:,0]  - Cdrag_alpha_0[0,:])                              
                    CX_d_a[a_i,:]    =  -(CX_res[:,0]   - CX_alpha_0[0,:]   )
                    CY_d_a[a_i,:]    =  -(CY_res[:,0]   - CY_alpha_0[0,:]  )
                    CZ_d_a[a_i,:]    =  -(CZ_res[:,0]   - CZ_alpha_0[0,:]  )
                    CL_d_a[a_i,:]    =  -(CL_res[:,0]   - CL_alpha_0[0,:])
                    CM_d_a[a_i,:]    =  -(CM_res[:,0]   - CM_alpha_0[0,:]  )
                    CN_d_a[a_i,:]    =  (CN_res[:,0]   - CN_alpha_0[0,:]  )
 
                training.Clift_delta_a  = Clift_d_a
                training.Cdrag_delta_a  = Cdrag_d_a 
                training.CX_delta_a     = CX_d_a 
                training.CY_delta_a     = CY_d_a 
                training.CZ_delta_a     = CZ_d_a 
                training.CL_delta_a     = CL_d_a 
                training.CM_delta_a     = CM_d_a 
                training.CN_delta_a     = CN_d_a
                
                training.dClift_ddelta_a = (Clift_d_a[0,:] - Clift_d_a[1,:]) / (delta_a[0] - delta_a[1])
                training.dCdrag_ddelta_a = (Cdrag_d_a[0,:] - Cdrag_d_a[1,:]) / (delta_a[0] - delta_a[1]) 
                training.dCX_ddelta_a    = (CX_d_a[0,:] - CX_d_a[1,:]) / (delta_a[0] - delta_a[1]) 
                training.dCY_ddelta_a    = (CY_d_a[0,:] - CY_d_a[1,:]) / (delta_a[0] - delta_a[1]) 
                training.dCZ_ddelta_a    = (CZ_d_a[0,:] - CZ_d_a[1,:]) / (delta_a[0] - delta_a[1]) 
                training.dCL_ddelta_a    = ((CL_d_a[0,:] - CL_d_a[1,:]) / (delta_a[0] - delta_a[1]))
                training.dCM_ddelta_a    = (CM_d_a[0,:] - CM_d_a[1,:]) / (delta_a[0] - delta_a[1]) 
                training.dCN_ddelta_a    = (CN_d_a[0,:] - CN_d_a[1,:]) / (delta_a[0] - delta_a[1]) 

            # --------------------------------------------------------------------------------------------------------------
            # Elevator 
            # --------------------------------------------------------------------------------------------------------------    
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator: 
            
                Clift_d_e      = np.zeros((len_d_e,len_Mach)) 
                Cdrag_d_e      = np.zeros((len_d_e,len_Mach)) 
                CX_d_e         = np.zeros((len_d_e,len_Mach)) 
                CY_d_e         = np.zeros((len_d_e,len_Mach)) 
                CZ_d_e         = np.zeros((len_d_e,len_Mach)) 
                CL_d_e         = np.zeros((len_d_e,len_Mach)) 
                CM_d_e         = np.zeros((len_d_e,len_Mach)) 
                CN_d_e         = np.zeros((len_d_e,len_Mach))
 
                for e_i in range(len_d_e): 
                    Machs                                           = np.atleast_2d(np.repeat(Mach,1)).T         
                    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
                    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
                    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
                    conditions.freestream.mach_number               = Machs     
                    vehicle.wings[wing.tag].control_surfaces.elevator.deflection =  delta_e[e_i]
                    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_, _= call_VLM(conditions,settings,vehicle)
                    vehicle.wings[wing.tag].control_surfaces.elevator.deflection = 0 
                    Clift_d_e[e_i,:] = Clift_res[:,0]  - Clift_alpha_0[0,:]
                    Cdrag_d_e[e_i,:] = Cdrag_res[:,0]  - Cdrag_alpha_0[0,:]                                
                    CX_d_e[e_i,:]    = CX_res[:,0]   - CX_alpha_0[0,:]   
                    CY_d_e[e_i,:]    = CY_res[:,0]   - CY_alpha_0[0,:]  
                    CZ_d_e[e_i,:]    = CZ_res[:,0]   - CZ_alpha_0[0,:]  
                    CL_d_e[e_i,:]    = CL_res[:,0]   - CL_alpha_0[0,:]  
                    CM_d_e[e_i,:]    = CM_res[:,0]   - CM_alpha_0[0,:]  
                    CN_d_e[e_i,:]    = CN_res[:,0]   - CN_alpha_0[0,:]     
                
                training.Clift_delta_e  = Clift_d_e
                training.Cdrag_delta_e  = Cdrag_d_e  
                training.CX_delta_e  = CX_d_e   
                training.CY_delta_e  = CY_d_e   
                training.CZ_delta_e  = CZ_d_e  
                training.CL_delta_e  = CL_d_e  
                training.CM_delta_e  = CM_d_e  
                training.CN_delta_e  = CN_d_e
                
                training.dClift_ddelta_e = 0.5*((Clift_d_e[0,:] - Clift_d_e[1,:]) / (delta_e[0] - delta_e[1]))
                training.dCdrag_ddelta_e = (Cdrag_d_e[0,:] - Cdrag_d_e[1,:]) / (delta_e[0] - delta_e[1])  
                training.dCX_ddelta_e = (CX_d_e[0,:] - CX_d_e[1,:]) / (delta_e[0] - delta_e[1])  
                training.dCY_ddelta_e = (CY_d_e[0,:] - CY_d_e[1,:]) / (delta_e[0] - delta_e[1]) 
                training.dCZ_ddelta_e = (CZ_d_e[0,:] - CZ_d_e[1,:]) / (delta_e[0] - delta_e[1]) 
                training.dCL_ddelta_e = (CL_d_e[0,:] - CL_d_e[1,:]) / (delta_e[0] - delta_e[1])  
                training.dCM_ddelta_e = (CM_d_e[0,:] - CM_d_e[1,:]) / (delta_e[0] - delta_e[1])  
                training.dCN_ddelta_e = (CN_d_e[0,:] - CN_d_e[1,:]) / (delta_e[0] - delta_e[1])
    
            # --------------------------------------------------------------------------------------------------------------
            # Rudder 
            # --------------------------------------------------------------------------------------------------------------  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder: 
                Clift_d_r      = np.zeros((len_d_r,len_Mach)) 
                Cdrag_d_r      = np.zeros((len_d_r,len_Mach)) 
                CX_d_r         = np.zeros((len_d_r,len_Mach)) 
                CY_d_r         = np.zeros((len_d_r,len_Mach)) 
                CZ_d_r         = np.zeros((len_d_r,len_Mach)) 
                CL_d_r         = np.zeros((len_d_r,len_Mach)) 
                CM_d_r         = np.zeros((len_d_r,len_Mach)) 
                CN_d_r         = np.zeros((len_d_r,len_Mach))
              
                for r_i in range(len_d_r): 
                    Machs                                           = np.atleast_2d(np.repeat(Mach,1)).T         
                    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
                    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
                    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
                    conditions.freestream.mach_number               = Machs    
                    vehicle.wings[wing.tag].control_surfaces.rudder.deflection =  delta_r[r_i]
                    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_, _= call_VLM(conditions,settings,vehicle)
                    vehicle.wings[wing.tag].control_surfaces.rudder.deflection = 0 
                    Clift_d_r[r_i,:] =   -(Clift_res[:,0]  - Clift_alpha_0[0,:])
                    Cdrag_d_r[r_i,:] =   -(Cdrag_res[:,0]  - Cdrag_alpha_0[0,:])                            
                    CX_d_r[r_i,:]    =   -(CX_res[:,0]   - CX_alpha_0[0,:]   )
                    CY_d_r[r_i,:]    =   -(CY_res[:,0]   - CY_alpha_0[0,:]  )
                    CZ_d_r[r_i,:]    =   -(CZ_res[:,0]   - CZ_alpha_0[0,:]  )
                    CL_d_r[r_i,:]    =   -(CL_res[:,0]   - CL_alpha_0[0,:]  )
                    CM_d_r[r_i,:]    =   -(CM_res[:,0]   - CM_alpha_0[0,:]  )
                    CN_d_r[r_i,:]    =   (CN_res[:,0]   - CN_alpha_0[0,:] )
                
                training.Clift_delta_r  = Clift_d_r
                training.Cdrag_delta_r  = Cdrag_d_r     
                training.CX_delta_r     = CX_d_r   
                training.CY_delta_r     = CY_d_r 
                training.CZ_delta_r     = CZ_d_r 
                training.CL_delta_r     = CL_d_r  
                training.CM_delta_r     = CM_d_r                   
                training.CN_delta_r     = CN_d_r    
                training.dClift_ddelta_r = (Clift_d_r[0,:] - Clift_d_r[1,:]) / (delta_r[0] - delta_r[1])
                training.dCdrag_ddelta_r = (Cdrag_d_r[0,:] - Cdrag_d_r[1,:]) / (delta_r[0] - delta_r[1])  
                training.dCX_ddelta_r    = (CX_d_r[0,:] - CX_d_r[1,:]) / (delta_r[0] - delta_r[1])   
                training.dCY_ddelta_r    = (CY_d_r[0,:] - CY_d_r[1,:]) / (delta_r[0] - delta_r[1]) 
                training.dCZ_ddelta_r    = (CZ_d_r[0,:] - CZ_d_r[1,:]) / (delta_r[0] - delta_r[1])  
                training.dCL_ddelta_r    = (CL_d_r[0,:] - CL_d_r[1,:]) / (delta_r[0] - delta_r[1])  
                training.dCM_ddelta_r    = (CM_d_r[0,:] - CM_d_r[1,:]) / (delta_r[0] - delta_r[1]) 
                training.dCN_ddelta_r    = (CN_d_r[0,:] - CN_d_r[1,:]) / (delta_r[0] - delta_r[1])  
                    
            # --------------------------------------------------------------------------------------------------------------
            # Flap
            # --------------------------------------------------------------------------------------------------------------  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap:
                Clift_d_f      = np.zeros((len_d_f,len_Mach)) 
                Cdrag_d_f      = np.zeros((len_d_f,len_Mach)) 
                CX_d_f         = np.zeros((len_d_f,len_Mach)) 
                CY_d_f         = np.zeros((len_d_f,len_Mach)) 
                CZ_d_f         = np.zeros((len_d_f,len_Mach)) 
                CL_d_f         = np.zeros((len_d_f,len_Mach)) 
                CM_d_f         = np.zeros((len_d_f,len_Mach)) 
                CN_d_f         = np.zeros((len_d_f,len_Mach))
                
                for f_i in range(len_d_f): 
                    Machs                                           = np.atleast_2d(np.repeat(Mach,1)).T         
                    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
                    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
                    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
                    conditions.freestream.mach_number               = Machs    
                    vehicle.wings[wing.tag].control_surfaces.flap.deflection = delta_f[f_i]
                    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_, _= call_VLM(conditions,settings,vehicle)
                    vehicle.wings[wing.tag].control_surfaces.flap.deflection = 0   
                    Clift_d_f[f_i,:] = Clift_res[:,0]  - Clift_alpha_0[0,:]
                    Cdrag_d_f[f_i,:] = Cdrag_res[:,0]  - Cdrag_alpha_0[0,:]                                
                    CX_d_f[f_i,:]    = CX_res[:,0]   - CX_alpha_0[0,:]   
                    CY_d_f[f_i,:]    = CY_res[:,0]   - CY_alpha_0[0,:]  
                    CZ_d_f[f_i,:]    = CZ_res[:,0]   - CZ_alpha_0[0,:]  
                    CL_d_f[f_i,:]    = CL_res[:,0]   - CL_alpha_0[0,:]  
                    CM_d_f[f_i,:]    = CM_res[:,0]   - CM_alpha_0[0,:]  
                    CN_d_f[f_i,:]    = CN_res[:,0]   - CN_alpha_0[0,:]         
 
                training.Clift_delta_f  = Clift_d_f    
                training.Cdrag_delta_f  = Cdrag_d_f    
                training.CX_delta_f     = CX_d_f  
                training.CY_delta_f     = CY_d_f  
                training.CZ_delta_f     = CZ_d_f  
                training.CL_delta_f     = CL_d_f  
                training.CM_delta_f     = CM_d_f                   
                training.CN_delta_f     = CN_d_f 
                training.dClift_ddelta_f = (Clift_d_f[0,:] - Clift_d_f[1,:]) / (delta_f[0] - delta_f[1])
                training.dCdrag_ddelta_f = (Cdrag_d_f[0,:] - Cdrag_d_f[1,:]) / (delta_f[0] - delta_f[1])  
                training.dCX_ddelta_f    = (CX_d_f[0,:] - CX_d_f[1,:]) / (delta_f[0] - delta_f[1])  
                training.dCY_ddelta_f    = (CY_d_f[0,:] - CY_d_f[1,:]) / (delta_f[0] - delta_f[1]) 
                training.dCZ_ddelta_f    = (CZ_d_f[0,:] - CZ_d_f[1,:]) / (delta_f[0] - delta_f[1])   
                training.dCL_ddelta_f    = (CL_d_f[0,:] - CL_d_f[1,:]) / (delta_f[0] - delta_f[1])  
                training.dCM_ddelta_f    = (CM_d_f[0,:] - CM_d_f[1,:]) / (delta_f[0] - delta_f[1])  
                training.dCN_ddelta_f    = (CN_d_f[0,:] - CN_d_f[1,:]) / (delta_f[0] - delta_f[1])   
                
            # --------------------------------------------------------------------------------------------------------------
            # Slat
            # --------------------------------------------------------------------------------------------------------------  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Slat:
                Clift_d_s      = np.zeros((len_d_s,len_Mach)) 
                Cdrag_d_s      = np.zeros((len_d_s,len_Mach)) 
                CX_d_s         = np.zeros((len_d_s,len_Mach)) 
                CY_d_s         = np.zeros((len_d_s,len_Mach)) 
                CZ_d_s         = np.zeros((len_d_s,len_Mach)) 
                CL_d_s         = np.zeros((len_d_s,len_Mach)) 
                CM_d_s         = np.zeros((len_d_s,len_Mach)) 
                CN_d_s         = np.zeros((len_d_s,len_Mach))
       
                for s_i in range(len_d_s):
                    Machs                                           = np.atleast_2d(np.repeat(Mach,1)).T         
                    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
                    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
                    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
                    conditions.freestream.mach_number               = Machs     
                    vehicle.wings[wing.tag].control_surfaces.slat.deflection = delta_s[s_i]
                    Clift_res,Cdrag_res,CX_res,CY_res,CZ_res,CL_res,CM_res,CN_res ,_,_,_,_,_,_,_,_,_,_,_, _= call_VLM(conditions,settings,vehicle)
                    vehicle.wings[wing.tag].control_surfaces.slat.deflection = 0 
                    Clift_d_s[s_i,:] = Clift_res[:,0]  - Clift_alpha_0[0,:]
                    Cdrag_d_s[s_i,:] = Cdrag_res[:,0]  - Cdrag_alpha_0[0,:]                                
                    CX_d_s[s_i,:]    = CX_res[:,0]   - CX_alpha_0[0,:]   
                    CY_d_s[s_i,:]    = CY_res[:,0]   - CY_alpha_0[0,:]  
                    CZ_d_s[s_i,:]    = CZ_res[:,0]   - CZ_alpha_0[0,:]  
                    CL_d_s[s_i,:]    = CL_res[:,0]   - CL_alpha_0[0,:]  
                    CM_d_s[s_i,:]    = CM_res[:,0]   - CM_alpha_0[0,:]  
                    CN_d_s[s_i,:]    = CN_res[:,0]   - CN_alpha_0[0,:] 
 
                training.Clift_delta_s   = Clift_d_s
                training.Cdrag_delta_s   = Cdrag_d_s
                training.CX_delta_s      = CX_d_s
                training.CY_delta_s      = CY_d_s 
                training.CZ_delta_s      = CZ_d_s 
                training.CL_delta_s      = CL_d_s 
                training.CM_delta_s      = CM_d_s
                training.CN_delta_s      = CN_d_s  
                training.dClift_ddelta_s = (Clift_d_s[0,:] - Clift_d_s[1,:]) / (delta_s[0] - delta_s[1])
                training.dCdrag_ddelta_s = (Cdrag_d_s[0,:] - Cdrag_d_s[1,:]) / (delta_s[0] - delta_s[1])
                training.dCX_ddelta_s    = (CX_d_s[0,:] - CX_d_s[1,:]) / (delta_s[0] - delta_s[1])
                training.dCY_ddelta_s    = (CY_d_s[0,:] - CY_d_s[1,:]) / (delta_s[0] - delta_s[1])
                training.dCZ_ddelta_s    = (CZ_d_s[0,:] - CZ_d_s[1,:]) / (delta_s[0] - delta_s[1]) 
                training.dCL_ddelta_s    = (CL_d_s[0,:] - CL_d_s[1,:]) / (delta_s[0] - delta_s[1]) 
                training.dCM_ddelta_s    = (CM_d_s[0,:] - CM_d_s[1,:]) / (delta_s[0] - delta_s[1])  
                training.dCN_ddelta_s    = (CN_d_s[0,:] - CN_d_s[1,:]) / (delta_s[0] - delta_s[1])
  
    training.NP            = 0  
    
    return training
        
        

def train_trasonic_model(aerodynamics, training_subsonic,training_supersonic,sub_Mach, sup_Mach): 
    """Sub function that call methods to run VLM for sample point evaluation. 
    
    Assumptions:
        None
        
    Source:
        None

    Args:
        aerodynamics       : VLM analysis          [unitless] 
        
    Returns: 
        None    
    """    

    vehicle        = aerodynamics.vehicle 
    AoA            = aerodynamics.training.angle_of_attack                  
    Beta           = aerodynamics.training.sideslip_angle
    training       = Data() 
    training.Mach  = np.array([sub_Mach[-1], sup_Mach[0]])
    u              = aerodynamics.training.u
    v              = aerodynamics.training.v
    w              = aerodynamics.training.w
    pitch_rate     = aerodynamics.training.pitch_rate
    roll_rate      = aerodynamics.training.roll_rate
    yaw_rate       = aerodynamics.training.yaw_rate  
    
    # --------------------------------------------------------------------------------------------------------------
    # Alpha
    # -------------------------------------------------------------------------------------------------------------- 
    
    Clift_alpha   =  np.concatenate((training_subsonic.Clift_alpha[:,-1][:,None] , training_supersonic.Clift_alpha[:,0][:,None] ), axis = 1)
    Cdrag_alpha   =  np.concatenate((training_subsonic.Cdrag_alpha[:,-1][:,None]  , training_supersonic.Cdrag_alpha[:,0][:,None] ), axis = 1) 
    CX_alpha      =  np.concatenate((training_subsonic.CX_alpha[:,-1][:,None]    , training_supersonic.CX_alpha[:,0][:,None] ), axis = 1)   
    CY_alpha      =  np.concatenate((training_subsonic.CY_alpha[:,-1][:,None]    , training_supersonic.CY_alpha[:,0][:,None] ), axis = 1)   
    CZ_alpha      =  np.concatenate((training_subsonic.CZ_alpha[:,-1][:,None]    , training_supersonic.CZ_alpha[:,0][:,None] ), axis = 1)   
    CL_alpha      =  np.concatenate((training_subsonic.CL_alpha[:,-1][:,None]    , training_supersonic.CL_alpha[:,0][:,None] ), axis = 1)   
    CM_alpha      =  np.concatenate((training_subsonic.CM_alpha[:,-1][:,None]    , training_supersonic.CM_alpha[:,0][:,None] ), axis = 1)   
    CN_alpha      =  np.concatenate((training_subsonic.CN_alpha[:,-1][:,None]    , training_supersonic.CN_alpha[:,0][:,None] ), axis = 1)   

    Clift_wing_alpha = Data()
    Cdrag_wing_alpha = Data() 
    for wing in vehicle.wings: 
        Clift_wing_alpha[wing.tag] =  np.concatenate((training_subsonic.Clift_wing_alpha[wing.tag][:,-1][:,None] , training_supersonic.Clift_wing_alpha[wing.tag][:,0][:,None] ), axis = 1)     
        Cdrag_wing_alpha[wing.tag] =  np.concatenate((training_subsonic.Cdrag_wing_alpha[wing.tag][:,-1][:,None] , training_supersonic.Cdrag_wing_alpha[wing.tag][:,0][:,None] ), axis = 1)     
    
    # --------------------------------------------------------------------------------------------------------------
    # Beta 
    # -------------------------------------------------------------------------------------------------------------- 
    
    Clift_beta =  np.concatenate((training_subsonic.Clift_beta[:,-1][:,None] , training_supersonic.Clift_beta[:,0][:,None] ), axis = 1)      
    Cdrag_beta =  np.concatenate((training_subsonic.Cdrag_beta[:,-1][:,None] , training_supersonic.Cdrag_beta[:,0][:,None] ), axis = 1)             
    CX_beta    =  np.concatenate((training_subsonic.CX_beta[:,-1][:,None]    , training_supersonic.CX_beta[:,0][:,None] ), axis = 1)        
    CY_beta    =  np.concatenate((training_subsonic.CY_beta[:,-1][:,None]    , training_supersonic.CY_beta[:,0][:,None] ), axis = 1)        
    CZ_beta    =  np.concatenate((training_subsonic.CZ_beta[:,-1][:,None]    , training_supersonic.CZ_beta[:,0][:,None] ), axis = 1)        
    CL_beta    =  np.concatenate((training_subsonic.CL_beta[:,-1][:,None]    , training_supersonic.CL_beta[:,0][:,None] ), axis = 1)        
    CM_beta    =  np.concatenate((training_subsonic.CM_beta[:,-1][:,None]    , training_supersonic.CM_beta[:,0][:,None] ), axis = 1)        
    CN_beta    =  np.concatenate((training_subsonic.CN_beta[:,-1][:,None]    , training_supersonic.CN_beta[:,0][:,None] ), axis = 1)        
 
    # -------------------------------------------------------      
    # Velocity u 
    # ------------------------------------------------------- 
    
    Clift_u     =   np.concatenate((training_subsonic.Clift_u[:,-1][:,None] , training_supersonic.Clift_u[:,0][:,None] ), axis = 1)     
    Cdrag_u     =   np.concatenate((training_subsonic.Cdrag_u[:,-1][:,None] , training_supersonic.Cdrag_u[:,0][:,None] ), axis = 1)     
    CX_u        =   np.concatenate((training_subsonic.CX_u[:,-1][:,None]    , training_supersonic.CX_u[:,0][:,None] ), axis = 1)        
    CY_u        =   np.concatenate((training_subsonic.CY_u[:,-1][:,None]    , training_supersonic.CY_u[:,0][:,None] ), axis = 1)        
    CZ_u        =   np.concatenate((training_subsonic.CZ_u[:,-1][:,None]    , training_supersonic.CZ_u[:,0][:,None] ), axis = 1)        
    CL_u        =   np.concatenate((training_subsonic.CL_u[:,-1][:,None]    , training_supersonic.CL_u[:,0][:,None] ), axis = 1)        
    CM_u        =   np.concatenate((training_subsonic.CM_u[:,-1][:,None]    , training_supersonic.CM_u[:,0][:,None] ), axis = 1)        
    CN_u        =   np.concatenate((training_subsonic.CN_u[:,-1][:,None]    , training_supersonic.CN_u[:,0][:,None] ), axis = 1)        
    
    # -------------------------------------------------------               
    # Velocity v 
    # -------------------------------------------------------  
    
    Clift_v     =  np.concatenate((training_subsonic.Clift_v[:,-1][:,None] , training_supersonic.Clift_v[:,0][:,None] ), axis = 1)       
    Cdrag_v     =  np.concatenate((training_subsonic.Cdrag_v[:,-1][:,None] , training_supersonic.Cdrag_v[:,0][:,None] ), axis = 1)       
    CX_v        =  np.concatenate((training_subsonic.CX_v[:,-1][:,None]    , training_supersonic.CX_v[:,0][:,None] ), axis = 1)          
    CY_v        =  np.concatenate((training_subsonic.CY_v[:,-1][:,None]    , training_supersonic.CY_v[:,0][:,None] ), axis = 1)          
    CZ_v        =  np.concatenate((training_subsonic.CZ_v[:,-1][:,None]    , training_supersonic.CZ_v[:,0][:,None] ), axis = 1)          
    CL_v        =  np.concatenate((training_subsonic.CL_v[:,-1][:,None]    , training_supersonic.CL_v[:,0][:,None] ), axis = 1)          
    CM_v        =  np.concatenate((training_subsonic.CM_v[:,-1][:,None]    , training_supersonic.CM_v[:,0][:,None] ), axis = 1)          
    CN_v        =  np.concatenate((training_subsonic.CN_v[:,-1][:,None]    , training_supersonic.CN_v[:,0][:,None] ), axis = 1)          
    
    # -------------------------------------------------------               
    # Velocity w 
    # -------------------------------------------------------  
    Clift_w     =  np.concatenate((training_subsonic.Clift_w[:,-1][:,None] , training_supersonic.Clift_w[:,0][:,None] ), axis = 1)   
    Cdrag_w     =  np.concatenate((training_subsonic.Cdrag_w[:,-1][:,None] , training_supersonic.Cdrag_w[:,0][:,None] ), axis = 1)   
    CX_w        =  np.concatenate((training_subsonic.CX_w[:,-1][:,None]    , training_supersonic.CX_w[:,0][:,None] ), axis = 1)      
    CY_w        =  np.concatenate((training_subsonic.CY_w[:,-1][:,None]    , training_supersonic.CY_w[:,0][:,None] ), axis = 1)      
    CZ_w        =  np.concatenate((training_subsonic.CZ_w[:,-1][:,None]    , training_supersonic.CZ_w[:,0][:,None] ), axis = 1)      
    CL_w        =  np.concatenate((training_subsonic.CL_w[:,-1][:,None]    , training_supersonic.CL_w[:,0][:,None] ), axis = 1)      
    CM_w        =  np.concatenate((training_subsonic.CM_w[:,-1][:,None]    , training_supersonic.CM_w[:,0][:,None] ), axis = 1)      
    CN_w        =  np.concatenate((training_subsonic.CN_w[:,-1][:,None]    , training_supersonic.CN_w[:,0][:,None] ), axis = 1)      

                    
    # -------------------------------------------------------               
    # Pitch Rate 
    # -------------------------------------------------------
    Clift_q     =  np.concatenate((training_subsonic.Clift_q[:,-1][:,None] , training_supersonic.Clift_q[:,0][:,None] ), axis = 1)     
    Cdrag_q     =  np.concatenate((training_subsonic.Cdrag_q[:,-1][:,None] , training_supersonic.Cdrag_q[:,0][:,None] ), axis = 1)     
    CX_q        =  np.concatenate((training_subsonic.CX_q[:,-1][:,None]    , training_supersonic.CX_q[:,0][:,None] ), axis = 1)        
    CY_q        =  np.concatenate((training_subsonic.CY_q[:,-1][:,None]    , training_supersonic.CY_q[:,0][:,None] ), axis = 1)        
    CZ_q        =  np.concatenate((training_subsonic.CZ_q[:,-1][:,None]    , training_supersonic.CZ_q[:,0][:,None] ), axis = 1)        
    CL_q        =  np.concatenate((training_subsonic.CL_q[:,-1][:,None]    , training_supersonic.CL_q[:,0][:,None] ), axis = 1)        
    CM_q        =  np.concatenate((training_subsonic.CM_q[:,-1][:,None]    , training_supersonic.CM_q[:,0][:,None] ), axis = 1)        
    CN_q        =  np.concatenate((training_subsonic.CN_q[:,-1][:,None]    , training_supersonic.CN_q[:,0][:,None] ), axis = 1)        
  
    # -------------------------------------------------------               
    # Roll  Rate 
    # -------------------------------------------------------     
    Clift_p     =  np.concatenate((training_subsonic.Clift_p[:,-1][:,None] , training_supersonic.Clift_p[:,0][:,None] ), axis = 1)    
    Cdrag_p     =  np.concatenate((training_subsonic.Cdrag_p[:,-1][:,None] , training_supersonic.Cdrag_p[:,0][:,None] ), axis = 1)    
    CX_p        =  np.concatenate((training_subsonic.CX_p[:,-1][:,None]    , training_supersonic.CX_p[:,0][:,None] ), axis = 1)       
    CY_p        =  np.concatenate((training_subsonic.CY_p[:,-1][:,None]    , training_supersonic.CY_p[:,0][:,None] ), axis = 1)       
    CZ_p        =  np.concatenate((training_subsonic.CZ_p[:,-1][:,None]    , training_supersonic.CZ_p[:,0][:,None] ), axis = 1)       
    CL_p        =  np.concatenate((training_subsonic.CL_p[:,-1][:,None]    , training_supersonic.CL_p[:,0][:,None] ), axis = 1)       
    CM_p        =  np.concatenate((training_subsonic.CM_p[:,-1][:,None]    , training_supersonic.CM_p[:,0][:,None] ), axis = 1)       
    CN_p        =  np.concatenate((training_subsonic.CN_p[:,-1][:,None]    , training_supersonic.CN_p[:,0][:,None] ), axis = 1)       


    # -------------------------------------------------------               
    # Yaw Rate 
    # -------------------------------------------------------         
    Clift_r     =  np.concatenate((training_subsonic.Clift_r[:,-1][:,None] , training_supersonic.Clift_r[:,0][:,None] ), axis = 1)      
    Cdrag_r     =  np.concatenate((training_subsonic.Cdrag_r[:,-1][:,None] , training_supersonic.Cdrag_r[:,0][:,None] ), axis = 1)      
    CX_r        =  np.concatenate((training_subsonic.CX_r[:,-1][:,None]    , training_supersonic.CX_r[:,0][:,None] ), axis = 1)         
    CY_r        =  np.concatenate((training_subsonic.CY_r[:,-1][:,None]    , training_supersonic.CY_r[:,0][:,None] ), axis = 1)         
    CZ_r        =  np.concatenate((training_subsonic.CZ_r[:,-1][:,None]    , training_supersonic.CZ_r[:,0][:,None] ), axis = 1)         
    CL_r        =  np.concatenate((training_subsonic.CL_r[:,-1][:,None]    , training_supersonic.CL_r[:,0][:,None] ), axis = 1)         
    CM_r        =  np.concatenate((training_subsonic.CM_r[:,-1][:,None]    , training_supersonic.CM_r[:,0][:,None] ), axis = 1)         
    CN_r        =  np.concatenate((training_subsonic.CN_r[:,-1][:,None]    , training_supersonic.CN_r[:,0][:,None] ), axis = 1)         
 
    # STABILITY COEFFICIENTS 
    training.Clift_wing_alpha  = Clift_wing_alpha   

    training.Clift_alpha       = Clift_alpha   
    training.Clift_beta        = Clift_beta
                    
    training.Clift_u           = Clift_u       
    training.Clift_v           = Clift_v       
    training.Clift_w           = Clift_w       
    training.Clift_p           = Clift_p       
    training.Clift_q           = Clift_q       
    training.Clift_r           = Clift_r       
    training.Cdrag_alpha       = Cdrag_alpha 
    training.Cdrag_wing_alpha  = Cdrag_wing_alpha    
    training.Cdrag_beta        = Cdrag_beta 
    training.Cdrag_u           = Cdrag_u       
    training.Cdrag_v           = Cdrag_v       
    training.Cdrag_w           = Cdrag_w       
    training.Cdrag_p           = Cdrag_p        
    training.Cdrag_q           = Cdrag_q       
    training.Cdrag_r           = Cdrag_r         
    training.CX_alpha          = CX_alpha      
    training.CX_beta           = CX_beta 
                       
    training.CX_u              = CX_u          
    training.CX_v              = CX_v          
    training.CX_w              = CX_w          
    training.CX_p              = CX_p           
    training.CX_q              = CX_q          
    training.CX_r              = CX_r            
    training.CY_alpha          = CY_alpha      
    training.CY_beta           = CY_beta
         
                        
    training.CY_u              = CY_u          
    training.CY_v              = CY_v          
    training.CY_w              = CY_w          
    training.CY_p              = CY_p            
    training.CY_q              = CY_q           
    training.CY_r              = CY_r             
    training.CZ_alpha          = CZ_alpha      
    training.CZ_beta           = CZ_beta
          
    training.CZ_u              = CZ_u          
    training.CZ_v              = CZ_v          
    training.CZ_w              = CZ_w          
    training.CZ_p              = CZ_p           
    training.CZ_q              = CZ_q          
    training.CZ_r              = CZ_r            
    training.CL_alpha          = CL_alpha      
    training.CL_beta           = CL_beta
                
    training.CL_u              = CL_u          
    training.CL_v              = CL_v          
    training.CL_w              = CL_w          
    training.CL_p              = CL_p           
    training.CL_q              = CL_q          
    training.CL_r              = CL_r            
    training.CM_alpha          = CM_alpha      
    training.CM_beta           = CM_beta 
                   
    training.CM_u              = CM_u          
    training.CM_v              = CM_v          
    training.CM_w              = CM_w          
    training.CM_p              = CM_p             
    training.CM_q              = CM_q            
    training.CM_r              = CM_r              
    training.CN_alpha          = CN_alpha      
    training.CN_beta           = CN_beta 
                   
    training.CN_u              = CN_u          
    training.CN_v              = CN_v          
    training.CN_w              = CN_w          
    training.CN_p              = CN_p            
    training.CN_q              = CN_q           
    training.CN_r              = CN_r
      
            
    # STABILITY DERIVATIVES 
    training.dClift_dalpha = (Clift_alpha[0,:] - Clift_alpha[1,:]) / (AoA[0] - AoA[1])
    training.dClift_dbeta  = (Clift_beta[0,:] - Clift_beta[1,:]) / (Beta[0] - Beta[1]) 
    training.dClift_du     = (Clift_u[0,:] - Clift_u[1,:]) / (u[0] - u[1])            
    training.dClift_dv     = (Clift_v[0,:] - Clift_v[1,:]) / (v[0] - v[1])          
    training.dClift_dw     = (Clift_w[0,:] - Clift_w[1,:]) / (w[0] - w[1])         
    training.dClift_dp     = (Clift_p[0,:] - Clift_p[1,:]) / (roll_rate[0]-roll_rate[1])            
    training.dClift_dq     = (Clift_q[0,:] - Clift_q[1,:]) / (pitch_rate[0]-pitch_rate[1])        
    training.dClift_dr     = (Clift_r[0,:] - Clift_r[1,:]) / (yaw_rate[0]-yaw_rate[1])                
    training.dCdrag_dalpha = (Cdrag_alpha[0,:] - Cdrag_alpha[1,:]) / (AoA[0] - AoA[1])    
    training.dCdrag_dbeta  = (Cdrag_beta[0,:] - Cdrag_beta[1,:]) / (Beta[0] - Beta[1])
                
    training.dCdrag_du     = (Cdrag_u[0,:] - Cdrag_u[1,:]) / (u[0] - u[1])                     
    training.dCdrag_dv     = (Cdrag_v[0,:] - Cdrag_v[1,:]) / (v[0] - v[1])                   
    training.dCdrag_dw     = (Cdrag_w[0,:] - Cdrag_w[1,:]) / (w[0] - w[1])                  
    training.dCdrag_dp     = (Cdrag_p[0,:] - Cdrag_p[1,:]) / (roll_rate[0]-roll_rate[1])             
    training.dCdrag_dq     = (Cdrag_q[0,:] - Cdrag_q[1,:]) / (pitch_rate[0]-pitch_rate[1])         
    training.dCdrag_dr     = (Cdrag_r[0,:] - Cdrag_r[1,:]) / (yaw_rate[0]-yaw_rate[1])                 
    training.dCX_dalpha    = (CX_alpha[0,:] - CX_alpha[1,:]) / (AoA[0] - AoA[1])            
    training.dCX_dbeta     = (CX_beta[0,:] - CX_beta[1,:]) / (Beta[0] - Beta[1]) 
                
    training.dCX_du        = (CX_u[0,:] - CX_u[1,:]) / (u[0] - u[1])                                 
    training.dCX_dv        = (CX_v[0,:] - CX_v[1,:]) / (v[0] - v[1])                               
    training.dCX_dw        = (CX_w[0,:] - CX_w[1,:]) / (w[0] - w[1])                              
    training.dCX_dp        = (CX_p[0,:] - CX_p[1,:]) / (roll_rate[0]-roll_rate[1])                
    training.dCX_dq        = (CX_q[0,:] - CX_q[1,:]) / (pitch_rate[0]-pitch_rate[1])            
    training.dCX_dr        = (CX_r[0,:] - CX_r[1,:]) / (yaw_rate[0]-yaw_rate[1])                    
    training.dCY_dalpha    = (CY_alpha[0,:] - CY_alpha[1,:]) / (AoA[0] - AoA[1])         
    training.dCY_dbeta     = (CY_beta[0,:] - CY_beta[1,:]) / (Beta[0] - Beta[1]) 
            
                
    training.dCY_du     = (CY_u[0,:] - CY_u[1,:]) / (u[0] - u[1])                                             
    training.dCY_dv     = (CY_v[0,:] - CY_v[1,:]) / (v[0] - v[1])                                           
    training.dCY_dw     = (CY_w[0,:] - CY_w[1,:]) / (w[0] - w[1])                                          
    training.dCY_dp     = (CY_p[0,:] - CY_p[1,:]) / (roll_rate[0]-roll_rate[1])                 
    training.dCY_dq     = (CY_q[0,:] - CY_q[1,:]) / (pitch_rate[0]-pitch_rate[1])             
    training.dCY_dr     = (CY_r[0,:] - CY_r[1,:]) / (yaw_rate[0]-yaw_rate[1])                     
    training.dCZ_dalpha = (CZ_alpha[0,:] - CZ_alpha[1,:]) / (AoA[0] - AoA[1])             
    training.dCZ_dbeta  = (CZ_beta[0,:] - CZ_beta[1,:]) / (Beta[0] - Beta[1])
    
                      
    training.dCZ_du     = (CZ_u[0,:] - CZ_u[1,:]) / (u[0] - u[1])                                              
    training.dCZ_dv     = (CZ_v[0,:] - CZ_v[1,:]) / (v[0] - v[1])                                              
    training.dCZ_dw     = (CZ_w[0,:] - CZ_w[1,:]) / (w[0] - w[1])                                              
    training.dCZ_dp     = (CZ_p[0,:] - CZ_p[1,:]) / (roll_rate[0]-roll_rate[1])                
    training.dCZ_dq     = (CZ_q[0,:] - CZ_q[1,:]) / (pitch_rate[0]-pitch_rate[1])            
    training.dCZ_dr     = (CZ_r[0,:] - CZ_r[1,:]) / (yaw_rate[0]-yaw_rate[1])                    
    training.dCL_dalpha = (CL_alpha[0,:] - CL_alpha[1,:]) / (AoA[0] - AoA[1])         
    training.dCL_dbeta  = (CL_beta[0,:] - CL_beta[1,:]) / (Beta[0] - Beta[1])                
                
                
    training.dCL_du     = (CL_u[0,:] - CL_u[1,:]) / (u[0] - u[1])                                              
    training.dCL_dv     = (CL_v[0,:] - CL_v[1,:]) / (v[0] - v[1])                                              
    training.dCL_dw     = (CL_w[0,:] - CL_w[1,:]) / (w[0] - w[1])                                              
    training.dCL_dp     = (CL_p[0,:] - CL_p[1,:]) / (roll_rate[0]-roll_rate[1])                
    training.dCL_dq     = (CL_q[0,:] - CL_q[1,:]) / (pitch_rate[0]-pitch_rate[1])            
    training.dCL_dr     = (CL_r[0,:] - CL_r[1,:]) / (yaw_rate[0]-yaw_rate[1])                    
    training.dCM_dalpha = (CM_alpha[0,:] - CM_alpha[1,:]) / (AoA[0] - AoA[1])          
    training.dCM_dbeta  = (CM_beta[0,:] - CM_beta[1,:]) / (Beta[0] - Beta[1])  
                
    training.dCM_du     = (CM_u[0,:] - CM_u[1,:]) / (u[0] - u[1])                                               
    training.dCM_dv     = (CM_v[0,:] - CM_v[1,:]) / (v[0] - v[1])                                               
    training.dCM_dw     = (CM_w[0,:] - CM_w[1,:]) / (w[0] - w[1])                                               
    training.dCM_dp     = (CM_p[0,:] - CM_p[1,:]) / (roll_rate[0]-roll_rate[1])                 
    training.dCM_dq     = (CM_q[0,:] - CM_q[1,:]) / (pitch_rate[0]-pitch_rate[1])             
    training.dCM_dr     = (CM_r[0,:] - CM_r[1,:]) / (yaw_rate[0]-yaw_rate[1])                     
    training.dCN_dalpha = (CN_alpha[0,:] - CN_alpha[1,:]) / (AoA[0] - AoA[1])          
    training.dCN_dbeta  = (CN_beta[0,:] - CN_beta[1,:]) / (Beta[0] - Beta[1]) 
                     
    training.dCN_du = (CN_u[0,:] - CN_u[1,:]) / (u[0] - u[1])                                               
    training.dCN_dv = (CN_v[0,:] - CN_v[1,:]) / (v[0] - v[1])                                               
    training.dCN_dw = (CN_w[0,:] - CN_w[1,:]) / (w[0] - w[1])                                               
    training.dCN_dp = (CN_p[0,:] - CN_p[1,:]) / (roll_rate[0]-roll_rate[1])                 
    training.dCN_dq = (CN_q[0,:] - CN_q[1,:]) / (pitch_rate[0]-pitch_rate[1])             
    training.dCN_dr = (CN_r[0,:] - CN_r[1,:]) / (yaw_rate[0]-yaw_rate[1])


    '''  for control surfaces, subtract inflence WITHOUT control surface deflected from coefficients WITH control surfaces'''
     
    # --------------------------------------------------------------------------------------------------------------
    # Aileron 
    # --------------------------------------------------------------------------------------------------------------   
    if aerodynamics.aileron_flag:   
        training.Clift_delta_a   =  np.concatenate((training_subsonic.Clift_delta_a[:,-1][:,None] , training_supersonic.Clift_delta_a[:,0][:,None] ), axis = 1) 
        training.Cdrag_delta_a   =  np.concatenate((training_subsonic.Cdrag_delta_a[:,-1][:,None] , training_supersonic.Cdrag_delta_a[:,0][:,None] ), axis = 1) 
        training.CX_delta_a      =  np.concatenate((training_subsonic.CX_delta_a[:,-1][:,None]    , training_supersonic.CX_delta_a[:,0][:,None]    ), axis = 1) 
        training.CY_delta_a      =  np.concatenate((training_subsonic.CY_delta_a[:,-1][:,None]    , training_supersonic.CY_delta_a[:,0][:,None]    ), axis = 1) 
        training.CZ_delta_a      =  np.concatenate((training_subsonic.CZ_delta_a[:,-1][:,None]    , training_supersonic.CZ_delta_a[:,0][:,None]    ), axis = 1) 
        training.CL_delta_a      =  np.concatenate((training_subsonic.CL_delta_a[:,-1][:,None]    , training_supersonic.CL_delta_a[:,0][:,None]    ), axis = 1) 
        training.CM_delta_a      =  np.concatenate((training_subsonic.CM_delta_a[:,-1][:,None]    , training_supersonic.CM_delta_a[:,0][:,None]    ), axis = 1) 
        training.CN_delta_a      =  np.concatenate((training_subsonic.CN_delta_a[:,-1][:,None]    , training_supersonic.CN_delta_a[:,0][:,None]    ), axis = 1) 
        training.dClift_ddelta_a =  np.array([training_subsonic.dClift_ddelta_a[-1] , training_subsonic.dClift_ddelta_a[0]])
        training.dCdrag_ddelta_a =  np.array([training_subsonic.dCdrag_ddelta_a[-1] , training_subsonic.dCdrag_ddelta_a[0]])
        training.dCX_ddelta_a    =  np.array([training_subsonic.dCX_ddelta_a[-1]    , training_subsonic.dCX_ddelta_a[0]   ])
        training.dCY_ddelta_a    =  np.array([training_subsonic.dCY_ddelta_a[-1]    , training_subsonic.dCY_ddelta_a[0]   ])
        training.dCZ_ddelta_a    =  np.array([training_subsonic.dCZ_ddelta_a[-1]    , training_subsonic.dCZ_ddelta_a[0]   ])
        training.dCL_ddelta_a    =  np.array([training_subsonic.dCL_ddelta_a[-1]    , training_subsonic.dCL_ddelta_a[0]   ])
        training.dCM_ddelta_a    =  np.array([training_subsonic.dCM_ddelta_a[-1]    , training_subsonic.dCM_ddelta_a[0]   ])
        training.dCN_ddelta_a    =  np.array([training_subsonic.dCN_ddelta_a[-1]    , training_subsonic.dCN_ddelta_a[0]   ])
    
    # --------------------------------------------------------------------------------------------------------------
    # Elevator 
    # -------------------------------------------------------------------------------------------------------------- 
    if aerodynamics.elevator_flag:  
                    
        training.Clift_delta_e   =  np.concatenate((training_subsonic.Clift_delta_e[:,-1][:,None] , training_supersonic.Clift_delta_e[:,0][:,None] ), axis = 1)   
        training.Cdrag_delta_e   =  np.concatenate((training_subsonic.Cdrag_delta_e[:,-1][:,None] , training_supersonic.Cdrag_delta_e[:,0][:,None] ), axis = 1)   
        training.CX_delta_e      =  np.concatenate((training_subsonic.CX_delta_e[:,-1][:,None]    , training_supersonic.CX_delta_e[:,0][:,None]    ), axis = 1)   
        training.CY_delta_e      =  np.concatenate((training_subsonic.CY_delta_e[:,-1][:,None]    , training_supersonic.CY_delta_e[:,0][:,None]    ), axis = 1)   
        training.CZ_delta_e      =  np.concatenate((training_subsonic.CZ_delta_e[:,-1][:,None]    , training_supersonic.CZ_delta_e[:,0][:,None]    ), axis = 1)   
        training.CL_delta_e      =  np.concatenate((training_subsonic.CL_delta_e[:,-1][:,None]    , training_supersonic.CL_delta_e[:,0][:,None]    ), axis = 1)   
        training.CM_delta_e      =  np.concatenate((training_subsonic.CM_delta_e[:,-1][:,None]    , training_supersonic.CM_delta_e[:,0][:,None]    ), axis = 1)   
        training.CN_delta_e      =  np.concatenate((training_subsonic.CN_delta_e[:,-1][:,None]    , training_supersonic.CN_delta_e[:,0][:,None]    ), axis = 1)   
        training.dClift_ddelta_e =  np.array([training_subsonic.dClift_ddelta_e[-1] , training_subsonic.dClift_ddelta_e[0]])
        training.dCdrag_ddelta_e =  np.array([training_subsonic.dCdrag_ddelta_e[-1] , training_subsonic.dCdrag_ddelta_e[0]])
        training.dCX_ddelta_e    =  np.array([training_subsonic.dCX_ddelta_e[-1]    , training_subsonic.dCX_ddelta_e[0]   ])
        training.dCY_ddelta_e    =  np.array([training_subsonic.dCY_ddelta_e[-1]    , training_subsonic.dCY_ddelta_e[0]   ])
        training.dCZ_ddelta_e    =  np.array([training_subsonic.dCZ_ddelta_e[-1]    , training_subsonic.dCZ_ddelta_e[0]   ])
        training.dCL_ddelta_e    =  np.array([training_subsonic.dCL_ddelta_e[-1]    , training_subsonic.dCL_ddelta_e[0]   ])
        training.dCM_ddelta_e    =  np.array([training_subsonic.dCM_ddelta_e[-1]    , training_subsonic.dCM_ddelta_e[0]   ])
        training.dCN_ddelta_e    =  np.array([training_subsonic.dCN_ddelta_e[-1]    , training_subsonic.dCN_ddelta_e[0]   ])
        
    # --------------------------------------------------------------------------------------------------------------
    # Rudder 
    # -------------------------------------------------------------------------------------------------------------- 
    if aerodynamics.rudder_flag: 
        training.Clift_delta_r   =  np.concatenate((training_subsonic.Clift_delta_r[:,-1][:,None] , training_supersonic.Clift_delta_r[:,0][:,None] ), axis = 1)  
        training.Cdrag_delta_r   =  np.concatenate((training_subsonic.Cdrag_delta_r[:,-1][:,None] , training_supersonic.Cdrag_delta_r[:,0][:,None] ), axis = 1)  
        training.CX_delta_r      =  np.concatenate((training_subsonic.CX_delta_r[:,-1][:,None]    , training_supersonic.CX_delta_r[:,0][:,None]    ), axis = 1)  
        training.CY_delta_r      =  np.concatenate((training_subsonic.CY_delta_r[:,-1][:,None]    , training_supersonic.CY_delta_r[:,0][:,None]    ), axis = 1)  
        training.CZ_delta_r      =  np.concatenate((training_subsonic.CZ_delta_r[:,-1][:,None]    , training_supersonic.CZ_delta_r[:,0][:,None]    ), axis = 1)  
        training.CL_delta_r      =  np.concatenate((training_subsonic.CL_delta_r[:,-1][:,None]    , training_supersonic.CL_delta_r[:,0][:,None]    ), axis = 1)  
        training.CM_delta_r      =  np.concatenate((training_subsonic.CM_delta_r[:,-1][:,None]    , training_supersonic.CM_delta_r[:,0][:,None]    ), axis = 1)  
        training.CN_delta_r      =  np.concatenate((training_subsonic.CN_delta_r[:,-1][:,None]    , training_supersonic.CN_delta_r[:,0][:,None]    ), axis = 1)  
        training.dClift_ddelta_r =  np.array([training_subsonic.dClift_ddelta_r[-1] , training_subsonic.dClift_ddelta_r[0]])
        training.dCdrag_ddelta_r =  np.array([training_subsonic.dCdrag_ddelta_r[-1] , training_subsonic.dCdrag_ddelta_r[0]])
        training.dCX_ddelta_r    =  np.array([training_subsonic.dCX_ddelta_r[-1]    , training_subsonic.dCX_ddelta_r[0]   ])
        training.dCY_ddelta_r    =  np.array([training_subsonic.dCY_ddelta_r[-1]    , training_subsonic.dCY_ddelta_r[0]   ])
        training.dCZ_ddelta_r    =  np.array([training_subsonic.dCZ_ddelta_r[-1]    , training_subsonic.dCZ_ddelta_r[0]   ])
        training.dCL_ddelta_r    =  np.array([training_subsonic.dCL_ddelta_r[-1]    , training_subsonic.dCL_ddelta_r[0]   ])
        training.dCM_ddelta_r    =  np.array([training_subsonic.dCM_ddelta_r[-1]    , training_subsonic.dCM_ddelta_r[0]   ])
        training.dCN_ddelta_r    =  np.array([training_subsonic.dCN_ddelta_r[-1]    , training_subsonic.dCN_ddelta_r[0]   ])
                        
    # --------------------------------------------------------------------------------------------------------------
    # Flap
    # -------------------------------------------------------------------------------------------------------------- 
    if aerodynamics.flap_flag: 
        training.Clift_delta_f   = np.concatenate((training_subsonic.Clift_delta_f[:,-1][:,None] , training_supersonic.Clift_delta_f[:,0][:,None] ), axis = 1)   
        training.Cdrag_delta_f   = np.concatenate((training_subsonic.Cdrag_delta_f[:,-1][:,None] , training_supersonic.Cdrag_delta_f[:,0][:,None] ), axis = 1)   
        training.CX_delta_f      = np.concatenate((training_subsonic.CX_delta_f[:,-1][:,None]    , training_supersonic.CX_delta_f[:,0][:,None]    ), axis = 1)   
        training.CY_delta_f      = np.concatenate((training_subsonic.CY_delta_f[:,-1][:,None]    , training_supersonic.CY_delta_f[:,0][:,None]    ), axis = 1)   
        training.CZ_delta_f      = np.concatenate((training_subsonic.CZ_delta_f[:,-1][:,None]    , training_supersonic.CZ_delta_f[:,0][:,None]    ), axis = 1)   
        training.CL_delta_f      = np.concatenate((training_subsonic.CL_delta_f[:,-1][:,None]    , training_supersonic.CL_delta_f[:,0][:,None]    ), axis = 1)   
        training.CM_delta_f      = np.concatenate((training_subsonic.CM_delta_f[:,-1][:,None]    , training_supersonic.CM_delta_f[:,0][:,None]    ), axis = 1)   
        training.CN_delta_f      = np.concatenate((training_subsonic.CN_delta_f[:,-1][:,None]    , training_supersonic.CN_delta_f[:,0][:,None]    ), axis = 1)   
        training.dClift_ddelta_f = np.array([training_subsonic.dClift_ddelta_f[-1] , training_subsonic.dClift_ddelta_f[0]])
        training.dCdrag_ddelta_f = np.array([training_subsonic.dCdrag_ddelta_f[-1] , training_subsonic.dCdrag_ddelta_f[0]])
        training.dCX_ddelta_f    = np.array([training_subsonic.dCX_ddelta_f[-1]    , training_subsonic.dCX_ddelta_f[0]   ])
        training.dCY_ddelta_f    = np.array([training_subsonic.dCY_ddelta_f[-1]    , training_subsonic.dCY_ddelta_f[0]   ])
        training.dCZ_ddelta_f    = np.array([training_subsonic.dCZ_ddelta_f[-1]    , training_subsonic.dCZ_ddelta_f[0]   ])
        training.dCL_ddelta_f    = np.array([training_subsonic.dCL_ddelta_f[-1]    , training_subsonic.dCL_ddelta_f[0]   ])
        training.dCM_ddelta_f    = np.array([training_subsonic.dCM_ddelta_f[-1]    , training_subsonic.dCM_ddelta_f[0]   ])
        training.dCN_ddelta_f    = np.array([training_subsonic.dCN_ddelta_f[-1]    , training_subsonic.dCN_ddelta_f[0]   ])
                    
    # --------------------------------------------------------------------------------------------------------------
    # Slat
    # -------------------------------------------------------------------------------------------------------------- 
    if aerodynamics.slat_flag: 
        training.Clift_delta_s   =  np.concatenate((training_subsonic.Clift_delta_s[:,-1][:,None] , training_supersonic.Clift_delta_s[:,0][:,None] ), axis = 1)   
        training.Cdrag_delta_s   =  np.concatenate((training_subsonic.Cdrag_delta_s[:,-1][:,None] , training_supersonic.Cdrag_delta_s[:,0][:,None] ), axis = 1)   
        training.CX_delta_s      =  np.concatenate((training_subsonic.CX_delta_s[:,-1][:,None]    , training_supersonic.CX_delta_s[:,0][:,None]    ), axis = 1)   
        training.CY_delta_s      =  np.concatenate((training_subsonic.CY_delta_s[:,-1][:,None]    , training_supersonic.CY_delta_s[:,0][:,None]    ), axis = 1)   
        training.CZ_delta_s      =  np.concatenate((training_subsonic.CZ_delta_s[:,-1][:,None]    , training_supersonic.CZ_delta_s[:,0][:,None]    ), axis = 1)   
        training.CL_delta_s      =  np.concatenate((training_subsonic.CL_delta_s[:,-1][:,None]    , training_supersonic.CL_delta_s[:,0][:,None]    ), axis = 1)   
        training.CM_delta_s      =  np.concatenate((training_subsonic.CM_delta_s[:,-1][:,None]    , training_supersonic.CM_delta_s[:,0][:,None]    ), axis = 1)   
        training.CN_delta_s      =  np.concatenate((training_subsonic.CN_delta_s[:,-1][:,None]    , training_supersonic.CN_delta_s[:,0][:,None]    ), axis = 1)   
        training.dClift_ddelta_s =  np.array([training_subsonic.dClift_ddelta_s[-1] , training_subsonic.dClift_ddelta_s[0]])
        training.dCdrag_ddelta_s =  np.array([training_subsonic.dCdrag_ddelta_s[-1] , training_subsonic.dCdrag_ddelta_s[0]])
        training.dCX_ddelta_s    =  np.array([training_subsonic.dCX_ddelta_s[-1]    , training_subsonic.dCX_ddelta_s[0]   ])
        training.dCY_ddelta_s    =  np.array([training_subsonic.dCY_ddelta_s[-1]    , training_subsonic.dCY_ddelta_s[0]   ])
        training.dCZ_ddelta_s    =  np.array([training_subsonic.dCZ_ddelta_s[-1]    , training_subsonic.dCZ_ddelta_s[0]   ])
        training.dCL_ddelta_s    =  np.array([training_subsonic.dCL_ddelta_s[-1]    , training_subsonic.dCL_ddelta_s[0]   ])
        training.dCM_ddelta_s    =  np.array([training_subsonic.dCM_ddelta_s[-1]    , training_subsonic.dCM_ddelta_s[0]   ])
        training.dCN_ddelta_s    =  np.array([training_subsonic.dCN_ddelta_s[-1]    , training_subsonic.dCN_ddelta_s[0]   ])
    training.NP            = 0  
    
    return training
        
# ----------------------------------------------------------------------
#  Evaluate VLM
# ----------------------------------------------------------------------
def call_VLM(conditions,settings,vehicle):
    """Calculate aerodynamics coefficients inluding specific wing coefficients using the VLM
        
    Assumptions:
        None
        
    Source:
        None

    Args: 
        conditions : flight conditions     [unitless]
        settings   : VLM analysis settings [unitless]
        vehicle    : vehicle configuration [unitless] 
        
    Returns: 
        None  
    """
 
    Clift_wings         = Data()
    Cdrag_wings         = Data()
    AoA_wing_induced    = Data()
    
    results = VLM(conditions,settings,vehicle)
    Clift   = results.CL       
    Cdrag   = results.CDi     
    Clift_w = results.CL_wing        
    Cdrag_w = results.CDi_wing       
    CX      = results.CX       
    CY      = results.CY       
    CZ      = results.CZ       
    CL      = results.CL_mom
    alpha_i = results.alpha_i 
    CM      = results.CM       
    CN      = results.CN 
    S_ref   = results.S_ref    
    b_ref   = results.b_ref    
    c_ref   = results.c_ref    
    X_ref   = results.X_ref    
    Y_ref   = results.Y_ref    
    Z_ref   = results.Z_ref
    cl_y    = results.cl_y     
    cdi_y   = results.cdi_y 
    CPi     = results.CP 
    

    # Dimensionalize the lift and drag for each wing
    areas          = vehicle.vortex_distribution.wing_areas
    dim_wing_lifts = Clift_w  * areas
    dim_wing_drags = Cdrag_w * areas
    
    i = 0
    # Assign the lift and drag and non-dimensionalize
    for wing in vehicle.wings.values():
        ref = wing.areas.reference
        if wing.symmetric:
            Clift_wings[wing.tag]      = np.atleast_2d(np.sum(dim_wing_lifts[:,i:(i+2)],axis=1)).T/ref
            Cdrag_wings[wing.tag]      = np.atleast_2d(np.sum(dim_wing_drags[:,i:(i+2)],axis=1)).T/ref
            AoA_wing_induced[wing.tag] = np.concatenate((alpha_i[i],alpha_i[i+1]),axis=1)
            i+=1
        else:
            Clift_wings[wing.tag]      = np.atleast_2d(dim_wing_lifts[:,i]).T/ref
            Cdrag_wings[wing.tag]      = np.atleast_2d(dim_wing_drags[:,i]).T/ref
            AoA_wing_induced[wing.tag] = alpha_i[i]
        i+=1
    
    return Clift,Cdrag,CX,CY,CZ,CL,CM,CN, S_ref,b_ref,c_ref,X_ref,Y_ref ,Z_ref, Clift_wings,Cdrag_wings,AoA_wing_induced,cl_y,cdi_y,CPi  

        