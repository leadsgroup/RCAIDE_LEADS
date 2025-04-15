# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/train_VLM_surrogates.py
#  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
import RCAIDE 
from RCAIDE.Framework.Core import  Data 
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM   import VLM 
from copy import deepcopy

# package imports
import numpy  as np

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
    
    # only build supersonic surrogates if necessary
    if len(sup_Mach) > 2: 
        training.supersonic  =  train_model(aerodynamics, sup_Mach)
        training.transonic   =  train_trasonic_model(aerodynamics, training.subsonic,training.supersonic,sub_Mach, sup_Mach)
    else:
        training.supersonic  = None
        training.transonic   = None

    # compute neutral point
     # --------------------------------------------------------------------------------------------      
    # Equilibrium Condition 
    # --------------------------------------------------------------------------------------------       


    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
    conditions.freestream.mach_number               = np.array([[0.5]])
    conditions.aerodynamics.angles.alpha            = np.array([[0.0]])


    np_vehicle = deepcopy(aerodynamics.vehicle)
    np_settings = deepcopy(aerodynamics.settings) 
    VLM_results_0 = VLM(conditions,np_settings,np_vehicle) 
    CM_0    = VLM_results_0.CM  

    delta_angle  = aerodynamics.training.angle_purtubation  
    conditions.aerodynamics.angles.alpha   += delta_angle 
    VLM_results_1 = VLM(conditions,np_settings,np_vehicle) 
    CM_alpha_prime    = VLM_results_1.CM 
    
    vehicle_shifted_CG = deepcopy(aerodynamics.vehicle)
    delta_cg = 0.1
    vehicle_shifted_CG.mass_properties.center_of_gravity[0][0] +=delta_cg
    VLM_results_2 = VLM(conditions,np_settings,vehicle_shifted_CG)  
    CM_alpha_cg_prime  = VLM_results_2.CM  
  
    dCM_dalpha_cg = (CM_alpha_cg_prime   - CM_0) / (delta_angle)    
    dCM_dalpha    = (CM_alpha_prime   - CM_0) / (delta_angle)    
     
    m  =  (dCM_dalpha_cg[0] - dCM_dalpha[0]) /delta_cg 
    b  =  dCM_dalpha_cg[0]  - (m * vehicle_shifted_CG.mass_properties.center_of_gravity[0][0])
    NP =  -b / m  
      
    aerodynamics.vehicle.mass_properties.neutral_point = NP 
    
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
    
    vehicle        = deepcopy(aerodynamics.vehicle)
    settings       = aerodynamics.settings
    AoA            = aerodynamics.training.angle_of_attack                  
    Beta           = aerodynamics.training.sideslip_angle
    training       = Data()
    training.Mach  = Mach 
    
    # loop through wings to determine what control surfaces are present 
    for wing in vehicle.wings: 
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
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap: 
                delta_f                     = aerodynamics.training.rudder_deflection
                len_d_f                     = len(delta_f)  
                aerodynamics.flap_flag      = True 
             
    u              = aerodynamics.training.u
    pitch_rate     = aerodynamics.training.pitch_rate
    roll_rate      = aerodynamics.training.roll_rate
    yaw_rate       = aerodynamics.training.yaw_rate  
    len_Mach       = len(Mach)        
    len_AoA        = len(AoA)  
    len_Beta       = len(Beta)
    len_u          = len(u)
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

    VLM_results = VLM(conditions,settings,vehicle)
    Clift_res        = VLM_results.CLift
    Cdrag_res        = VLM_results.CDrag_induced
    CX_res           = VLM_results.CX
    CY_res           = VLM_results.CY
    CZ_res           = VLM_results.CZ
    CL_res           = VLM_results.CL
    CM_res           = VLM_results.CM
    CN_res           = VLM_results.CN
    S_ref            = VLM_results.S_ref
    b_ref            = VLM_results.b_ref
    c_ref            = VLM_results.c_ref
    X_ref            = VLM_results.X_ref
    Y_ref            = VLM_results.Y_ref
    Z_ref            = VLM_results.Z_ref        
    
    Clift_alpha   = np.reshape(Clift_res,(len_Mach,len_AoA)).T 
    Cdrag_alpha   = np.reshape(Cdrag_res,(len_Mach,len_AoA)).T 
    CX_alpha      = np.reshape(CX_res,(len_Mach,len_AoA)).T 
    CY_alpha      = np.reshape(CY_res,(len_Mach,len_AoA)).T 
    CZ_alpha      = np.reshape(CZ_res,(len_Mach,len_AoA)).T 
    CL_alpha      = np.reshape(CL_res,(len_Mach,len_AoA)).T 
    CM_alpha      = np.reshape(CM_res,(len_Mach,len_AoA)).T 
    CN_alpha      = np.reshape(CN_res,(len_Mach,len_AoA)).T  
    
    # Angle of Attack at 0 Degrees .
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
        Clift_wing_alpha[wing.tag] = np.reshape(VLM_results.CLift_wings[wing.tag],(len_Mach,len_AoA)).T    
        Cdrag_wing_alpha[wing.tag] = np.reshape(VLM_results.CDrag_induced_wings[wing.tag],(len_Mach,len_AoA)).T  
  
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
    
    VLM_results = VLM(conditions,settings,vehicle)
    Clift_res = VLM_results.CLift
    Cdrag_res = VLM_results.CDrag_induced
    CX_res    = VLM_results.CX
    CY_res    = VLM_results.CY
    CZ_res    = VLM_results.CZ
    CL_res    = VLM_results.CL
    CM_res    = VLM_results.CM
    CN_res    = VLM_results.CN
    
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
    

    VLM_results = VLM(conditions,settings,vehicle)
    CX_res    = VLM_results.CX
    CZ_res    = VLM_results.CZ
    CM_res    = VLM_results.CM
    CX_u        = np.reshape(VLM_results.CX,(len_Mach,len_u)).T    - CX_alpha_0   
    CZ_u        = np.reshape(VLM_results.CZ,(len_Mach,len_u)).T    - CZ_alpha_0   
    CM_u        = np.reshape(VLM_results.CM,(len_Mach,len_u)).T    - CM_alpha_0  
                    
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
    
    VLM_results = VLM(conditions,settings,vehicle)
    CM_res    = VLM_results.CM  
    CM_q        = np.reshape(CM_res,(len_Mach,len_q)).T    - CM_alpha_0    
    CZ_q        = np.reshape(CZ_res,(len_Mach,len_q)).T    - CZ_alpha_0

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
    VLM_results = VLM(conditions,settings,vehicle)
    CL_res    = VLM_results.CL
    CN_res    = VLM_results.CN
    CY_res    = VLM_results.CY
    CL_p        = (np.reshape(CL_res,(len_Mach,len_p)).T    - CL_alpha_0   )
    CN_p        = (np.reshape(CN_res,(len_Mach,len_p)).T    - CN_alpha_0   )   
    CY_p        = (np.reshape(CY_res,(len_Mach,len_p)).T    - CY_alpha_0   )

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
    
    VLM_results = VLM(conditions,settings,vehicle)
    CL_res    = VLM_results.CL
    CN_res    = VLM_results.CN
    CY_res    = VLM_results.CY
    CL_r        = (np.reshape(CL_res,(len_Mach,len_r)).T    - CL_alpha_0   )
    CN_r        = (np.reshape(CN_res,(len_Mach,len_r)).T    - CN_alpha_0   ) 
    CY_r        = (np.reshape(CY_res,(len_Mach,len_r)).T    - CY_alpha_0   )
        
    # STABILITY COEFFICIENTS  
    training.Clift_alpha       = Clift_alpha 
    training.Clift_wing_alpha  = Clift_wing_alpha
    training.Clift_beta        = Clift_beta

    training.Cdrag_alpha       = Cdrag_alpha   
    training.Cdrag_wing_alpha  = Cdrag_wing_alpha  
    training.Cdrag_beta        = Cdrag_beta 

    training.CX_alpha          = CX_alpha 
    training.CX_beta           = CX_beta
    training.CX_u             = CX_u

    training.CY_alpha          = CY_alpha      
    training.CY_beta           = CY_beta 
    training.CY_r              = CY_r
    training.CY_p              = CY_p

    training.CZ_alpha          = CZ_alpha  
    training.CZ_beta           = CZ_beta
    training.CZ_u              = CZ_u
    training.CZ_q              = CZ_q

    training.CL_alpha          = CL_alpha      
    training.CL_beta           = CL_beta  
    training.CL_p              = CL_p
    training.CL_r              = CL_r

    training.CM_alpha          = CM_alpha 
    training.CM_beta           = CM_beta
    training.CM_u              = CM_u
    training.CM_q              = CM_q

    training.CN_alpha          = CN_alpha      
    training.CN_beta           = CN_beta  
    training.CN_p              = CN_p
    training.CN_r              = CN_r
      
    # STABILITY DERIVATIVES 
    training.dCX_dalpha = (CX_alpha[0,:] - CX_alpha[1,:]) / (AoA[0] - AoA[1])       
    training.dCX_du = (CX_u[0,:] - CX_u[1,:]) / (u[0] - u[1])                                     

    training.dCY_dbeta = 2*((CY_beta[0,:] - CY_beta[1,:]) / (Beta[0] - Beta[1])) # Note correction 
    training.dCY_dr     = (CY_r[0,:] - CY_r[1,:]) / (yaw_rate[0]-yaw_rate[1])   # Added here   

    training.dCZ_dalpha = (CZ_alpha[0,:] - CZ_alpha[1,:]) / (AoA[0] - AoA[1])             
    training.dCZ_du = (CZ_u[0,:] - CZ_u[1,:]) / (u[0] - u[1])    
    training.dCZ_dq     = (CZ_q[0,:] - CZ_q[1,:]) / (pitch_rate[0]-pitch_rate[1])    
 

    
    training.dCL_dbeta = ((CL_beta[0,:] - CL_beta[1,:]) / (Beta[0] - Beta[1]))                
    training.dCL_dp = -2*((CL_p[0,:] - CL_p[1,:]) / (roll_rate[0]-roll_rate[1]))    # Note correction            
    training.dCL_dr = (CL_r[0,:] - CL_r[1,:]) / (yaw_rate[0]-yaw_rate[1])    

    training.dCM_dalpha = (CM_alpha[0,:] - CM_alpha[1,:]) / (AoA[0] - AoA[1])          
    training.dCM_du = (CM_u[0,:] - CM_u[1,:]) / (u[0] - u[1])                                               
    training.dCM_dq = 10*((CM_q[0,:] - CM_q[1,:]) / (pitch_rate[0]-pitch_rate[1]))  # Note correction           
            
    training.dCN_dbeta = (CN_beta[0,:] - CN_beta[1,:]) / (Beta[0] - Beta[1]) 
    training.dCN_dp = -3*((CN_p[0,:] - CN_p[1,:]) / (roll_rate[0]-roll_rate[1]))   # Note correction               
    training.dCN_dr = 3*((CN_r[0,:] - CN_r[1,:]) / (yaw_rate[0]-yaw_rate[1])) # Note correction 

    '''  for control surfaces, subtract inflence WITHOUT control surface deflected from coefficients WITH control surfaces'''
      
    for wing in vehicle.wings: 
        for control_surface in wing.control_surfaces: 
            # --------------------------------------------------------------------------------------------------------------
            # Aileron 
            # --------------------------------------------------------------------------------------------------------------                   
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Aileron:
                CY_d_a         = np.zeros((len_d_a,len_Mach)) 
                CL_d_a         = np.zeros((len_d_a,len_Mach)) 
                CN_d_a         = np.zeros((len_d_a,len_Mach))
                
                for a_i in range(len_d_a):    
                    Machs                                           = np.atleast_2d(np.repeat(Mach,1)).T         
                    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
                    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
                    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
                    conditions.freestream.mach_number               = Machs    
                    vehicle.wings[wing.tag].control_surfaces.aileron.deflection =  delta_a[a_i]
                    VLM_results = VLM(conditions,settings,vehicle)
                    CY_res    = VLM_results.CY
                    CL_res    = VLM_results.CL
                    CN_res    = VLM_results.CN
                    vehicle.wings[wing.tag].control_surfaces.aileron.deflection = 0
                
                    CY_d_a[a_i,:]    =  -(CY_res[:,0]   - CY_alpha_0[0,:]  )
                    CL_d_a[a_i,:]    =  -(CL_res[:,0]   - CL_alpha_0[0,:])
                    CN_d_a[a_i,:]    =  (CN_res[:,0]   - CN_alpha_0[0,:]  )
 
                training.CY_delta_a     = CY_d_a 
                training.CL_delta_a     = CL_d_a 
                training.CN_delta_a     = CN_d_a
                training.dCY_ddelta_a    = (CY_d_a[0,:] - CY_d_a[1,:]) / (delta_a[0] - delta_a[1]) 
                training.dCL_ddelta_a    = ((CL_d_a[0,:] - CL_d_a[1,:]) / (delta_a[0] - delta_a[1]))
                training.dCN_ddelta_a    = (CN_d_a[0,:] - CN_d_a[1,:]) / (delta_a[0] - delta_a[1]) 

            # --------------------------------------------------------------------------------------------------------------
            # Elevator 
            # --------------------------------------------------------------------------------------------------------------    
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator: 
            
                Clift_d_e      = np.zeros((len_d_e,len_Mach)) 
                CM_d_e         = np.zeros((len_d_e,len_Mach)) 
 
                for e_i in range(len_d_e): 
                    Machs                                           = np.atleast_2d(np.repeat(Mach,1)).T         
                    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
                    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
                    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
                    conditions.freestream.mach_number               = Machs     
                    vehicle.wings[wing.tag].control_surfaces.elevator.deflection =  delta_e[e_i]
                
                    VLM_results = VLM(conditions,settings,vehicle)
                    Clift_res = VLM_results.CLift
                    CM_res    = VLM_results.CM
                    
                    vehicle.wings[wing.tag].control_surfaces.elevator.deflection = 0 
                    Clift_d_e[e_i,:] = Clift_res[:,0]  - Clift_alpha_0[0,:]
                    CM_d_e[e_i,:]    = CM_res[:,0]   - CM_alpha_0[0,:]   
                
                training.Clift_delta_e  = Clift_d_e
                training.CM_delta_e     = CM_d_e  
                
                training.dClift_ddelta_e = ((Clift_d_e[0,:] - Clift_d_e[1,:]) / (delta_e[0] - delta_e[1]))
                training.dCM_ddelta_e    = (CM_d_e[0,:] - CM_d_e[1,:]) / (delta_e[0] - delta_e[1])  
    
            # --------------------------------------------------------------------------------------------------------------
            # Rudder 
            # --------------------------------------------------------------------------------------------------------------  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder: 
                CY_d_r         = np.zeros((len_d_r,len_Mach)) 
                CL_d_r         = np.zeros((len_d_r,len_Mach)) 
                CN_d_r         = np.zeros((len_d_r,len_Mach))
              
                for r_i in range(len_d_r): 
                    Machs                                           = np.atleast_2d(np.repeat(Mach,1)).T         
                    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
                    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
                    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
                    conditions.freestream.mach_number               = Machs    
                    vehicle.wings[wing.tag].control_surfaces.rudder.deflection =  delta_r[r_i]
                    VLM_results = VLM(conditions,settings,vehicle)
                    CY_res    = VLM_results.CY
                    CL_res    = VLM_results.CL
                    CN_res    = VLM_results.CN
                    vehicle.wings[wing.tag].control_surfaces.rudder.deflection = 0 
                    CY_d_r[r_i,:]    =   -(CY_res[:,0]   - CY_alpha_0[0,:]  )
                    CL_d_r[r_i,:]    =   -(CL_res[:,0]   - CL_alpha_0[0,:]  )
                    CN_d_r[r_i,:]    =   (CN_res[:,0]   - CN_alpha_0[0,:] )
                
                training.CY_delta_r     = CY_d_r 
                training.CL_delta_r     = CL_d_r                   
                training.CN_delta_r     = CN_d_r    
                training.dCY_ddelta_r    = (CY_d_r[0,:] - CY_d_r[1,:]) / (delta_r[0] - delta_r[1]) 
                training.dCL_ddelta_r    = (CL_d_r[0,:] - CL_d_r[1,:]) / (delta_r[0] - delta_r[1])  
                training.dCN_ddelta_r    = (CN_d_r[0,:] - CN_d_r[1,:]) / (delta_r[0] - delta_r[1])  
                    
            # --------------------------------------------------------------------------------------------------------------
            # Flap
            # --------------------------------------------------------------------------------------------------------------  
            if type(control_surface) == RCAIDE.Library.Components.Wings.Control_Surfaces.Flap:
                CM_d_f         = np.zeros((len_d_f,len_Mach)) 
                
                for f_i in range(len_d_f): 
                    Machs                                           = np.atleast_2d(np.repeat(Mach,1)).T         
                    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
                    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs) *1E-12
                    conditions.aerodynamics.angles.beta             = np.zeros_like(Machs) 
                    conditions.freestream.mach_number               = Machs    
                    vehicle.wings[wing.tag].control_surfaces.flap.deflection = delta_f[f_i]
                    VLM_results = VLM(conditions,settings,vehicle)
                    CM_res    = VLM_results.CM
                    vehicle.wings[wing.tag].control_surfaces.flap.deflection = 0   
                    CM_d_f[f_i,:]    = CM_res[:,0]   - CM_alpha_0[0,:]          
 
                training.CM_delta_f     = CM_d_f                    
                training.dCM_ddelta_f    = (CM_d_f[0,:] - CM_d_f[1,:]) / (delta_f[0] - delta_f[1])  
                
  
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

    vehicle        = deepcopy(aerodynamics.vehicle)
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
    CX_u        =   np.concatenate((training_subsonic.CX_u[:,-1][:,None]    , training_supersonic.CX_u[:,0][:,None] ), axis = 1)         
    CZ_u        =   np.concatenate((training_subsonic.CZ_u[:,-1][:,None]    , training_supersonic.CZ_u[:,0][:,None] ), axis = 1)          
    CM_u        =   np.concatenate((training_subsonic.CM_u[:,-1][:,None]    , training_supersonic.CM_u[:,0][:,None] ), axis = 1)         
             
                    
    # -------------------------------------------------------               
    # Pitch Rate 
    # -------------------------------------------------------        
    CZ_q        =  np.concatenate((training_subsonic.CZ_q[:,-1][:,None]    , training_supersonic.CZ_q[:,0][:,None] ), axis = 1)          
    CM_q        =  np.concatenate((training_subsonic.CM_q[:,-1][:,None]    , training_supersonic.CM_q[:,0][:,None] ), axis = 1)          
  
    # -------------------------------------------------------               
    # Roll  Rate 
    # -------------------------------------------------------            
    #CY_p        =  np.concatenate((training_subsonic.CY_p[:,-1][:,None]    , training_supersonic.CY_p[:,0][:,None] ), axis = 1)         
    CL_p        =  np.concatenate((training_subsonic.CL_p[:,-1][:,None]    , training_supersonic.CL_p[:,0][:,None] ), axis = 1)         
    CN_p        =  np.concatenate((training_subsonic.CN_p[:,-1][:,None]    , training_supersonic.CN_p[:,0][:,None] ), axis = 1)       


    # -------------------------------------------------------               
    # Yaw Rate 
    # -------------------------------------------------------                  
    CY_r        =  np.concatenate((training_subsonic.CY_r[:,-1][:,None]    , training_supersonic.CY_r[:,0][:,None] ), axis = 1)          
    CL_r        =  np.concatenate((training_subsonic.CL_r[:,-1][:,None]    , training_supersonic.CL_r[:,0][:,None] ), axis = 1)          
    CN_r        =  np.concatenate((training_subsonic.CN_r[:,-1][:,None]    , training_supersonic.CN_r[:,0][:,None] ), axis = 1)         
 
    # STABILITY COEFFICIENTS 
    training.Clift_wing_alpha  = Clift_wing_alpha   

    training.Clift_alpha       = Clift_alpha  
    training.Cdrag_alpha       = Cdrag_alpha
    training.Clift_beta        = Clift_beta
    training.Cdrag_beta        = Cdrag_beta
                      
    training.Cdrag_wing_alpha  = Cdrag_wing_alpha           
    training.CX_alpha          = CX_alpha  
    training.CX_u              = CX_u  
    training.CX_beta           = CX_beta

    training.CY_beta           = CY_beta        
    training.CY_r              = CY_r  
    training.CY_alpha          = CY_alpha

    training.CZ_alpha          = CZ_alpha  
    training.CZ_beta           = CZ_beta
    training.CZ_u              = CZ_u              
    training.CZ_q              = CZ_q  

    training.CL_beta           = CL_beta
    training.CL_p              = CL_p           
    training.CL_r              = CL_r   
    training.CL_alpha          = CL_alpha

    training.CM_alpha          = CM_alpha  
    training.CM_beta           = CM_beta
    training.CM_u              = CM_u               
    training.CM_q              = CM_q    

    training.CN_beta           = CN_beta 
    training.CN_alpha          = CN_alpha
    training.CN_p              = CN_p            
    training.CN_r              = CN_r
      
            
    # STABILITY DERIVATIVES 
              
    training.dCX_dalpha    = (CX_alpha[0,:] - CX_alpha[1,:]) / (AoA[0] - AoA[1])            
    training.dCX_du        = (CX_u[0,:] - CX_u[1,:]) / (u[0] - u[1])                                 
         
    training.dCY_dbeta     = (CY_beta[0,:] - CY_beta[1,:]) / (Beta[0] - Beta[1])    
    training.dCY_dr     = (CY_r[0,:] - CY_r[1,:]) / (yaw_rate[0]-yaw_rate[1])                     
    
    training.dCZ_dalpha = (CZ_alpha[0,:] - CZ_alpha[1,:]) / (AoA[0] - AoA[1])             
    training.dCZ_du     = (CZ_u[0,:] - CZ_u[1,:]) / (u[0] - u[1])                                              
    training.dCZ_dq     = (CZ_q[0,:] - CZ_q[1,:]) / (pitch_rate[0]-pitch_rate[1])    

    training.dCL_dbeta  = (CL_beta[0,:] - CL_beta[1,:]) / (Beta[0] - Beta[1])                                                    
    training.dCL_dp     = (CL_p[0,:] - CL_p[1,:]) / (roll_rate[0]-roll_rate[1])                
    training.dCL_dr     = (CL_r[0,:] - CL_r[1,:]) / (yaw_rate[0]-yaw_rate[1])                    
    
    training.dCM_dalpha = (CM_alpha[0,:] - CM_alpha[1,:]) / (AoA[0] - AoA[1])          
    training.dCM_du     = (CM_u[0,:] - CM_u[1,:]) / (u[0] - u[1])                                               
    training.dCM_dq     = (CM_q[0,:] - CM_q[1,:]) / (pitch_rate[0]-pitch_rate[1])             
    
    training.dCN_dbeta  = (CN_beta[0,:] - CN_beta[1,:]) / (Beta[0] - Beta[1])                
    training.dCN_dp = (CN_p[0,:] - CN_p[1,:]) / (roll_rate[0]-roll_rate[1])                 
    training.dCN_dr = (CN_r[0,:] - CN_r[1,:]) / (yaw_rate[0]-yaw_rate[1])


    '''  for control surfaces, subtract inflence WITHOUT control surface deflected from coefficients WITH control surfaces'''
     
    # --------------------------------------------------------------------------------------------------------------
    # Aileron 
    # --------------------------------------------------------------------------------------------------------------   
    if aerodynamics.aileron_flag:   
        #training.Clift_delta_a   =  np.concatenate((training_subsonic.Clift_delta_a[:,-1][:,None] , training_supersonic.Clift_delta_a[:,0][:,None] ), axis = 1) 
        #training.Cdrag_delta_a   =  np.concatenate((training_subsonic.Cdrag_delta_a[:,-1][:,None] , training_supersonic.Cdrag_delta_a[:,0][:,None] ), axis = 1) 
        #training.CX_delta_a      =  np.concatenate((training_subsonic.CX_delta_a[:,-1][:,None]    , training_supersonic.CX_delta_a[:,0][:,None]    ), axis = 1) 
        training.CY_delta_a      =  np.concatenate((training_subsonic.CY_delta_a[:,-1][:,None]    , training_supersonic.CY_delta_a[:,0][:,None]    ), axis = 1) 
        #training.CZ_delta_a      =  np.concatenate((training_subsonic.CZ_delta_a[:,-1][:,None]    , training_supersonic.CZ_delta_a[:,0][:,None]    ), axis = 1) 
        training.CL_delta_a      =  np.concatenate((training_subsonic.CL_delta_a[:,-1][:,None]    , training_supersonic.CL_delta_a[:,0][:,None]    ), axis = 1) 
        #training.CM_delta_a      =  np.concatenate((training_subsonic.CM_delta_a[:,-1][:,None]    , training_supersonic.CM_delta_a[:,0][:,None]    ), axis = 1) 
        training.CN_delta_a      =  np.concatenate((training_subsonic.CN_delta_a[:,-1][:,None]    , training_supersonic.CN_delta_a[:,0][:,None]    ), axis = 1) 
        #training.dClift_ddelta_a =  np.array([training_subsonic.dClift_ddelta_a[-1] , training_subsonic.dClift_ddelta_a[0]])
        #training.dCdrag_ddelta_a =  np.array([training_subsonic.dCdrag_ddelta_a[-1] , training_subsonic.dCdrag_ddelta_a[0]])
        #training.dCX_ddelta_a    =  np.array([training_subsonic.dCX_ddelta_a[-1]    , training_subsonic.dCX_ddelta_a[0]   ])
        training.dCY_ddelta_a    =  np.array([training_subsonic.dCY_ddelta_a[-1]    , training_subsonic.dCY_ddelta_a[0]   ])
        #training.dCZ_ddelta_a    =  np.array([training_subsonic.dCZ_ddelta_a[-1]    , training_subsonic.dCZ_ddelta_a[0]   ])
        training.dCL_ddelta_a    =  np.array([training_subsonic.dCL_ddelta_a[-1]    , training_subsonic.dCL_ddelta_a[0]   ])
        #training.dCM_ddelta_a    =  np.array([training_subsonic.dCM_ddelta_a[-1]    , training_subsonic.dCM_ddelta_a[0]   ])
        training.dCN_ddelta_a    =  np.array([training_subsonic.dCN_ddelta_a[-1]    , training_subsonic.dCN_ddelta_a[0]   ])
    
    # --------------------------------------------------------------------------------------------------------------
    # Elevator 
    # -------------------------------------------------------------------------------------------------------------- 
    if aerodynamics.elevator_flag:  
                    
        #training.Clift_delta_e   =  np.concatenate((training_subsonic.Clift_delta_e[:,-1][:,None] , training_supersonic.Clift_delta_e[:,0][:,None] ), axis = 1)   
        #training.Cdrag_delta_e   =  np.concatenate((training_subsonic.Cdrag_delta_e[:,-1][:,None] , training_supersonic.Cdrag_delta_e[:,0][:,None] ), axis = 1)   
        #training.CX_delta_e      =  np.concatenate((training_subsonic.CX_delta_e[:,-1][:,None]    , training_supersonic.CX_delta_e[:,0][:,None]    ), axis = 1)   
        #training.CY_delta_e      =  np.concatenate((training_subsonic.CY_delta_e[:,-1][:,None]    , training_supersonic.CY_delta_e[:,0][:,None]    ), axis = 1)   
        #training.CZ_delta_e      =  np.concatenate((training_subsonic.CZ_delta_e[:,-1][:,None]    , training_supersonic.CZ_delta_e[:,0][:,None]    ), axis = 1)   
        #training.CL_delta_e      =  np.concatenate((training_subsonic.CL_delta_e[:,-1][:,None]    , training_supersonic.CL_delta_e[:,0][:,None]    ), axis = 1)   
        training.CM_delta_e      =  np.concatenate((training_subsonic.CM_delta_e[:,-1][:,None]    , training_supersonic.CM_delta_e[:,0][:,None]    ), axis = 1)   
        #training.CN_delta_e      =  np.concatenate((training_subsonic.CN_delta_e[:,-1][:,None]    , training_supersonic.CN_delta_e[:,0][:,None]    ), axis = 1)   
        #training.dClift_ddelta_e =  np.array([training_subsonic.dClift_ddelta_e[-1] , training_subsonic.dClift_ddelta_e[0]])
        #training.dCdrag_ddelta_e =  np.array([training_subsonic.dCdrag_ddelta_e[-1] , training_subsonic.dCdrag_ddelta_e[0]])
        #training.dCX_ddelta_e    =  np.array([training_subsonic.dCX_ddelta_e[-1]    , training_subsonic.dCX_ddelta_e[0]   ])
        #training.dCY_ddelta_e    =  np.array([training_subsonic.dCY_ddelta_e[-1]    , training_subsonic.dCY_ddelta_e[0]   ])
        #training.dCZ_ddelta_e    =  np.array([training_subsonic.dCZ_ddelta_e[-1]    , training_subsonic.dCZ_ddelta_e[0]   ])
        #training.dCL_ddelta_e    =  np.array([training_subsonic.dCL_ddelta_e[-1]    , training_subsonic.dCL_ddelta_e[0]   ])
        training.dCM_ddelta_e    =  np.array([training_subsonic.dCM_ddelta_e[-1]    , training_subsonic.dCM_ddelta_e[0]   ])
        #training.dCN_ddelta_e    =  np.array([training_subsonic.dCN_ddelta_e[-1]    , training_subsonic.dCN_ddelta_e[0]   ])
        
    # --------------------------------------------------------------------------------------------------------------
    # Rudder 
    # -------------------------------------------------------------------------------------------------------------- 
    if aerodynamics.rudder_flag: 
        #training.Clift_delta_r   =  np.concatenate((training_subsonic.Clift_delta_r[:,-1][:,None] , training_supersonic.Clift_delta_r[:,0][:,None] ), axis = 1)  
        #training.Cdrag_delta_r   =  np.concatenate((training_subsonic.Cdrag_delta_r[:,-1][:,None] , training_supersonic.Cdrag_delta_r[:,0][:,None] ), axis = 1)  
        #training.CX_delta_r      =  np.concatenate((training_subsonic.CX_delta_r[:,-1][:,None]    , training_supersonic.CX_delta_r[:,0][:,None]    ), axis = 1)  
        training.CY_delta_r      =  np.concatenate((training_subsonic.CY_delta_r[:,-1][:,None]    , training_supersonic.CY_delta_r[:,0][:,None]    ), axis = 1)  
        #training.CZ_delta_r      =  np.concatenate((training_subsonic.CZ_delta_r[:,-1][:,None]    , training_supersonic.CZ_delta_r[:,0][:,None]    ), axis = 1)  
        training.CL_delta_r      =  np.concatenate((training_subsonic.CL_delta_r[:,-1][:,None]    , training_supersonic.CL_delta_r[:,0][:,None]    ), axis = 1)  
        #training.CM_delta_r      =  np.concatenate((training_subsonic.CM_delta_r[:,-1][:,None]    , training_supersonic.CM_delta_r[:,0][:,None]    ), axis = 1)  
        training.CN_delta_r      =  np.concatenate((training_subsonic.CN_delta_r[:,-1][:,None]    , training_supersonic.CN_delta_r[:,0][:,None]    ), axis = 1)  
        #training.dClift_ddelta_r =  np.array([training_subsonic.dClift_ddelta_r[-1] , training_subsonic.dClift_ddelta_r[0]])
        #training.dCdrag_ddelta_r =  np.array([training_subsonic.dCdrag_ddelta_r[-1] , training_subsonic.dCdrag_ddelta_r[0]])
        #training.dCX_ddelta_r    =  np.array([training_subsonic.dCX_ddelta_r[-1]    , training_subsonic.dCX_ddelta_r[0]   ])
        training.dCY_ddelta_r    =  np.array([training_subsonic.dCY_ddelta_r[-1]    , training_subsonic.dCY_ddelta_r[0]   ])
        #training.dCZ_ddelta_r    =  np.array([training_subsonic.dCZ_ddelta_r[-1]    , training_subsonic.dCZ_ddelta_r[0]   ])
        training.dCL_ddelta_r    =  np.array([training_subsonic.dCL_ddelta_r[-1]    , training_subsonic.dCL_ddelta_r[0]   ])
        #training.dCM_ddelta_r    =  np.array([training_subsonic.dCM_ddelta_r[-1]    , training_subsonic.dCM_ddelta_r[0]   ])
        training.dCN_ddelta_r    =  np.array([training_subsonic.dCN_ddelta_r[-1]    , training_subsonic.dCN_ddelta_r[0]   ])
                        
    # --------------------------------------------------------------------------------------------------------------
    # Flap
    # -------------------------------------------------------------------------------------------------------------- 
    if aerodynamics.flap_flag: 
        #training.Clift_delta_f   = np.concatenate((training_subsonic.Clift_delta_f[:,-1][:,None] , training_supersonic.Clift_delta_f[:,0][:,None] ), axis = 1)   
        #training.Cdrag_delta_f   = np.concatenate((training_subsonic.Cdrag_delta_f[:,-1][:,None] , training_supersonic.Cdrag_delta_f[:,0][:,None] ), axis = 1)   
        #training.CX_delta_f      = np.concatenate((training_subsonic.CX_delta_f[:,-1][:,None]    , training_supersonic.CX_delta_f[:,0][:,None]    ), axis = 1)   
        #training.CY_delta_f      = np.concatenate((training_subsonic.CY_delta_f[:,-1][:,None]    , training_supersonic.CY_delta_f[:,0][:,None]    ), axis = 1)   
        #training.CZ_delta_f      = np.concatenate((training_subsonic.CZ_delta_f[:,-1][:,None]    , training_supersonic.CZ_delta_f[:,0][:,None]    ), axis = 1)   
        #training.CL_delta_f      = np.concatenate((training_subsonic.CL_delta_f[:,-1][:,None]    , training_supersonic.CL_delta_f[:,0][:,None]    ), axis = 1)   
        training.CM_delta_f      = np.concatenate((training_subsonic.CM_delta_f[:,-1][:,None]    , training_supersonic.CM_delta_f[:,0][:,None]    ), axis = 1)   
        #training.CN_delta_f      = np.concatenate((training_subsonic.CN_delta_f[:,-1][:,None]    , training_supersonic.CN_delta_f[:,0][:,None]    ), axis = 1)   
        #training.dClift_ddelta_f = np.array([training_subsonic.dClift_ddelta_f[-1] , training_subsonic.dClift_ddelta_f[0]])
        #training.dCdrag_ddelta_f = np.array([training_subsonic.dCdrag_ddelta_f[-1] , training_subsonic.dCdrag_ddelta_f[0]])
        #training.dCX_ddelta_f    = np.array([training_subsonic.dCX_ddelta_f[-1]    , training_subsonic.dCX_ddelta_f[0]   ])
        #training.dCY_ddelta_f    = np.array([training_subsonic.dCY_ddelta_f[-1]    , training_subsonic.dCY_ddelta_f[0]   ])
        #training.dCZ_ddelta_f    = np.array([training_subsonic.dCZ_ddelta_f[-1]    , training_subsonic.dCZ_ddelta_f[0]   ])
        #training.dCL_ddelta_f    = np.array([training_subsonic.dCL_ddelta_f[-1]    , training_subsonic.dCL_ddelta_f[0]   ])
        training.dCM_ddelta_f    = np.array([training_subsonic.dCM_ddelta_f[-1]    , training_subsonic.dCM_ddelta_f[0]   ])
        #training.dCN_ddelta_f    = np.array([training_subsonic.dCN_ddelta_f[-1]    , training_subsonic.dCN_ddelta_f[0]   ])
                    
    # --------------------------------------------------------------------------------------------------------------
    # Slat
    # -------------------------------------------------------------------------------------------------------------- 
    # if aerodynamics.slat_flag: 
    #     training.Clift_delta_s   =  np.concatenate((training_subsonic.Clift_delta_s[:,-1][:,None] , training_supersonic.Clift_delta_s[:,0][:,None] ), axis = 1)   
    #     training.Cdrag_delta_s   =  np.concatenate((training_subsonic.Cdrag_delta_s[:,-1][:,None] , training_supersonic.Cdrag_delta_s[:,0][:,None] ), axis = 1)   
    #     training.CX_delta_s      =  np.concatenate((training_subsonic.CX_delta_s[:,-1][:,None]    , training_supersonic.CX_delta_s[:,0][:,None]    ), axis = 1)   
    #     training.CY_delta_s      =  np.concatenate((training_subsonic.CY_delta_s[:,-1][:,None]    , training_supersonic.CY_delta_s[:,0][:,None]    ), axis = 1)   
    #     training.CZ_delta_s      =  np.concatenate((training_subsonic.CZ_delta_s[:,-1][:,None]    , training_supersonic.CZ_delta_s[:,0][:,None]    ), axis = 1)   
    #     training.CL_delta_s      =  np.concatenate((training_subsonic.CL_delta_s[:,-1][:,None]    , training_supersonic.CL_delta_s[:,0][:,None]    ), axis = 1)   
    #     training.CM_delta_s      =  np.concatenate((training_subsonic.CM_delta_s[:,-1][:,None]    , training_supersonic.CM_delta_s[:,0][:,None]    ), axis = 1)   
    #     training.CN_delta_s      =  np.concatenate((training_subsonic.CN_delta_s[:,-1][:,None]    , training_supersonic.CN_delta_s[:,0][:,None]    ), axis = 1)   
    #     training.dClift_ddelta_s =  np.array([training_subsonic.dClift_ddelta_s[-1] , training_subsonic.dClift_ddelta_s[0]])
    #     training.dCdrag_ddelta_s =  np.array([training_subsonic.dCdrag_ddelta_s[-1] , training_subsonic.dCdrag_ddelta_s[0]])
    #     training.dCX_ddelta_s    =  np.array([training_subsonic.dCX_ddelta_s[-1]    , training_subsonic.dCX_ddelta_s[0]   ])
    #     training.dCY_ddelta_s    =  np.array([training_subsonic.dCY_ddelta_s[-1]    , training_subsonic.dCY_ddelta_s[0]   ])
    #     training.dCZ_ddelta_s    =  np.array([training_subsonic.dCZ_ddelta_s[-1]    , training_subsonic.dCZ_ddelta_s[0]   ])
    #     training.dCL_ddelta_s    =  np.array([training_subsonic.dCL_ddelta_s[-1]    , training_subsonic.dCL_ddelta_s[0]   ])
    #     training.dCM_ddelta_s    =  np.array([training_subsonic.dCM_ddelta_s[-1]    , training_subsonic.dCM_ddelta_s[0]   ])
    #     training.dCN_ddelta_s    =  np.array([training_subsonic.dCN_ddelta_s[-1]    , training_subsonic.dCN_ddelta_s[0]   ])
    training.NP            = 0  
    
    return training 