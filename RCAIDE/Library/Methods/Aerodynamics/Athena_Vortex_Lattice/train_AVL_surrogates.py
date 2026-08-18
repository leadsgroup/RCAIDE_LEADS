# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/train_AVL_surrogates.py
#  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE 
from RCAIDE.Framework.Mission.Common                                             import Results  
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.run_AVL_analysis  import run_AVL_analysis  
from RCAIDE.Library.Components.Wings.Control_Surfaces                            import Aileron , Elevator , Slat , Flap , Rudder 
 
# Package imports 
import os
import numpy as np
from shutil import rmtree    

# ----------------------------------------------------------------------------------------------------------------------
#  train_AVL_surrogates
# ---------------------------------------------------------------------------------------------------------------------- 
def train_AVL_surrogates(aerodynamics,vehicle):
    """Call methods to run VLM for sample point evaluation. 
    
    Assumptions:
        None
        
    Source:
        None

    Args:
        aerodynamics       : VLM analysis          [unitless] 
        
    Returns: 
        None    
    """ 
 
    run_folder             = os.path.abspath(aerodynamics.settings.filenames.run_folder) 
    training               = aerodynamics.training  
    AoA                    = training.angle_of_attack
    Mach                   = training.Mach
    side_slip_angle        = aerodynamics.settings.side_slip_angle
    roll_rate_coefficient  = aerodynamics.settings.roll_rate_coefficient
    pitch_rate_coefficient = aerodynamics.settings.pitch_rate_coefficient
    lift_coefficient       = aerodynamics.settings.lift_coefficient 
    n_sw                   = aerodynamics.settings.number_of_spanwise_vortices 
    atmosphere             = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data              = atmosphere.compute_values(altitude = 0.0)         
    
    
    n_wings = 0   
    cs_functions = []  
         
    for wing in vehicle.wings:         
        n_wings += 1 
        if wing.xz_plane_symmetric: 
            n_wings += 1
            
        if wing.control_surfaces:   
            for ctrl_surf in wing.control_surfaces:  
                if (type(ctrl_surf) ==  Slat):
                    ctrl_surf_function  = 'slat'
                    aerodynamics.slat_flag   = True 
                elif (type(ctrl_surf) ==  Flap):
                    ctrl_surf_function  = 'flap'  
                    aerodynamics.flap_flag   = True 
                elif (type(ctrl_surf) ==  Aileron):
                    ctrl_surf_function  = 'aileron'     
                    aerodynamics.aileron_flag   = True                      
                elif (type(ctrl_surf) ==  Elevator):
                    ctrl_surf_function  = 'elevator' 
                    aerodynamics.elevator_flag   = True 
                elif (type(ctrl_surf) ==  Rudder):
                    ctrl_surf_function = 'rudder'   
                    aerodynamics.rudder_flag      = True                    
                cs_functions.append(ctrl_surf_function)  
            
    aerodynamics.settings.control_surface_tags =  cs_functions
            
            
    len_AoA  = len(AoA)
    len_Mach = len(Mach)
    
    training.Clift_alpha          = np.zeros((len_AoA,len_Mach))
    training.Cdrag_induced_alpha  = np.zeros((len_AoA,len_Mach))
    training.span_efficincy       = np.zeros((len_AoA,len_Mach))
    training.oswald_efficiency    = np.zeros((len_AoA,len_Mach))
    training.Clift_spanwise       =  np.zeros((len_AoA,len_Mach,n_sw*n_wings))
    training.dClift_dalpha        = np.zeros((len_AoA,len_Mach))
    training.dCY_dalpha           = np.zeros((len_AoA,len_Mach))
    training.dCL_dalpha           = np.zeros((len_AoA,len_Mach))
    training.dCM_dalpha           = np.zeros((len_AoA,len_Mach))
    training.dCN_dalpha           = np.zeros((len_AoA,len_Mach))
    training.dClift_dbeta         = np.zeros((len_AoA,len_Mach))
    training.dCY_dbeta            = np.zeros((len_AoA,len_Mach))
    training.dCL_dbeta            = np.zeros((len_AoA,len_Mach))
    training.dCM_dbeta            = np.zeros((len_AoA,len_Mach))
    training.dCN_dbeta            = np.zeros((len_AoA,len_Mach))       
    training.dClift_dp            = np.zeros((len_AoA,len_Mach))       
    training.dClift_dq            = np.zeros((len_AoA,len_Mach))       
    training.dClift_dr            = np.zeros((len_AoA,len_Mach))       
    training.dCY_dp               = np.zeros((len_AoA,len_Mach))     
    training.dCY_dq               = np.zeros((len_AoA,len_Mach))     
    training.dCY_dr               = np.zeros((len_AoA,len_Mach))     
    training.dCL_dp               = np.zeros((len_AoA,len_Mach))     
    training.dCL_dq               = np.zeros((len_AoA,len_Mach))     
    training.dCL_dr               = np.zeros((len_AoA,len_Mach))     
    training.dCM_dp               = np.zeros((len_AoA,len_Mach))     
    training.dCM_dq               = np.zeros((len_AoA,len_Mach))     
    training.dCM_dr               = np.zeros((len_AoA,len_Mach))     
    training.dCN_dp               = np.zeros((len_AoA,len_Mach))     
    training.dCN_dq               = np.zeros((len_AoA,len_Mach))     
    training.dCN_dr               = np.zeros((len_AoA,len_Mach))     
    training.dCX_du               = np.zeros((len_AoA,len_Mach))     
    training.dCX_dv               = np.zeros((len_AoA,len_Mach))     
    training.dCX_dw               = np.zeros((len_AoA,len_Mach))     
    training.dCY_du               = np.zeros((len_AoA,len_Mach))     
    training.dCY_dv               = np.zeros((len_AoA,len_Mach))     
    training.dCY_dw               = np.zeros((len_AoA,len_Mach))     
    training.dCZ_du               = np.zeros((len_AoA,len_Mach))     
    training.dCZ_dv               = np.zeros((len_AoA,len_Mach))     
    training.dCZ_dw               = np.zeros((len_AoA,len_Mach))     
    training.dCL_du               = np.zeros((len_AoA,len_Mach))     
    training.dCL_dv               = np.zeros((len_AoA,len_Mach))     
    training.dCL_dw               = np.zeros((len_AoA,len_Mach))     
    training.dCM_du               = np.zeros((len_AoA,len_Mach))     
    training.dCM_dv               = np.zeros((len_AoA,len_Mach))     
    training.dCM_dw               = np.zeros((len_AoA,len_Mach))     
    training.dCN_du               = np.zeros((len_AoA,len_Mach))     
    training.dCN_dv               = np.zeros((len_AoA,len_Mach))     
    training.dCN_dw               = np.zeros((len_AoA,len_Mach))
    training.dCX_dp               = np.zeros((len_AoA,len_Mach))
    training.dCX_dq               = np.zeros((len_AoA,len_Mach))
    training.dCX_dr               = np.zeros((len_AoA,len_Mach))
    training.dCY_dp               = np.zeros((len_AoA,len_Mach))
    training.dCY_dq               = np.zeros((len_AoA,len_Mach))
    training.dCY_dr               = np.zeros((len_AoA,len_Mach))
    training.dCZ_dp               = np.zeros((len_AoA,len_Mach))
    training.dCZ_dq               = np.zeros((len_AoA,len_Mach))
    training.dCZ_dr               = np.zeros((len_AoA,len_Mach))
    training.neutral_point        = np.zeros((len_AoA,len_Mach))
    training.spiral_criteria      = np.zeros((len_AoA,len_Mach)) 
     

    '''  for control surfaces, subtract inflence WITHOUT control surface deflected from coefficients WITH control surfaces'''

    for cs in  aerodynamics.settings.control_surface_tags:  
        if cs == 'flap':
            letter = 'f' 
        if cs == 'slat':
            letter = 's'
        if cs == 'rudder':
            letter = 'r'
        if cs == 'elevator':
            letter = 'e' 
        if cs == 'aileron':
            letter = 'a'    
            
        lift_derivative = 'dClift_ddelta_' + letter 
        drag_derivative = 'dCdrag_induced_ddelta_' + letter 
        L_derivative    = 'dCL_ddelta_' + letter 
        M_derivative    = 'dCM_ddelta_' + letter 
        N_derivative    = 'dCN_ddelta_' + letter 
        Y_derivative    = 'dCY_ddelta_' + letter   
            
        training[lift_derivative]  = np.zeros((len_AoA,len_Mach))
        training[drag_derivative]  = np.zeros((len_AoA,len_Mach))
        training[L_derivative]     = np.zeros((len_AoA,len_Mach))
        training[M_derivative]     = np.zeros((len_AoA,len_Mach))
        training[N_derivative]     = np.zeros((len_AoA,len_Mach))
        training[Y_derivative]     = np.zeros((len_AoA,len_Mach)) 
        

    # remove old files in run directory  
    if os.path.exists(aerodynamics.settings.filenames.run_folder):
        if aerodynamics.settings.new_regression_results:
            rmtree(run_folder)

    for i,_ in enumerate(Mach):
        # Set training conditions
        run_conditions = Results()
        run_conditions.expand_rows(len_AoA)
        run_conditions.aerodynamics.angles.alpha           = np.array([AoA]).T  
        run_conditions.freestream.density                  = np.ones_like(run_conditions.aerodynamics.angles.alpha)*atmo_data.density 
        run_conditions.freestream.gravity                  = np.ones_like(run_conditions.aerodynamics.angles.alpha)*9.81          
        run_conditions.freestream.speed_of_sound           = np.ones_like(run_conditions.aerodynamics.angles.alpha)*atmo_data.speed_of_sound[0,0]  
        run_conditions.freestream.velocity                 = np.ones_like(run_conditions.aerodynamics.angles.alpha)*Mach[i] * run_conditions.freestream.speed_of_sound 
        run_conditions.freestream.mach_number              = np.ones_like(run_conditions.aerodynamics.angles.alpha)*Mach[i]
        run_conditions.aerodynamics.angles.beta            = np.ones_like(run_conditions.aerodynamics.angles.alpha)*side_slip_angle 
        run_conditions.static_stability.coefficients.roll  = np.ones_like(run_conditions.aerodynamics.angles.alpha)*roll_rate_coefficient   
        if lift_coefficient == None: 
            run_conditions.aerodynamics.coefficients.lift.inviscid.total= lift_coefficient
        else:
            run_conditions.aerodynamics.coefficients.lift.inviscid.total= np.array([lift_coefficient]).T  
        run_conditions.static_stability.coefficients.pitch = np.ones_like(run_conditions.aerodynamics.angles.alpha)*pitch_rate_coefficient 

        # Run Analysis at AoA[i] and Mach[i]
        run_AVL_analysis(aerodynamics,run_conditions, vehicle) 
    
        # Pack the outputs
        training.Clift_alpha[:,i]          = run_conditions.aerodynamics.coefficients.lift.inviscid.total[:,0]             
        training.Cdrag_induced_alpha[:,i]  = run_conditions.aerodynamics.coefficients.drag.induced.inviscid[:,0]          
        training.span_efficincy[:,i]       = run_conditions.aerodynamics.coefficients.drag.induced.efficiency_factor[:,0]      
        training.oswald_efficiency[:,i]    = run_conditions.aerodynamics.oswald_efficiency[:,0]                             
        training.Clift_spanwise[:,i]       = run_conditions.aerodynamics.coefficients.lift.spanwise   
        training.dClift_dalpha[:,i]        = run_conditions.static_stability.derivatives.Clift_alpha[:,0]     
        training.dCY_dalpha[:,i]           = run_conditions.static_stability.derivatives.CY_alpha[:,0]       
        training.dCL_dalpha[:,i]           = run_conditions.static_stability.derivatives.CL_alpha[:,0]       
        training.dCM_dalpha[:,i]           = run_conditions.static_stability.derivatives.CM_alpha[:,0]       
        training.dCN_dalpha[:,i]           = run_conditions.static_stability.derivatives.CN_alpha[:,0]       
        training.dClift_dbeta[:,i]         = run_conditions.static_stability.derivatives.Clift_beta[:,0]    
        training.dCY_dbeta[:,i]            = run_conditions.static_stability.derivatives.CY_beta[:,0]        
        training.dCL_dbeta[:,i]            = run_conditions.static_stability.derivatives.CL_beta[:,0]        
        training.dCM_dbeta[:,i]            = run_conditions.static_stability.derivatives.CM_beta[:,0]        
        training.dCN_dbeta[:,i]            = run_conditions.static_stability.derivatives.CN_beta[:,0]        
        training.dClift_dp[:,i]            = run_conditions.static_stability.derivatives.Clift_p[:,0]        
        training.dClift_dq[:,i]            = run_conditions.static_stability.derivatives.Clift_q[:,0]        
        training.dClift_dr[:,i]            = run_conditions.static_stability.derivatives.Clift_r[:,0]        
        training.dCY_dp[:,i]               = run_conditions.static_stability.derivatives.CY_p[:,0]         
        training.dCY_dq[:,i]               = run_conditions.static_stability.derivatives.CY_q[:,0]         
        training.dCY_dr[:,i]               = run_conditions.static_stability.derivatives.CY_r[:,0]         
        training.dCL_dp[:,i]               = run_conditions.static_stability.derivatives.CL_p[:,0]         
        training.dCL_dq[:,i]               = run_conditions.static_stability.derivatives.CL_q[:,0]         
        training.dCL_dr[:,i]               = run_conditions.static_stability.derivatives.CL_r[:,0]         
        training.dCM_dp[:,i]               = run_conditions.static_stability.derivatives.CM_p[:,0]         
        training.dCM_dq[:,i]               = run_conditions.static_stability.derivatives.CM_q[:,0]         
        training.dCM_dr[:,i]               = run_conditions.static_stability.derivatives.CM_r[:,0]         
        training.dCN_dp[:,i]               = run_conditions.static_stability.derivatives.CN_p[:,0]         
        training.dCN_dq[:,i]               = run_conditions.static_stability.derivatives.CN_q[:,0]         
        training.dCN_dr[:,i]               = run_conditions.static_stability.derivatives.CN_r[:,0]         
        training.dCX_du[:,i]               = run_conditions.static_stability.derivatives.CX_u[:,0]         
        training.dCX_dv[:,i]               = run_conditions.static_stability.derivatives.CX_v[:,0]         
        training.dCX_dw[:,i]               = run_conditions.static_stability.derivatives.CX_w[:,0]         
        training.dCY_du[:,i]               = run_conditions.static_stability.derivatives.CY_u[:,0]         
        training.dCY_dv[:,i]               = run_conditions.static_stability.derivatives.CY_v[:,0]         
        training.dCY_dw[:,i]               = run_conditions.static_stability.derivatives.CY_w[:,0]         
        training.dCZ_du[:,i]               = run_conditions.static_stability.derivatives.CZ_u[:,0]         
        training.dCZ_dv[:,i]               = run_conditions.static_stability.derivatives.CZ_v[:,0]         
        training.dCZ_dw[:,i]               = run_conditions.static_stability.derivatives.CZ_w[:,0]         
        training.dCL_du[:,i]               = run_conditions.static_stability.derivatives.CL_u[:,0]         
        training.dCL_dv[:,i]               = run_conditions.static_stability.derivatives.CL_v[:,0]         
        training.dCL_dw[:,i]               = run_conditions.static_stability.derivatives.CL_w[:,0]         
        training.dCM_du[:,i]               = run_conditions.static_stability.derivatives.CM_u[:,0]         
        training.dCM_dv[:,i]               = run_conditions.static_stability.derivatives.CM_v[:,0]         
        training.dCM_dw[:,i]               = run_conditions.static_stability.derivatives.CM_w[:,0]         
        training.dCN_du[:,i]               = run_conditions.static_stability.derivatives.CN_u[:,0]         
        training.dCN_dv[:,i]               = run_conditions.static_stability.derivatives.CN_v[:,0]         
        training.dCN_dw[:,i]               = run_conditions.static_stability.derivatives.CN_w[:,0]   
        training.dCX_dp[:,i]               = run_conditions.static_stability.derivatives.CX_p[:,0] 
        training.dCX_dq[:,i]               = run_conditions.static_stability.derivatives.CX_q[:,0] 
        training.dCX_dr[:,i]               = run_conditions.static_stability.derivatives.CX_r[:,0] 
        training.dCY_dp[:,i]               = run_conditions.static_stability.derivatives.CY_p[:,0] 
        training.dCY_dq[:,i]               = run_conditions.static_stability.derivatives.CY_q[:,0] 
        training.dCY_dr[:,i]               = run_conditions.static_stability.derivatives.CY_r[:,0] 
        training.dCZ_dp[:,i]               = run_conditions.static_stability.derivatives.CZ_p[:,0] 
        training.dCZ_dq[:,i]               = run_conditions.static_stability.derivatives.CZ_q[:,0] 
        training.dCZ_dr[:,i]               = run_conditions.static_stability.derivatives.CZ_r[:,0]  
        training.neutral_point[:,i]        = run_conditions.static_stability.neutral_point[:,0]  
        training.spiral_criteria[:,i]      = run_conditions.static_stability.spiral_criteria[:,0]   
    
        '''  for control surfaces, subtract inflence WITHOUT control surface deflected from coefficients WITH control surfaces''' 

        for cs in  aerodynamics.settings.control_surface_tags:  
            if cs == 'flap':
                letter = 'f' 
            if cs == 'slat':
                letter = 's'
            if cs == 'rudder':
                letter = 'r'
            if cs == 'elevator':
                letter = 'e' 
            if cs == 'aileron':
                letter = 'a'    
                
            lift_derivative = 'Clift_delta_' + letter 
            drag_derivative = 'Cdrag_induced_delta_' + letter 
            L_derivative    = 'CL_delta_' + letter 
            M_derivative    = 'CM_delta_' + letter 
            N_derivative    = 'CN_delta_' + letter 
            Y_derivative    = 'CY_delta_' + letter

            training_lift_derivative = 'dClift_ddelta_' + letter 
            training_drag_derivative = 'dCdrag_induced_ddelta_' + letter 
            training_L_derivative    = 'dCL_ddelta_' + letter 
            training_M_derivative    = 'dCM_ddelta_' + letter 
            training_N_derivative    = 'dCN_ddelta_' + letter 
            training_Y_derivative    = 'dCY_ddelta_' + letter
            
                
            training[training_lift_derivative][:,i] = run_conditions.static_stability.derivatives[lift_derivative][:,0]
            training[training_drag_derivative][:,i] = run_conditions.static_stability.derivatives[drag_derivative][:,0]
            training[training_L_derivative][:,i]    = run_conditions.static_stability.derivatives[L_derivative][:,0]
            training[training_M_derivative][:,i]    = run_conditions.static_stability.derivatives[M_derivative][:,0]
            training[training_N_derivative][:,i]    = run_conditions.static_stability.derivatives[N_derivative][:,0]
            training[training_Y_derivative][:,i]    = run_conditions.static_stability.derivatives[Y_derivative][:,0] 
 
    return training