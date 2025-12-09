# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/train_AVL_surrogates.py
#  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE 
from RCAIDE.Framework.Mission.Common                                             import Results  
from RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.run_AVL_analysis  import run_AVL_analysis  
 
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
    atmosphere             = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data              = atmosphere.compute_values(altitude = 0.0)         
    
    len_AoA  = len(AoA)
    len_Mach = len(Mach)
    CM       = np.zeros((len_AoA,len_Mach))
    CL       = np.zeros_like(CM)
    CD       = np.zeros_like(CM)
    e        = np.zeros_like(CM)
    Cm_alpha = np.zeros_like(CM)
    Cn_beta  = np.zeros_like(CM)
    NP       = np.zeros_like(CM)  

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
 
        CL[:,i]       = run_conditions.aerodynamics.coefficients.lift.inviscid.total[:,0]
        CD[:,i]       = run_conditions.aerodynamics.coefficients.drag.induced.total[:,0]      
        e [:,i]       = run_conditions.aerodynamics.coefficients.drag.induced.efficiency_factor[:,0]   
        CM[:,i]       = run_conditions.static_stability.coefficients.pitch[:,0]
        Cm_alpha[:,i] = run_conditions.static_stability.derivatives.CM_alpha[:,0]
        Cn_beta[:,i]  = run_conditions.static_stability.derivatives.CN_beta[:,0]
        NP[:,i]       = run_conditions.static_stability.neutral_point[:,0]     

    if aerodynamics.training_file:
        # load data 
        data_array   = np.loadtxt(aerodynamics.training_file) 
        
        # convert from 1D to 2D        
        CL_1D         = np.atleast_2d(data_array[:,0]) 
        CD_1D         = np.atleast_2d(data_array[:,1])            
        e_1D          = np.atleast_2d(data_array[:,2])
        CM_1D         = np.atleast_2d(data_array[:,3]) 
        Cm_alpha_1D   = np.atleast_2d(data_array[:,4])            
        Cn_beta_1D    = np.atleast_2d(data_array[:,5])
        NP_1D         = np.atleast_2d(data_array[:,6])

        # convert from 1D to 2D
        CL        = np.reshape(CL_1D, (len_AoA,-1))
        CD        = np.reshape(CD_1D, (len_AoA,-1))
        e         = np.reshape(e_1D , (len_AoA,-1)) 
        CM        = np.reshape(CM_1D, (len_AoA,-1))
        Cm_alpha  = np.reshape(Cm_alpha_1D, (len_AoA,-1))
        Cn_beta   = np.reshape(Cn_beta_1D , (len_AoA,-1))
        NP        = np.reshape(NP_1D , (len_AoA,-1))

    # Save the data for regression 
    if aerodynamics.settings.new_regression_results:
        # convert from 2D to 1D
        CL_1D       = CL.reshape([len_AoA*len_Mach,1]) 
        CD_1D       = CD.reshape([len_AoA*len_Mach,1])  
        e_1D        = e.reshape([len_AoA*len_Mach,1]) 
        CM_1D       = CM.reshape([len_AoA*len_Mach,1]) 
        Cm_alpha_1D = Cm_alpha.reshape([len_AoA*len_Mach,1])  
        Cn_beta_1D  = Cn_beta.reshape([len_AoA*len_Mach,1])         
        NP_1D       = Cn_beta.reshape([len_AoA*len_Mach,1]) 
        np.savetxt(vehicle.tag+'_stability_data.txt',np.hstack([CL_1D,CD_1D,e_1D,CM_1D,Cm_alpha_1D, Cn_beta_1D,NP_1D ]),fmt='%10.8f',header='   CM       Cm_alpha       Cn_beta       NP ')

    # Store training data
    # Save the data for regression
    training_data = np.zeros((7,len_AoA,len_Mach))
    training_data[0,:,:] = CL 
    training_data[1,:,:] = CD 
    training_data[2,:,:] = e  
    training_data[3,:,:] = CM       
    training_data[4,:,:] = Cm_alpha 
    training_data[5,:,:] = Cn_beta  
    training_data[6,:,:] = NP      

    # Store training data
    training.coefficients = training_data
    
