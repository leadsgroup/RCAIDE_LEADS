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
def train_AVL_surrogates(aerodynamics):
    """
    Generates training data for AVL surrogate models by running parametric analysis across flight envelope.

    Parameters
    ----------
    aerodynamics : Data
        AVL aerodynamics analysis object containing configuration and training parameters
            - vehicle : Vehicle
                Aircraft configuration with wings, fuselage, and control surfaces
            - training : Data
                Training parameter definitions
                    - angle_of_attack : array_like
                        Array of training angles of attack in radians
                    - Mach : array_like
                        Array of training Mach numbers
            - settings : Data
                Analysis configuration parameters
                    - side_slip_angle : float
                        Sideslip angle for analysis in radians
                    - roll_rate_coefficient : float
                        Non-dimensional roll rate coefficient
                    - pitch_rate_coefficient : float
                        Non-dimensional pitch rate coefficient
                    - lift_coefficient : float, optional
                        Target lift coefficient for trimmed analysis
                    - new_regression_results : bool
                        Flag to generate new training data files
                    - filenames : Data
                        File path configuration for analysis directories
            - training_file : str, optional
                Path to existing training data file to load instead of running analysis

    Returns
    -------
    None
        Training data is stored in aerodynamics.training.coefficients and optionally saved to file

    Notes
    -----
    This function creates a comprehensive training dataset for surrogate model
    development by executing AVL analysis across a parametric grid of flight
    conditions. The training data encompasses the complete flight envelope
    including angles of attack and Mach numbers specified in the training
    configuration.

    For each flight condition, the function extracts seven key aerodynamic
    coefficients: lift (CL), induced drag (CD), span efficiency (e), pitching
    moment (CM), pitch damping derivative (Cm_alpha), yaw stability derivative
    (Cn_beta), and neutral point location (NP).

    The function supports both training data generation and loading from
    existing files. When new_regression_results is enabled, fresh analysis
    is performed and results are saved. Otherwise, existing training files
    can be loaded to skip time-intensive analysis.

    **Major Assumptions**
        * Aircraft configuration remains constant across training envelope
        * Linear variation of coefficients between training points is adequate
        * Standard atmospheric conditions at sea level for reference

    **Definitions**

    'Training Data'
        Systematic collection of input-output pairs used for surrogate model development

    'Flight Envelope'
        Complete range of operational flight conditions for aircraft

    'Parametric Grid'
        Regular sampling of independent variables across their operational ranges

    'Span Efficiency Factor'
        Measure of wing aerodynamic efficiency relative to elliptical lift distribution

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.build_AVL_surrogates : Function that creates surrogate models from training data
    RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976 : Standard atmosphere model
    """ 
 
    run_folder             = os.path.abspath(aerodynamics.settings.filenames.run_folder)
    vehicle                = aerodynamics.vehicle
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
            run_conditions.aerodynamics.coefficients.lift.total= lift_coefficient
        else:
            run_conditions.aerodynamics.coefficients.lift.total= np.array([lift_coefficient]).T  
        run_conditions.static_stability.coefficients.pitch = np.ones_like(run_conditions.aerodynamics.angles.alpha)*pitch_rate_coefficient 

        # Run Analysis at AoA[i] and Mach[i]
        run_AVL_analysis(aerodynamics,run_conditions)
 
        CL[:,i]       = run_conditions.aerodynamics.coefficients.lift.total[:,0]
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
    
