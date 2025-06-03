# RCAIDE/Library/Methods/Aerodynamics/Athena_Vortex_Lattice/build_AVL_surrogates.py
#
# Created: Oct 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# package imports 
from scipy.interpolate                                           import RegularGridInterpolator 

# ----------------------------------------------------------------------------------------------------------------------
#  build_AVL_surrogates
# ---------------------------------------------------------------------------------------------------------------------- 
def build_AVL_surrogates(aerodynamics):
    """
    Builds surrogate models from AVL training data for rapid aerodynamic coefficient prediction.

    Parameters
    ----------
    aerodynamics : Data
        VLM aerodynamics analysis object containing training data and surrogate containers
            - surrogates : Data
                Container for storing trained surrogate interpolators
            - training : Data
                Training dataset from AVL analysis
                    - angle_of_attack : array_like
                        Training angles of attack in radians
                    - Mach : array_like
                        Training Mach numbers
                    - coefficients : array_like
                        Training aerodynamic coefficients [7, n_aoa, n_mach]
                            - coefficients[0,:,:] : Lift coefficient (CL)
                            - coefficients[1,:,:] : Induced drag coefficient (CDi)
                            - coefficients[2,:,:] : Span efficiency factor (e)
                            - coefficients[3,:,:] : Moment coefficient (CM)
                            - coefficients[4,:,:] : Pitch moment derivative (Cm_alpha)
                            - coefficients[5,:,:] : Yaw moment derivative (Cn_beta)
                            - coefficients[6,:,:] : Neutral point location (NP)

    Returns
    -------
    None
        Surrogate interpolators are stored directly in aerodynamics.surrogates

    Notes
    -----
    This function creates RegularGridInterpolator objects from SciPy.interpolate for each aerodynamic
    coefficient using linear interpolation. The surrogates enable rapid 
    evaluation of aerodynamic properties across the flight envelope without
    requiring repeated AVL analysis.

    The training data must be organized on a regular grid of angle of attack
    and Mach number points. Linear interpolation is used with extrapolation
    allowed beyond the training bounds.

    All surrogate models are stored in the aerodynamics.surrogates container
    with descriptive attribute names for easy access during flight simulation.

    **Major Assumptions**
        * Training data exists on regular rectangular grid
        * Linear interpolation provides sufficient accuracy
        * Extrapolation using nearest values is acceptable. If required, then the user is suggested to change the training points
        * Aerodynamic coefficients vary smoothly with flight conditions

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Athena_Vortex_Lattice.train_AVL_surrogates : Create training data
    """
    surrogates  = aerodynamics.surrogates
    training    = aerodynamics.training  
    AoA_data    = training.angle_of_attack
    mach_data   = training.Mach
    

    CL_data       = training.coefficients[0,:,:]
    CDi_data      = training.coefficients[1,:,:]
    e_data        = training.coefficients[2,:,:]  
    CM_data       = training.coefficients[3,:,:]
    Cm_alpha_data = training.coefficients[4,:,:]
    Cn_beta_data  = training.coefficients[5,:,:]
    NP_data       = training.coefficients[6,:,:] 
   
    surrogates.lift_coefficient            = RegularGridInterpolator((AoA_data, mach_data), CL_data      ,method = 'linear',   bounds_error=False, fill_value=None)   
    surrogates.drag_coefficient            = RegularGridInterpolator((AoA_data, mach_data), CDi_data     ,method = 'linear',   bounds_error=False, fill_value=None)   
    surrogates.span_efficiency_factor      = RegularGridInterpolator((AoA_data, mach_data), e_data       ,method = 'linear',   bounds_error=False, fill_value=None)   
    surrogates.moment_coefficient          = RegularGridInterpolator((AoA_data, mach_data), CM_data      ,method = 'linear',   bounds_error=False, fill_value=None)   
    surrogates.Cm_alpha_moment_coefficient = RegularGridInterpolator((AoA_data, mach_data), Cm_alpha_data,method = 'linear',   bounds_error=False, fill_value=None)    
    surrogates.Cn_beta_moment_coefficient  = RegularGridInterpolator((AoA_data, mach_data), Cn_beta_data ,method = 'linear',   bounds_error=False, fill_value=None)   
    surrogates.neutral_point               = RegularGridInterpolator((AoA_data, mach_data), NP_data      ,method = 'linear',   bounds_error=False, fill_value=None)     
        
    return 