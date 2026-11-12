# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/build_VLM_surrogates.py
#
# Created: Aug 2025, M. Clarke
# Modified: Oct 2026, H. Hussien

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core import  Data
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.control_surface_registry import CONTROL_SURFACE_TYPES, HIGH_LIFT_LETTERS, HIGH_LIFT_DRAG_TERMS

# package imports
from RCAIDE.Library.Methods.Utilities.Linear_Interpolator import Linear_Interpolator

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ----------------------------------------------------------------------------------------------------------------------   
def build_VLM_surrogates(aerodynamics, vehicle):
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
    if len(sup_Mach) > 2 and training.supersonic is not None: 
        surrogates.supersonic  =  build_surrogate(aerodynamics, training.supersonic, vehicle)
        surrogates.transonic   =  build_surrogate(aerodynamics, training.transonic, vehicle)
    else: 
        surrogates.supersonic  =  no_surrogate(aerodynamics, training.supersonic, vehicle)
        surrogates.transonic   =  no_surrogate(aerodynamics, training.transonic, vehicle)        

    # supersonic training deferred until a flight condition needs it (see evaluate_surrogate)
    surrogates.supersonic_pending = len(sup_Mach) > 2 and training.supersonic is None
    return

def build_surrogate(aerodynamics, training, vehicle):
    
    # unpack data
    surrogates     = Data()
    mach_data      = training.Mach
    AoA_data       = aerodynamics.training.angle_of_attack     
    
    # Pack the outputs
    surrogates.Clift_alpha        = Linear_Interpolator([AoA_data, mach_data], training.Clift_alpha)      
    surrogates.Cdrag_induced_alpha= Linear_Interpolator([AoA_data, mach_data], training.Cdrag_induced_alpha)      
    surrogates.CM_alpha           = Linear_Interpolator([AoA_data, mach_data], training.CM_alpha)  
    surrogates.CX_alpha           = Linear_Interpolator([AoA_data, mach_data], training.CX_alpha)  
    surrogates.CZ_alpha           = Linear_Interpolator([AoA_data, mach_data], training.CZ_alpha)
    surrogates.CY_alpha           = Linear_Interpolator([AoA_data, mach_data], training.CY_alpha)
    surrogates.CL_alpha           = Linear_Interpolator([AoA_data, mach_data], training.CL_alpha)
    surrogates.CN_alpha           = Linear_Interpolator([AoA_data, mach_data], training.CN_alpha)  
    surrogates.Clift_spanwise     = Linear_Interpolator([AoA_data, mach_data], training.Clift_spanwise)      
    if 'gamma_alpha' in training: # training data stored before vortex strengths were added has no gamma_alpha
        surrogates.gamma_alpha    = Linear_Interpolator([AoA_data, mach_data], training.gamma_alpha)      
    else:
        surrogates.gamma_alpha    = None



    # Use interpolat.interp1d below
    surrogates.CM_zero_lift     = Linear_Interpolator([mach_data], training.CM_zero_lift)
    
    surrogates.dCY_dbeta       = Linear_Interpolator([AoA_data, mach_data], training.dCY_dbeta)
    surrogates.dCY_dr          = Linear_Interpolator([AoA_data, mach_data], training.dCY_dr)
    surrogates.dCY_dp           = Linear_Interpolator([AoA_data, mach_data], training.dCY_dp)
    
    surrogates.dCZ_dq          = Linear_Interpolator([AoA_data, mach_data], training.dCZ_dq)
    
    surrogates.dCL_dbeta       = Linear_Interpolator([AoA_data, mach_data], training.dCL_dbeta)
    surrogates.dCL_dp          = Linear_Interpolator([AoA_data, mach_data], training.dCL_dp)
    surrogates.dCL_dr          = Linear_Interpolator([AoA_data, mach_data], training.dCL_dr)
    
    surrogates.dCM_dq          = Linear_Interpolator([AoA_data, mach_data], training.dCM_dq)
    
    surrogates.dCN_dbeta       = Linear_Interpolator([AoA_data, mach_data], training.dCN_dbeta)
    surrogates.dCN_dp          = Linear_Interpolator([AoA_data, mach_data], training.dCN_dp)
    surrogates.dCN_dr          = Linear_Interpolator([AoA_data, mach_data], training.dCN_dr)
   

    # see control_surface_registry.py for why every control surface is handled through one generic path here
    for cls, letter, name, channel, flag, deflection_attr in CONTROL_SURFACE_TYPES:
        if getattr(aerodynamics, flag):
            for coeff in ('Clift', 'Cdrag', 'CX', 'CY', 'CZ', 'CL', 'CM', 'CN'):
                key = 'd' + coeff + '_ddelta_' + letter
                surrogates[key] = Linear_Interpolator([mach_data], training[key])
    for letter in HIGH_LIFT_LETTERS:
        for term in HIGH_LIFT_DRAG_TERMS:
            key = 'high_lift_Cdrag_' + term + '_' + letter
            if key in training:
                surrogates[key] = Linear_Interpolator([mach_data], training[key])

    return surrogates
 
 
def no_surrogate(aerodynamics, training, vehicle):
    
    # unpack data
    surrogates     = Data()    
     
    # Pack the outputs     
    surrogates.Clift_alpha            = None     
    surrogates.Clift_spanwise         = None
    surrogates.gamma_alpha            = None
    surrogates.Cdrag_induced_alpha    = None     
    surrogates.CX_alpha               = None    
    surrogates.CY_alpha               = None    
    surrogates.CZ_alpha               = None    
    surrogates.CL_alpha               = None    
    surrogates.CM_alpha               = None    
    surrogates.CN_alpha               = None    
    surrogates.CM_zero_lift           = None
    
    surrogates.dCY_dbeta             = None
    surrogates.dCY_dr                = None
    surrogates.dCY_dp                 = None
    surrogates.dCZ_dq                = None
    surrogates.dCL_dbeta             = None
    surrogates.dCL_dp                = None
    surrogates.dCL_dr                = None
    surrogates.dCM_dq                = None
    surrogates.dCN_dbeta             = None
    surrogates.dCN_dp                = None
    surrogates.dCN_dr                = None

    for cls, letter, name, channel, flag, deflection_attr in CONTROL_SURFACE_TYPES:
        if getattr(aerodynamics, flag):
            for coeff in ('Clift', 'Cdrag', 'CX', 'CY', 'CZ', 'CL', 'CM', 'CN'):
                surrogates['d' + coeff + '_ddelta_' + letter] = None

    return surrogates