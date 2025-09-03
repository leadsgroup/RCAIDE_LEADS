# RCAIDE/Library/Methods/Aerodynamics/Vortex_Lattice_Method/train_SU2_surrogates.py
#  
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports  
import RCAIDE 
from RCAIDE.Framework.Core import  Data 
from RCAIDE.Library.Methods.Aerodynamics.SU2.SU2   import SU2 

# package imports
from copy import deepcopy
import pickle
import os
import shutil
import numpy  as np

# ----------------------------------------------------------------------------------------------------------------------
#  Vortex_Lattice
# ---------------------------------------------------------------------------------------------------------------------- 
def train_SU2_surrogates(aerodynamics):
    """Call methods to run SU2 for sample point evaluation. 
    
    Assumptions:
    
    Source:
        None

    Args:
        aerodynamics       : SU2 analysis          [unitless] 
        
    Returns: 
        None    
    """
 
    Mach           = aerodynamics.training.Mach   
    vehicle        = deepcopy(aerodynamics.vehicle)
    settings       = aerodynamics.settings
    AoA            = aerodynamics.training.angle_of_attack  
    training       = aerodynamics.training        
    training.Mach  = Mach 
     
    len_Mach       = len(Mach)        
    len_AoA        = len(AoA)   
    
    # --------------------------------------------------------------------------------------------------------------
    # Alpha
    # --------------------------------------------------------------------------------------------------------------
    
    # Setup new array shapes for vectorization  
    AoAs       = np.atleast_2d(np.tile(AoA,len_Mach).T.flatten()).T 
    Machs      = np.atleast_2d(np.repeat(Mach,len_AoA)).T        
     
    conditions                                      = RCAIDE.Framework.Mission.Common.Results()
    conditions.freestream.mach_number               = Machs
    conditions.aerodynamics.angles.alpha            = np.ones_like(Machs)*AoAs 
    conditions.freestream.pressure                  = np.ones_like(Machs)*training.pressure
    conditions.freestream.temperature               = np.ones_like(Machs)*training.temperature
    
    # Call SU2 
    if settings.run_new_SU2_sim:
        SU2_results      = SU2(conditions,settings,vehicle)
        file_path = 'SU2_surrogate.pkl'
        with open(file_path, 'wb') as file:
            pickle.dump(SU2_results, file)
        shutil.move(file_path, f"../{file_path}")
        # write and record the Su2 surrogate data
        print("Writing Surrogate Data into SU2_surrogate.pkl")
    else:
        print("Reading Surrogate...")
        if os.path.exists('SU2_surrogate.pkl'):
            print(f"SU2_surrogate.pkl found in {os.getcwd()}")
            with open('SU2_surrogate.pkl', 'rb') as file:
                SU2_results = pickle.load(file)
        else:
            print("SU2_surrogate.pkl NOT found! Have you trained the surrogate before using?")
        
    Clift_res        = SU2_results.CLift
    Cdrag_res        = SU2_results.CDift
    CM_res           = SU2_results.CM
    S_ref            = SU2_results.S_ref
    b_ref            = SU2_results.b_ref
    c_ref            = SU2_results.c_ref
    X_ref            = SU2_results.X_ref
    Y_ref            = SU2_results.Y_ref
    Z_ref            = SU2_results.Z_ref        
    
    Clift_alpha   = np.reshape(Clift_res,(len_Mach,len_AoA)).T 
    Cdrag_alpha   = np.reshape(Cdrag_res,(len_Mach,len_AoA)).T
    CM_alpha      = np.reshape(CM_res,(len_Mach,len_AoA)).T

    aerodynamics.reference_values.S_ref = S_ref
    aerodynamics.reference_values.b_ref = b_ref
    aerodynamics.reference_values.c_ref = c_ref
    aerodynamics.reference_values.X_ref = X_ref
    aerodynamics.reference_values.Y_ref = Y_ref
    aerodynamics.reference_values.Z_ref = Z_ref
    aerodynamics.reference_values.aspect_ratio = (b_ref ** 2) / S_ref
    
    Clift_wing_alpha = Data()
    Cdrag_induced_wing_alpha = Data() 
    for wing in vehicle.wings: 
        Clift_wing_alpha[wing.tag] = np.reshape(SU2_results.CLift_wings[wing.tag],(len_Mach,len_AoA)).T    
        Cdrag_induced_wing_alpha[wing.tag] = np.reshape(SU2_results.CDrag_induced_wings[wing.tag],(len_Mach,len_AoA)).T  
    
    # STABILITY COEFFICIENTS  
    training.Clift_alpha               = Clift_alpha  
    training.Cdrag_alpha               = Cdrag_alpha   
    training.Clift_wing_alpha          = Clift_wing_alpha   
    training.Cdrag_induced_wing_alpha  = Cdrag_induced_wing_alpha      
    training.CM_alpha                  = CM_alpha 
      
    # STABILITY DERIVATIVES 
    training.dClift_dalpha = (Clift_alpha[0,:] - Clift_alpha[1,:]) / (AoA[0] - AoA[1])     
    training.dCM_dalpha    = (CM_alpha[0,:] - CM_alpha[1,:]) / (AoA[0] - AoA[1])       
    return training
        
        
