# control_surfaces_vlm.py
# 
# Created:  July 2021, A. Blaufox
# Modified: 
# 
# File to test all-moving surfaces in VLM

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                       import Data, Units
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method   import VLM
from RCAIDE.Library.Plots  import *
from RCAIDE.load import load  
from RCAIDE.save import save  


import sys
import os
import numpy as np

# import vehicle file
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Lockheed_Martin_F22 import vehicle_setup as vehicle_setup
import matplotlib.pyplot                as plt

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    update_regression_values = True  # should be false unless code functionally changes
    
    # all-moving surface deflection cases
    deflection_configs = get_array_of_deflection_configs()
    
    # get settings and conditions
    conditions = get_conditions()      
    settings   = get_settings()
    n_cases    = len(conditions.freestream.mach_number)
    
    # create results objects    
    results        = Data()
    results.CL     = np.empty(shape=[0,n_cases])
    results.CDi    = np.empty(shape=[0,n_cases])
    results.CM     = np.empty(shape=[0,n_cases])
    results.CY  = np.empty(shape=[0,n_cases])
    results.CL_mom = np.empty(shape=[0,n_cases])
    results.CM = np.empty(shape=[0,n_cases])
    
    # run VLM
    for i,deflection_config in enumerate(deflection_configs):
        geometry    = vehicle_setup(deflection_config=deflection_config)
        data        = VLM(conditions, settings, geometry)
        
        plot_title  = "Deflection Configuration #{}".format(i+1)
        plot_3d_vehicle_vlm_panelization(geometry, show_wing_control_points=False, save_filename=plot_title, show_figure=False)        
        
        results.CL         = np.vstack((results.CL     , data.CLift.flatten()    ))
        results.CDi        = np.vstack((results.CDi    , data.CDrag_induced.flatten()   ))
        results.CM         = np.vstack((results.CM     , data.CM.flatten()    ))
        results.CY         = np.vstack((results.CY     , data.CY.flatten() ))
        results.CL_mom     = np.vstack((results.CL_mom , data.CL.flatten()))
        results.CM         = np.vstack((results.CM     , data.CM.flatten()))      
        
    # save/load results
    if update_regression_values:
        save_results(results)
    results_tr = load_results()
    
    # check results
    for key in results.keys():
        vals    = results[key]
        vals_tr = results_tr[key]
        errors  = (vals-vals_tr)/vals_tr
        
        print('results.{}:'.format(key))
        print(vals)
        print('results_tr.{}:'.format(key))
        print(vals_tr) 
        print('errors:')
        print(errors)
        print('           ')
        
        max_err = np.max(np.abs(errors))
        assert max_err < 1e-6 , 'Failed at {} test'.format(key)
    
    return

# ----------------------------------------------------------------------
#   Setup Functions
# ----------------------------------------------------------------------
def get_array_of_deflection_configs():  
    stabilator_sign_duplicates   = [ 1., -1.,  0.]
    v_tail_right_sign_duplicates = [-1.,  0.,  0.]
    
    stabilator_hinge_fractions   = [0.25, 0.5, 0.75]
    v_tail_right_hinge_fractions = [0.75, 0.0, 0.25]
    
    stabilator_use_constant_hinge_fractions   = [False, False, False]
    v_tail_right_use_constant_hinge_fractions = [False, True , False]
    
    zero_vec = np.array([0.,0.,0.])
    stabilator_hinge_vectors     = [zero_vec*1, zero_vec*1, zero_vec*1]
    v_tail_right_hinge_vectors   = [zero_vec*1, zero_vec*1, np.array([0.,1.,0.])]
    
    deflections                  = [ 0.,  0.,  0.]
    stab_defs                    = [10.,-10., 30.]
    vt_r_defs                    = [10.,-10.,-30.]
    
    n_configs = len(deflections)
    deflection_configs = [Data() for i in range(n_configs)]
    
    for i, deflection_config in enumerate(deflection_configs):
        deflection_config.  stabilator_sign_duplicate =   stabilator_sign_duplicates[i]  
        deflection_config.v_tail_right_sign_duplicate = v_tail_right_sign_duplicates[i]
        
        deflection_config.  stabilator_hinge_fraction =   stabilator_hinge_fractions[i] 
        deflection_config.v_tail_right_hinge_fraction = v_tail_right_hinge_fractions[i]
        
        deflection_config.  stabilator_use_constant_hinge_fraction =   stabilator_use_constant_hinge_fractions[i]
        deflection_config.v_tail_right_use_constant_hinge_fraction = v_tail_right_use_constant_hinge_fractions[i]
        
        deflection_config.  stabilator_hinge_vector   =   stabilator_hinge_vectors[i]
        deflection_config.v_tail_right_hinge_vector   = v_tail_right_hinge_vectors[i]      
        
        deflection_config.deflection                  = deflections[i]                 
        deflection_config.stab_def                    = stab_defs[i]                   
        deflection_config.vt_r_def                    = vt_r_defs[i] 

    return deflection_configs

def get_conditions():
    machs      = np.array([0.4  ,0.4  ,0.4  ,0.4  ,1.4  ,])
    altitudes  = np.array([5000 ,5000 ,5000 ,5000 ,5000 ,])  *Units.ft
    aoas       = np.array([0.   ,6.   ,6.   ,0.   ,6    ,])  *Units.degrees #angle of attack in degrees
    PSIs       = np.array([3.   ,5.   ,0.   ,0.   ,5.   ,])  *Units.degrees #sideslip angle  in degrees
    PITCHQs    = np.array([3.   ,6.   ,0.   ,0.   ,6.   ,])  *Units.degrees #pitch rate      in degrees/s   
    ROLLQs     = np.array([3.   ,6.   ,0.   ,0.   ,6.   ,])  *Units.degrees #roll  rate      in degrees/s
    YAWQs      = np.array([3.   ,6.   ,0.   ,0.   ,6.   ,])  *Units.degrees #yaw   rate      in degrees/s       
    
    atmosphere                              = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    speeds_of_sound                         = atmosphere.compute_values(altitudes).speed_of_sound
    v_infs                                  = machs * speeds_of_sound.flatten()
    conditions = RCAIDE.Framework.Mission.Common.Results()
    conditions.freestream.velocity          = np.atleast_2d(v_infs).T
    conditions.freestream.mach_number       = np.atleast_2d(machs).T   
    conditions.aerodynamics.angles.alpha    = np.atleast_2d(aoas).T
    conditions.aerodynamics.angles.beta     = np.atleast_2d(PSIs).T
    conditions.static_stability.pitch_rate  = np.atleast_2d(PITCHQs).T
    conditions.static_stability.roll_rate   = np.atleast_2d(ROLLQs).T
    conditions.static_stability.yaw_rate    = np.atleast_2d(YAWQs).T
    
    return conditions

def get_settings():
    settings = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method().settings
    settings.number_of_spanwise_vortices        = None
    settings.number_of_chordwise_vortices       = None  
    settings.wing_spanwise_vortices          = 5
    settings.wing_chordwise_vortices         = 4
    settings.fuselage_spanwise_vortices      = 5
    settings.fuselage_chordwise_vortices     = 4
        
    settings.propeller_wake_model            = None
    settings.spanwise_cosine_spacing         = False
    settings.model_fuselage                  = True
    settings.model_nacelle                   = True
    settings.leading_edge_suction_multiplier = 1. 
    settings.discretize_control_surfaces     = True
    settings.use_VORLAX_matrix_calculation   = False    
                
    #misc settings
    settings.show_prints = False
    
    return settings

# ----------------------------------------------------------------------
#   Save/Load Utility Functions
# ----------------------------------------------------------------------
def load_results():
    return load('all_moving_surfaces_vlm_results.res')

def save_results(results):
    print('!####! SAVING NEW REGRESSION RESULTS !####!')
    save(results,'all_moving_surfaces_vlm_results.res')
    return

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    
if __name__ == '__main__':
    main()
    plt.show()
