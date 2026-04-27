'''

The script below documents how to set up and plot the results of polar analysis of full aircraft configuration 

''' 

# ----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------- 
import RCAIDE
from RCAIDE.Framework.Core import Units , Data   
from RCAIDE.Library.Methods.Performance                            import aircraft_aerodynamic_analysis 
from RCAIDE.Library.Plots                                          import *   
import numpy as np
import matplotlib.pyplot  as plt
import os
import  sys

# local imports 
base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Navion    import vehicle_setup, configs_setup
# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():
    
    # Truth Values for Literature RANGE
    truth_vals = Data()  
    truth_vals.Clift_alpha  = np.array([4.2,5.5])
    truth_vals.CY_beta      = np.array([-0.77, -0.2])
    truth_vals.CL_beta      = np.array([-0.1, -0.05])
    truth_vals.CM_alpha     = np.array([-1.24,-0.5])
    truth_vals.CN_beta      = np.array([0.033,0.09])
    truth_vals.CL_p         = np.array([-0.48, -0.3])
    truth_vals.CL_r         = np.array([0.07,0.27])
    truth_vals.CM_q         = np.array([-13.29,-9.5])
    truth_vals.CN_p         = np.array([-0.1, -0.01])
    truth_vals.CN_r         = np.array([-0.14, -0.06]) 
    truth_vals.CM_delta_e   = np.array([-1.5, -1.42]) 
    truth_vals.CL_delta_a   = np.array([-0.152, -0.135 ]) 
    truth_vals.CN_delta_a   = np.array([ -0.0047, -0.0013]) 
    truth_vals.CN_delta_r   = np.array([-0.093, -0.075 ])

    vehicle  = vehicle_setup()    
    configs  = configs_setup(vehicle) 
    analyses = analyses_setup(configs)  

 
    angle_of_attack_range                 = np.array([[2 * Units.degree]])
    Mach_number_range                     = np.ones_like(angle_of_attack_range) * 0.15 
    results                               = aircraft_aerodynamic_analysis(analyses          = analyses.base,
                                                                          angle_of_attacks  = angle_of_attack_range,
                                                                          mach_numbers      = Mach_number_range, 
                                                                          altitude          =  1000. * Units.feet ) 
 
    SSD = results.static_stability.derivatives
    # Display the stability derivatives
    print(f"CLift_alpha: {SSD.Clift_alpha[0,0]:.3f}")
    print(f"CY_beta: {SSD.CY_beta[0,0]:.3f}")
    print(f"CL_beta: {SSD.CL_beta[0,0]:.4f}")
    print(f"CM_alpha: {SSD.CM_alpha[0,0]:.3f}")
    print(f"CN_beta: {SSD.CN_beta[0,0]:.3f}")
    print(f"CL_p: {SSD.CL_p[0,0]:.5f}")
    print(f"CL_r: {SSD.CL_r[0,0]:.5f}")
    print(f"CM_q: {SSD.CM_q[0,0]:.5f}")
    print(f"CN_p: {SSD.CN_p[0,0]:.5f}")
    print(f"CN_r: {SSD.CN_r[0,0]:.5f}")
    print(f"CM_delta_e: {SSD.CM_delta_e[0,0]:.5f}")
    print(f"CL_delta_a: {SSD.CL_delta_a[0,0]:.5f}")
    print(f"CN_delta_a: {SSD.CN_delta_a[0,0]:.5f}")
    print(f"CN_delta_r: {SSD.CN_delta_r[0,0]:.5f}") 
    

    print('Literature Validation ')
    for key, val in list(truth_vals.items()):
        computed = SSD[key][0, 0]
        in_range = truth_vals[key][0] <= computed <= truth_vals[key][1]

        lower_bound_percent_error = 100 * abs((truth_vals[key][0] - computed) / truth_vals[key][0])
        upper_bound_percent_error = 100 * abs((truth_vals[key][1] - computed) / truth_vals[key][1])

        if not in_range:
            if computed < truth_vals[key][0]:
                max_percent_error = lower_bound_percent_error
            else:
                max_percent_error = upper_bound_percent_error
            print(key, round(computed, 5), ' outside range by ', round(max_percent_error, 2), ' % error')
        else:
            print(key, round(computed, 5), ' inside range')

    print('Code Verification ')

    RCAIDE_vals = Data()
    for key in truth_vals.keys():
        RCAIDE_vals[key] = SSD[key][0, 0]

    RCAIDE_error = Data()
    for key, val in list(RCAIDE_vals.items()):
        RCAIDE_error[key] = abs((RCAIDE_vals[key] - SSD[key][0, 0]) / RCAIDE_vals[key]) if RCAIDE_vals[key] != 0 else 0.0

    print('Errors:')
    print(RCAIDE_error)

    for k, v in list(RCAIDE_error.items()):
        print(v)
        assert(np.abs(v) < 1e-6)
         
    
    return 


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in list(configs.items()):
        analysis = base_analysis(config)
        analyses[tag] = analysis
 
    return analyses


def base_analysis(vehicle):

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    analyses.vehicle =  vehicle
     
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry() 
    analyses.append(geometry)
  
    aerodynamics   = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.use_surrogate = False  
    analyses.append(aerodynamics)
    
    return analyses 


     
if __name__ == '__main__': 
    main()    
    plt.show()