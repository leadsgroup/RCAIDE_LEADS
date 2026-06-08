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
  
    print('Literature Validation ')
    rows = []
    for key in truth_vals.keys():
        computed  = SSD[key][0, 0]
        lo, hi    = truth_vals[key][0], truth_vals[key][1]
        in_range  = lo <= computed <= hi

        if not in_range:
            lower_err = 100 * abs((lo - computed) / lo)
            upper_err = 100 * abs((hi - computed) / hi)
            pct_err   = lower_err if computed < lo else upper_err
            status    = 'OUTSIDE'
            sign_note = 'opposite sign' if (computed * lo < 0) else 'same sign'
            err_str   = f'{pct_err:.2f}%'
        else:
            status    = 'inside'
            sign_note = ''
            err_str   = ''

        rows.append((key, f'{computed:.5f}', f'[{lo}, {hi}]', status, err_str, sign_note))

    col_headers = ('Derivative', 'Computed', 'Range', 'Status', '% Error', 'Sign')
    col_widths  = [max(len(h), max(len(r[i]) for r in rows)) for i, h in enumerate(col_headers)]
    fmt         = '  '.join(f'{{:<{w}}}' for w in col_widths)
    sep         = '  '.join('-' * w for w in col_widths)

    print(fmt.format(*col_headers))
    print(sep)
    for row in rows:
        print(fmt.format(*row))

    print('Code Verification ')

    RCAIDE_vals = Data() 
    RCAIDE_vals.Clift_alpha = 5.71836275727471
    RCAIDE_vals.CY_beta = -0.14312877260856274
    RCAIDE_vals.CL_beta = -0.07529640050767267
    RCAIDE_vals.CM_alpha = -1.1788082925116026
    RCAIDE_vals.CN_beta = 0.10219800736234788
    RCAIDE_vals.CL_p = -0.4331829968640342
    RCAIDE_vals.CL_r = 0.06303177725789796
    RCAIDE_vals.CM_q = -13.318912390825936
    RCAIDE_vals.CN_p = -0.07239626725266779
    RCAIDE_vals.CN_r = -0.09868917917133957
    RCAIDE_vals.CM_delta_e = -1.6100758451817097
    RCAIDE_vals.CL_delta_a = -0.11580752428597008
    RCAIDE_vals.CN_delta_a = -0.008238547197219613
    RCAIDE_vals.CN_delta_r = -0.07718356528882132
 

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