#--------------------------------------------------------------------------------------------
#   Imports
# -------------------------------------------------------------------------------------------
 
from RCAIDE.Framework.Core                              import Units
from RCAIDE.Library.Plots                               import *     
from RCAIDE.Library.Methods.Performance                 import rotor_aerodynamic_analysis

import os
import numpy as np 
import matplotlib.pyplot as plt


# python imports 
import numpy as np
import pylab as plt 
import sys
import os

# local imports 
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'Verification', 'Vehicles', 'Rotors')))
from APC_11x4_Propeller    import APC_11x4_Propeller

# ----------------------------------------------------------------------
#   Reference
# ----------------------------------------------------------------------

# APC 11x4 propeller
# https://www.apcprop.com/technical-information/performance-data/

# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():
    
    propeller_test()
    
    return

def propeller_test():
    
    # define propeller 
    propeller      = APC_11x4_Propeller()
    
    # define velocity range 
    velocity_range  = np.linspace(0, 10, 50) 
    
    # define RPM
    angular_velocity = 1000*Units.rpm
    
    # define angle of attack
    angle_of_attack =  0 * Units.degrees
    
    # run analysis
    results        = rotor_aerodynamic_analysis(propeller, velocity_range, angular_velocity = angular_velocity, angle_of_attack=angle_of_attack) 

    literature_advance_ratio = np.array([0.0000, 0.0186, 0.0371, 0.0557, 0.0742, 0.0928, 0.1113, 0.1299, 0.1484, 0.1670, 0.1855, 0.2041, 0.2226, 0.2412, 0.2597, 0.2783, 0.2968, 0.3154, 0.3339, 0.3525, 0.3710, 0.3896, 0.4081, 0.4267, 0.4452, 0.4638, 0.4823, 0.5009, 0.5195, 0.5380])

    literature_thrust = np.array([0.169, 0.165, 0.162, 0.158, 0.154, 0.149, 0.145, 0.140, 0.135, 0.130, 0.125, 0.120, 0.114, 0.109, 0.103, 0.097, 0.091, 0.084, 0.078, 0.071, 0.065, 0.058, 0.051, 0.044, 0.037, 0.030, 0.023, 0.016, 0.008, 0.001])

    fig, ax = plt.subplots(figsize=(10, 5))
    label_fontsize = 16
    tick_fontsize = 14
    ax.plot(literature_advance_ratio, literature_thrust, '-', label='literature')
    ax.plot(results.advance_ratio, results.thrust[:,0], '-', label='RCAIDE')
    ax.set_xlim(0, 0.4)
    ax.set_ylim(0, 0.2)
    ax.set_xlabel('J [-]', fontsize=label_fontsize)
    ax.set_ylabel('Thrust [N]', fontsize=label_fontsize)
    ax.tick_params(axis='both', which='major', labelsize=tick_fontsize)
    ax.legend(loc='upper right', fontsize=label_fontsize, ncol=2)
    fig.tight_layout()

    error = np.abs((results.thrust[0,0] - literature_thrust[0]) / literature_thrust[0]) * 100
    print("\nError in Thrust [%]:", error)
    assert error < 10
    
    return

if __name__ == '__main__':
    main()
    plt.show()
