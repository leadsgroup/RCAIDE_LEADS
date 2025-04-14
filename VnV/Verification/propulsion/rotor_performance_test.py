'''

The script below documents how to set up and plot the results of an isolated/static propeller analysis  

''' 
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
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles' + os.path.sep + 'Rotors'))
from Test_Propeller    import Test_Propeller
from Test_Rotor        import Test_Rotor 

# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():
    
    #propeller_test()
    rotor_test(new_regression=False)
    
    return

def propeller_test():
    
    # define propeller 
    propeller      = Test_Propeller()
    
    # define velocity range 
    velocity_range  = np.linspace(10, 100, 19) 
    
    # define RPM
    angular_velocity = 2500*Units.rpm
    
    # define angle of attack
    angle_of_attack =  0 * Units.degrees
    
    # run analysis
    results        = rotor_aerodynamic_analysis(propeller, velocity_range, angular_velocity = angular_velocity, angle_of_attack=angle_of_attack) 
     
    plot_rotor_disc_performance(propeller,results,i=0,title=None,save_figure=False) 
    plot_rotor_performance(propeller,results,title=None,save_figure=False, show_figure=False)
    
    thrust      = np.linalg.norm(results.thrust,axis=1)[0]
    thrust_true = 10043.181397103635

    diff_thrust = np.abs((thrust- thrust_true)/thrust_true)  
    print('\nthrust difference')
    print(diff_thrust)
    assert diff_thrust  < 1e-3
    
    # plot propeller 
    plot_3d_rotor(propeller,save_figure=False, show_figure=False) 
        
    return


def rotor_test(new_regression):
    # define rotor 
    rotor      = Test_Rotor(new_regression)
    rotor.orientation_euler_angles  = [0.,90 * Units.degrees ,0.]
    
    # define velocity range 
    velocity_range  = np.linspace(1, 5, 10).T
    
    # define RPM
    angular_velocity = 2500*Units.rpm
    
    # define angle of attack
    angle_of_attack = -90 * Units.degrees
    
    # run analysis
    results        = rotor_aerodynamic_analysis(rotor, velocity_range, angular_velocity = angular_velocity, angle_of_attack=angle_of_attack)
    
    thrust      = np.linalg.norm(results.thrust,axis=1)[0]
    thrust_true = 11498.853524041924

    diff_thrust = np.abs((thrust- thrust_true)/thrust_true)  
    print('\nthrust difference')
    print(diff_thrust)
    assert diff_thrust  < 1e-3
    

if __name__ == '__main__':
    main()
    plt.show()
