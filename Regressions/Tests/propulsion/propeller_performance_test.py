'''

The script below documents how to set up and plot the results of an isolated/static propeller analysis  

''' 
#--------------------------------------------------------------------------------------------
#   Imports
# -------------------------------------------------------------------------------------------
 
from RCAIDE.Framework.Core                              import Units
from RCAIDE.Library.Plots                               import *     
from RCAIDE.Library.Methods.Performance                 import propeller_aerodynamic_analysis

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

# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main():
    # define propeller 
    propeller      = Test_Propeller()
    
    # define velocity range 
    velocity_range =  np.atleast_2d(np.linspace(10, 100, 29)).T
    
    # define RPM
    angular_velocity = 2500*Units.rpm
    
    # run pr
    results        = propeller_aerodynamic_analysis(propeller, velocity_range,
                                                    angular_velocity = angular_velocity,
                                                    angle_of_attack = 0, 
                                                    altitude = 0,
                                                    delta_isa =0 ) 
    
     
    plot_rotor_disc_performance(propeller,results,i=0,title=None,save_figure=False) 
    plot_rotor_performance(propeller,results,title=None,save_figure=False, show_figure=False) 
    
    thrust      = results.thrust[0][0]
    thrust_true = 9055.113684358737

    diff_thrust = np.abs((thrust- thrust_true)/thrust_true)  
    print('\nthrust difference')
    print(diff_thrust)
    assert diff_thrust  < 1e-3
    
    # plot propeller 
    plot_3d_rotor(propeller,save_figure=False, show_figure=False) 
        
    return



if __name__ == '__main__':
    main()
    plt.show()
