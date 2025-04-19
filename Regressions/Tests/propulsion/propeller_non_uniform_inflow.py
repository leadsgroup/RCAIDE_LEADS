'''

The script below documents how to set up and plot the results of non uniform propeller infow 

''' 
#--------------------------------------------------------------------------------------------
#   Imports
# -------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core   import Units, Data 
from RCAIDE.Library.Methods.Performance    import propeller_aerodynamic_analysis  
from RCAIDE.Library.Plots import  *

import numpy as np
import pylab as plt
import os
import sys

# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles' + os.path.sep + 'Rotors'))
from Test_Propeller    import Test_Propeller  

#--------------------------------------------------------------------------------------------
#   Imports
# -------------------------------------------------------------------------------------------

def main():
    '''
    This example shows a propeller operating in three cases of nonuniform freestream flow:
    First, a propeller operates at a nonzero thrust angle relative to the freestream.
    Second, a propeller operates with an arbitrary upstream disturbance.
    Third, a propeller operates in the wake of an upstream wing

    ''' 

    #-------------------------------------------------------------
    # test propeller at inclined thrust angle
    #-------------------------------------------------------------
    inclined_angle_test()
    

    #-------------------------------------------------------------
    # test propeller in arbitrary nonuniform freestream disturbance
    #-------------------------------------------------------------    
    arbitrary_nonuniform_freestream_test()
     
    return

def inclined_angle_test():
    
    # define propeller 
    propeller                           = Test_Propeller() 
    propeller.use_2d_analysis           = True
    propeller.orientation_euler_angles  = [0.,20.*Units.degrees,0]
     

    # set operating conditions for propeller test
    # define velocity range 
    velocity_range =  np.array([[49.1744]])
    
    # define RPM
    angular_velocity = 207.16160479940007
    
    # run pr
    results        = propeller_aerodynamic_analysis(propeller, velocity_range,
                                                    angular_velocity = angular_velocity,
                                                    angle_of_attack = 0, 
                                                    altitude = 0,
                                                    delta_isa =0 )
    
    # spin propeller in nonuniform flow
    thrust  = np.linalg.norm( results.thrust )
    torque  = results.torque[0][0] 
    power   = results.power[0][0] 
    Cp      = results.power_coefficient[0][0] 
    etap    = results.efficiency[0][0]

    # plot velocities at propeller plane and resulting performance
    plot_rotor_disc_performance(propeller,results,title='Case 1: Operating at Thrust Angle')
     
    thrust_r = 4944.824082775179
    torque_r = 1076.6497275518457
    power_r  = 223040.4853664772
    Cp_r     = 0.1266072752046623
    etap_r   = 0.6515825891352817
    print('\nCase 1 Errors: \n')
    print('Thrust difference = ', np.abs(thrust - thrust_r) / thrust_r )
    print('Torque difference = ', np.abs(torque - torque_r) / torque_r )
    print('Power difference = ', np.abs(power - power_r) / power_r )
    print('Cp difference = ', np.abs(Cp - Cp_r) / Cp_r )
    print('Etap difference = ', np.abs(etap - etap_r) / etap_r )
    assert (np.abs(thrust - thrust_r) / thrust_r < 1e-2), "Nonuniform Propeller Thrust Angle Regression Failed at Thrust Test"
    assert (np.abs(torque - torque_r) / torque_r < 1e-2), "Nonuniform Propeller Thrust Angle Regression Failed at Torque Test"
    assert (np.abs(power - power_r) / power_r < 1e-2), "Nonuniform Propeller Thrust Angle Regression Failed at Power Test"
    assert (np.abs(Cp - Cp_r) / Cp_r < 1e-2), "Nonuniform Propeller Thrust Angle Regression Failed at Power Coefficient Test"
    assert (np.abs(etap - etap_r) / etap_r < 1e-2), "Nonuniform Propeller Thrust Angle Regression Failed at Efficiency Test"

    return

def arbitrary_nonuniform_freestream_test():
    

    # define propeller 
    propeller                           = Test_Propeller() 
    propeller.use_2d_analysis           = True
    propeller.nonuniform_freestream     = True
    propeller.orientation_euler_angles  = [0,0,0]
     

    # set an arbitrary nonuniform freestream disturbance
    Na             = 16
    Nr             = 20
    psi            = np.linspace(0,2*np.pi,Na+1)[:-1]
    psi_2d         = np.tile(np.atleast_2d(psi),(Nr,1))
    psi_2d         = np.repeat(psi_2d[None,:,:], 1, axis=0)

    va = (1+psi_2d) * 1.1
    vt = (1+psi_2d) * 2.0
    vr = (1+psi_2d) * 0.9

    propeller.tangential_velocities_2d = vt
    propeller.axial_velocities_2d      = va
    propeller.radial_velocities_2d     = vr
    

    # set operating conditions for propeller test
    # define velocity range 
    velocity_range =  np.array([[49.1744]])
    
    # define RPM
    angular_velocity = 2500*Units.rpm 

    # run pr
    results        = propeller_aerodynamic_analysis(propeller, velocity_range,
                                                    angular_velocity = angular_velocity,
                                                    angle_of_attack = 0, 
                                                    altitude = 0,
                                                    delta_isa =0 )
    
    # spin propeller in nonuniform flow
    thrust  = np.linalg.norm( results.thrust )
    torque  = results.torque[0][0] 
    power   = results.power[0][0] 
    Cp      = results.power_coefficient[0][0] 
    etap    = results.efficiency[0][0]
      
    # plot velocities at propeller plane and resulting performance
    plot_rotor_disc_performance(propeller,results,title='Case 2: Arbitrary Freestream')

    # expected results 
    thrust_r = 5331.676911368302
    torque_r = 1326.7646919395129
    power_r  = 347346.18410329154
    Cp_r     = 0.09769209269258024
    etap_r   = 0.7548147211901529
    print('\nCase 2 Errors: \n')
    print('Thrust difference = ', np.abs(thrust - thrust_r) / thrust_r )
    print('Torque difference = ', np.abs(torque - torque_r) / torque_r )
    print('Power difference = ', np.abs(power - power_r) / power_r )
    print('Cp difference = ', np.abs(Cp - Cp_r) / Cp_r )
    print('Etap difference = ', np.abs(etap - etap_r) / etap_r )
    assert (np.abs(thrust - thrust_r) / thrust_r < 1e-2), "Nonuniform Propeller Inflow Regression Failed at Thrust Test"
    assert (np.abs(torque - torque_r) / torque_r < 1e-2), "Nonuniform Propeller Inflow Regression Failed at Torque Test"
    assert (np.abs(power - power_r) / power_r < 1e-2), "Nonuniform Propeller Inflow Regression Failed at Power Test"
    assert (np.abs(Cp - Cp_r) / Cp_r < 1e-2), "Nonuniform Propeller Inflow Regression Failed at Power Coefficient Test"
    assert (np.abs(etap - etap_r) / etap_r < 1e-2), "Nonuniform Propeller Inflow Regression Failed at Efficiency Test"

    return

if __name__ == '__main__':
    main()
    plt.show()
