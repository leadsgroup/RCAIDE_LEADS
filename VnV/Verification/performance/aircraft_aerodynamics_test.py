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
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Boeing_737    import vehicle_setup as b737_vehicle_setup 
from BWB    import vehicle_setup  as   bwb_vehicle_setup
# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main(): 
    Boeing_737_Drag_Polar()
    BWB_Drag_Polar()

    return 

def Boeing_737_Drag_Polar():

    vehicle                               = b737_vehicle_setup()   
    angle_of_attack_range                 = np.atleast_2d(np.linspace(-5, 25, 18)).T*Units.degrees   
    Mach_number_range                     = np.ones_like(angle_of_attack_range) * 0.78
    aerodynamics_analysis_routine         = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics_analysis_routine.vehicle = vehicle
    temperatures                          = np.ones_like(angle_of_attack_range) * 340
    non_dimensional_reynolds_numbers      = np.ones_like(angle_of_attack_range) * 1E7
    results                               = aircraft_aerodynamic_analysis(aerodynamics_analysis = aerodynamics_analysis_routine,
                                                                          angle_of_attacks = angle_of_attack_range,
                                                                      non_dimensional_reynolds_numbers = non_dimensional_reynolds_numbers,
                                                                      temperatures                     = temperatures,
                                                                      mach_numbers = Mach_number_range)

    results                           = aircraft_aerodynamic_analysis(aerodynamics_analysis = aerodynamics_analysis_routine,
                                                                      angle_of_attacks = angle_of_attack_range,
                                                                      mach_numbers = Mach_number_range,
                                                                      altitude  = 0)

    CL_truth = np.array([-0.41735855, -0.29238238, -0.1674062 , -0.04243003,  0.08293259,
                         0.20829521,  0.3333446 ,  0.458394  ,  0.58227236,  0.70615071,
                         0.83002907,  0.95140521,  1.07278135,  1.19415749,  1.31092408,
                         1.42769068,  1.54445727,  1.66122386])

    CD_truth = np.array([0.02559086, 0.02399367, 0.02259914, 0.02140821, 0.02135306,
                         0.02150681, 0.02263618, 0.02397833, 0.02648091, 0.02918689,
                         0.03208667, 0.03624975, 0.04057963, 0.04507509, 0.05084044,
                         0.05677429, 0.06288579, 0.0691809 ]) 

    plot_aircraft_aerodynamics(results)

    #------------------------------------------------------------------------
    # setup figures
    #------------------------------------------------------------------------
    fig = plt.figure()  
    fig.set_size_inches(12,6) 
    axis_1 = fig.add_subplot(1, 2, 1)
    axis_2 = fig.add_subplot(1, 2, 2) 

    axis_1.plot( results.alpha/Units.degree, results.lift_coefficient, 'bo-',  label = 'New Results') 
    axis_1.plot( results.alpha/Units.degree, CL_truth, 'rs-',  label = 'Old Results') 
    axis_2.plot( results.alpha/Units.degree, results.drag_coefficient, 'bo-',  label = 'New Results') 
    axis_2.plot( results.alpha/Units.degree, CD_truth, 'rs-',  label = 'Old Results') 

    axis_1.set_xlabel('AoA') 
    axis_2.set_xlabel('AoA')  
    axis_1.set_ylabel('$C_L$') 
    axis_2.set_ylabel('$C_D$')   

    axis_1.legend()
    axis_2.legend()
    return  



def BWB_Drag_Polar():

    vehicle                               = bwb_vehicle_setup()   
    angle_of_attack_range                 = np.atleast_2d(np.linspace(-5, 25, 18)).T*Units.degrees   
    Mach_number_range                     = np.ones_like(angle_of_attack_range) * 0.78
    aerodynamics_analysis_routine         = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics_analysis_routine.vehicle = vehicle
    temperatures                          = np.ones_like(angle_of_attack_range) * 340
    non_dimensional_reynolds_numbers      = np.ones_like(angle_of_attack_range) * 1E7
    results                               = aircraft_aerodynamic_analysis(aerodynamics_analysis = aerodynamics_analysis_routine,
                                                                          angle_of_attacks = angle_of_attack_range,
                                                                      non_dimensional_reynolds_numbers = non_dimensional_reynolds_numbers,
                                                                      temperatures                     = temperatures,
                                                                      mach_numbers = Mach_number_range)

    results                           = aircraft_aerodynamic_analysis(aerodynamics_analysis = aerodynamics_analysis_routine,
                                                                      angle_of_attacks = angle_of_attack_range,
                                                                      mach_numbers = Mach_number_range,
                                                                      altitude  = 0)

    CL_truth = np.array([-0.66537549, -0.43113207, -0.19644241,  0.03928848,  0.27496979,
                         0.50921321,  0.74216586,  0.97253695,  1.19801695,  1.42066528,
                         1.59950605,  1.70267922,  1.80585239,  1.90902557,  2.01219874,
                         2.11537191,  2.21854509,  2.32171826])

    CD_truth = np.array([0.08133093, 0.03386143, 0.01340683, 0.01032553, 0.01485286,
                         0.02576822, 0.05289148, 0.0994132 , 0.16812622, 0.26008066,
                         0.41868775, 0.66980037, 1.03969088, 1.56379824, 2.28231239,
                         3.24017418, 4.48707534, 6.07745849]) # values are way too high

    plot_aircraft_aerodynamics(results)

    #------------------------------------------------------------------------
    # setup figures
    #------------------------------------------------------------------------
    fig = plt.figure()  
    fig.set_size_inches(12,6) 
    axis_1 = fig.add_subplot(1, 2, 1)
    axis_2 = fig.add_subplot(1, 2, 2) 

    axis_1.plot( results.alpha/Units.degree, results.lift_coefficient, 'bo-',  label = 'New Results') 
    axis_1.plot( results.alpha/Units.degree, CL_truth, 'rs-',  label = 'Old Results') 
    axis_2.plot( results.alpha/Units.degree, results.drag_coefficient, 'bo-',  label = 'New Results') 
    axis_2.plot( results.alpha/Units.degree, CD_truth, 'rs-',  label = 'Old Results') 

    axis_1.set_xlabel('AoA') 
    axis_2.set_xlabel('AoA')  
    axis_1.set_ylabel('$C_L$') 
    axis_2.set_ylabel('$C_D$')   

    axis_1.legend()
    axis_2.legend()
    return  



if __name__ == '__main__': 
    main()    
    plt.show()