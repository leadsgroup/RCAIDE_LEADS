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
from Boeing_737    import vehicle_setup   as B737_vehicle_setup  
from Boeing_737    import configs_setup   as B737_configs_setup 
from BWB           import vehicle_setup   as BWB_vehicle_setup
from BWB           import configs_setup   as BWB_configs_setup 
# ----------------------------------------------------------------------
#   Main
# ---------------------------------------------------------------------- 
def main(): 
    Boeing_737_Drag_Polar()
    BWB_Drag_Polar()

    return 

def Boeing_737_Drag_Polar():

    vehicle  = B737_vehicle_setup()    
    configs  = B737_configs_setup(vehicle) 
    analyses = analyses_setup(configs)  
    
    angle_of_attack_range                 = np.atleast_2d(np.linspace(-5, 25, 18)).T*Units.degrees   
    Mach_number_range                     = np.ones_like(angle_of_attack_range) * 0.78 
    temperatures                          = np.ones_like(angle_of_attack_range) * 340
    non_dimensional_reynolds_numbers      = np.ones_like(angle_of_attack_range) * 1E7
    results                               = aircraft_aerodynamic_analysis(analyses                         = analyses.base,
                                                                          angle_of_attacks                 = angle_of_attack_range,
                                                                          non_dimensional_reynolds_numbers = non_dimensional_reynolds_numbers,
                                                                          temperatures                     = temperatures,
                                                                          mach_numbers                     = Mach_number_range)

    results                           = aircraft_aerodynamic_analysis(analyses                         = analyses.base,
                                                                      angle_of_attacks                 = angle_of_attack_range,
                                                                      mach_numbers = Mach_number_range,
                                                                      altitude  = 0)

    CL_truth = np.array([-0.38186938, -0.10627372,  0.16950056,  0.4455403 ,  0.72072764,
                         0.99313006,  1.26355613,  1.5300295 ,  1.78970029,  2.04543276,
                         2.24649627,  2.35313159,  2.4597669 ,  2.56640222,  2.67303754,
                         2.77967285,  2.88630817,  2.99294349])
    
    CD_truth = np.array([0.06682168, 0.03297028, 0.02150798, 0.02273792, 0.04049766,
                         0.07110456, 0.09480261, 0.10963991, 0.15831249, 0.24051187,
                         0.36893079, 0.56138268, 0.84474023, 1.24610486, 1.79621305,
                         2.5294356 , 3.48377719, 4.70087622])
                      
    # plot results 
    plot_aircraft_aerodynamics(results, save_filename = "B737_Aircraft_Aerodynamic_Analysis")    
    
    # check errors 
    CL_error = np.max(np.abs(results.lift_coefficient[:, 0]-CL_truth))
    assert(CL_error<1e-6)

    CD_truth = np.max(np.abs(results.drag_coefficient[:, 0]-CD_truth))    
    assert(CL_error<1e-6)
      
    
    return   

def BWB_Drag_Polar():

    vehicle  = BWB_vehicle_setup()    
    configs  = BWB_configs_setup(vehicle) 
    analyses = analyses_setup(configs)
    
    angle_of_attack_range                 = np.atleast_2d(np.linspace(-5, 25, 18)).T*Units.degrees   
    Mach_number_range                     = np.ones_like(angle_of_attack_range) * 0.78 
    temperatures                          = np.ones_like(angle_of_attack_range) * 340
    non_dimensional_reynolds_numbers      = np.ones_like(angle_of_attack_range) * 1E7
    results                               = aircraft_aerodynamic_analysis(analyses                         = analyses.cruise,
                                                                          angle_of_attacks                 = angle_of_attack_range,
                                                                          non_dimensional_reynolds_numbers = non_dimensional_reynolds_numbers,
                                                                          temperatures                     = temperatures,
                                                                          mach_numbers                     = Mach_number_range)

    results                           = aircraft_aerodynamic_analysis(analyses         = analyses.cruise,
                                                                      angle_of_attacks = angle_of_attack_range,
                                                                      mach_numbers     = Mach_number_range,
                                                                      altitude         = 0)

    CL_truth = np.array([-0.66537549, -0.43113207, -0.19644241,  0.03928848,  0.27496979,
                         0.50921321,  0.74216586,  0.97253695,  1.19801695,  1.42066528,
                         1.59950605,  1.70267922,  1.80585239,  1.90902557,  2.01219874,
                         2.11537191,  2.21854509,  2.32171826])

    CD_truth = np.array([0.08133093, 0.03386143, 0.01340683, 0.01032553, 0.01485286,
                         0.02576822, 0.05289148, 0.0994132 , 0.16812622, 0.26008066,
                         0.41868775, 0.66980037, 1.03969088, 1.56379824, 2.28231239,
                         3.24017418, 4.48707534, 6.07745849])

    plot_aircraft_aerodynamics(results,  save_filename = "BWB_Aircraft_Aerodynamic_Analysis")


    CL_error = np.max(np.abs(results.lift_coefficient[:, 0]-CL_truth))
    assert(CL_error<1e-6)

    CD_truth = np.max(np.abs(results.drag_coefficient[:, 0]-CD_truth))    
    assert(CL_error<1e-6)
    
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
    analyses.append(aerodynamics)
    
    return analyses 


     
if __name__ == '__main__': 
    main()    
    plt.show()