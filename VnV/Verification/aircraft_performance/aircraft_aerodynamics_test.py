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
    
    angle_of_attack_range                 = np.atleast_2d(np.linspace(-1, 1, 3)).T*Units.degrees   
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

    CL_truth = np.array([0.46705086, 0.64126679, 0.81505847])


    CD_truth = np.array([0.02162666, 0.02365738, 0.02865083])
                      
    # plot results 
    plot_aircraft_aerodynamics(results, save_filename = "B737_Aircraft_Aerodynamic_Analysis")
    plot_pressure_coefficient_distribution(results)
    
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
    
    angle_of_attack_range                 = np.atleast_2d(np.linspace(-1, 1, 3)).T*Units.degrees   
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

    CL_truth = np.array([-0.17632756,  0.        ,  0.17632756])

    CD_truth = np.array([0.01744086, 0.0134204 , 0.01397554])

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
    aerodynamics.settings.use_surrogate = False 
    analyses.append(aerodynamics)
    
    return analyses 


     
if __name__ == '__main__': 
    main()    
    plt.show()