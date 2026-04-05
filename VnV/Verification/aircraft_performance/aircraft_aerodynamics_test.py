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

    CL_truth = np.array([-0.8859043 , -0.57402438, -0.26155033,  0.05231007,  0.36610444,
                        0.67798436,  0.98814569,  1.29486985,  1.59508185,  1.89152364,
                        2.12963852,  2.26700691,  2.4043753 ,  2.54174368,  2.67911207,
                        2.81648046,  2.95384884,  3.09121723])

    CD_truth = np.array([0.04902018, 0.0432557 , 0.02050936, 0.01461097, 0.02046714,
                        0.03695656, 0.08905799, 0.14631973, 0.22536315, 0.32287296,
                        0.33891408, 0.35478732, 0.37077092, 0.38686488, 0.40306919,
                        0.41938387, 0.43580891, 0.45234431])

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