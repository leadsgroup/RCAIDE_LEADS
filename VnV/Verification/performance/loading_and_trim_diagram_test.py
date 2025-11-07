# Regression/scripts/Tests/load_diagram_test.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Core import Units  , Container
from RCAIDE.Library.Methods.Performance.compute_load_and_trim_diagram        import compute_load_and_trim_diagram
from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt
from RCAIDE.Library.Plots import  * 

# python imports      
import os
import numpy as np  
import pickle
import sys 
import numpy as np
import matplotlib.pyplot as plt

# local imports 
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Embraer_190    import vehicle_setup as E190_vehicle_setup       

# ----------------------------------------------------------------------------------------------------------------------
#  REGRESSION
# ----------------------------------------------------------------------------------------------------------------------  
def main():
    
    
    vehicle    = E190_vehicle_setup()
    
    vehicle.mass_properties.payload =  vehicle.mass_properties.max_payload
     
    # take out control surfaces to make regression run faster
    for wing in vehicle.wings:
        wing.control_surfaces  = Container()
        
    #  Weights Analysis
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_Transport() 
    weights.settings.FLOPS.fidelity = 'Complex' 
    weights.vehicle = vehicle
 
    #  Aerodynamics Analysis 
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.number_of_spanwise_vortices   = 5
    aerodynamics.settings.number_of_chordwise_vortices  = 2  
    
    #  Stability Analysis
    stability     = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method()  
    stability.settings.number_of_spanwise_vortices   = 5
    stability.settings.number_of_chordwise_vortices  = 2       
    
    load_data =  compute_load_and_trim_diagram(vehicle,
                                          aerodynamic_analysis = aerodynamics,
                                          weights_analysis     = weights,
                                          stability_analysis   = stability,
                                          altitude             = 35000*Units.feet,
                                          airspeed             = 450 * Units['knots'],
                                          number_of_points     = 3)
    
    save_results(load_data,'loading_results') 
 
    LEMAC_truth = np.array([[-37.96970039,  51.73364162, 141.43698363],
                            [-37.96970039,  51.73364162, 141.43698363],
                            [-37.96970039,  51.73364162, 141.43698363]])

    plot_load_diagram(load_data) 

    LEMAC_error = np.max(abs((load_data.aerodynamic_LEMAC_location - LEMAC_truth)/LEMAC_truth))
    print(f"LEMAC error: {LEMAC_error}")
    assert LEMAC_error < 1e-4, f"LEMAC error too large: {LEMAC_error}"
        
    return


def save_results(data,filename): 
    pickle_file  = filename + '.pkl'
    with open(pickle_file, 'wb') as file:
        pickle.dump(data, file) 
    return 


def load_results(filename):  
    load_file = filename + '.pkl' 
    with open(load_file, 'rb') as file:
        results = pickle.load(file) 
    return results 

 
if __name__ == '__main__': 
    main()
    plt.show()