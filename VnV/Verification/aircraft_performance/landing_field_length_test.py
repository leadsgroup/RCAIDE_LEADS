# landing_field_length_test.py
#
# Created:  Mar, 2026, M. Clarke 

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE 
from RCAIDE.Library.Methods.Performance.estimate_landing_field_length import estimate_landing_field_length 

import numpy as np
import pylab as plt
import sys
import os
import numpy as np

# import vehicle file
base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Embraer_190 import vehicle_setup, configs_setup  




def main():
    # define vehicle 
    vehicle   = vehicle_setup()   
  
    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = analyses_setup(configs)
    
    landing_weight = 40000
    landing_field_length = estimate_landing_field_length( analyses = analyses.landing, 
                                                          landing_weight =landing_weight) 
    
    truth_LFL =  1429.1122681863346
    print('Weight (kg): ', landing_weight)
    print('Landing Field Length (m): ',landing_field_length)
    LFL_error = np.max(np.abs(landing_field_length-truth_LFL))
    assert(LFL_error<1e-6)
 
 
    return



def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses 

def base_analysis(vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle() 
    analyses.vehicle =  vehicle
    
    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()   
    analyses.append(geometry)

     # ------------------------------------------------------------------
    #  Weights 
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_Transport()    
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis  
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.settings.maximum_lift_coefficient_factor = 0.90
    analyses.append(aerodynamics) 

    # ------------------------------------------------------------------
    #  Energy
    energy          = RCAIDE.Framework.Analyses.Energy.Energy() 
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    # done!
    return analyses 
 

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()
    plt.show()

