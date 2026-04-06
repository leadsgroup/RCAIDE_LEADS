# take_off_weight_from_tofl.py
#
# Created:  Feb 2020 , M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUave Imports
import RCAIDE
from RCAIDE.Framework.Core  import Data,Units 
from RCAIDE.Library.Methods.Performance.estimate_take_off_weight_given_tofl import find_take_off_weight_given_tofl

# package imports
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

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():   
    vehicle = vehicle_setup()
    configs = configs_setup(vehicle)

    # --- Takeoff Configuration ---
    configuration                                = configs.takeoff
    configuration.wings['main_wing'].flaps_angle = 20. * Units.deg
    configuration.wings['main_wing'].slats_angle = 25. * Units.deg 
    configuration.V2_VS_ratio                    = 1.21

    analyses                                     = RCAIDE.Framework.Analyses.Analysis.Container()
    analyses                                     = base_analysis(configuration)
    analyses.aerodynamics.settings.maximum_lift_coefficient_factor = 0.90

    # Set Tofl 
    target_tofl = 1487.92650289 

    # Compute take off weight given tofl
    max_tow = find_take_off_weight_given_tofl(configuration,analyses,target_tofl)

    truth_max_tow = 56980.00000000001
    max_tow_error = np.max(np.abs(max_tow[0]-truth_max_tow)) 
    print('Range Error = %.4e' % max_tow_error)
    assert(max_tow_error   < 1e-6 )

    return  


def base_analysis(vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()  
    analyses.vehicle = vehicle

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics         = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    analyses.append(aerodynamics)
    
    # ------------------------------------------------------------------
    #  Weights Analysis
    weights         = RCAIDE.Framework.Analyses.Weights.Conventional_Transport() 
    analyses.append(weights)    

    # ------------------------------------------------------------------
    #  Energy Analysis
    energy         = RCAIDE.Framework.Analyses.Energy.Energy() 
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

