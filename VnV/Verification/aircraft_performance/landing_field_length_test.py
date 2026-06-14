# test_landing_field_length.py
#
# Created:  Tarik, Carlos, Celso, Jun 2014
# Modified: Emilio Botero 
#           Mar 2020, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE
from RCAIDE.Framework.Core   import Data , Units 
from RCAIDE.Library.Methods.Performance.estimate_landing_field_length import estimate_landing_field_length
from RCAIDE.Library.Methods.Geometry.Planform import wing_planform

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
<<<<<<< HEAD
    # define vehicle 
    vehicle   = vehicle_setup()
    vehicle.mass_properties.landing = 40000
     
    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = analyses_setup(configs) 
    
    landing_field_length = estimate_landing_field_length( analyses = analyses.landing) 
    
    truth_LFL =  1429.1122681863346
    print('Weight (kg): ', vehicle.mass_properties.landing)
    print('Landing Field Length (m): ',landing_field_length)
=======

    # ----------------------------------------------------------------------
    #   Main
    # ----------------------------------------------------------------------

    # --- Vehicle definition ---
    vehicle = vehicle_setup()
    for wing in vehicle.wings: 
        wing_planform(wing) 
        if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
            vehicle.reference_area = wing.areas.reference
    configs = configs_setup(vehicle)
    configs = configs_setup(vehicle)

    # --- Landing Configuration ---
    landing_config = configs.landing
    landing_config.wings['main_wing'].control_surfaces.flap.deflection = 30. * Units.deg
    landing_config.wings['main_wing'].control_surfaces.slat.deflection = 25. * Units.deg
    landing_config.wings['main_wing'].high_lift  = True
    # Vref_V2_ratio may be informed by user. If not, use default value (1.23)
    landing_config.Vref_VS_ratio = 1.23

    # CLmax for a given configuration may be informed by user
    # Used defined ajust factor for maximum lift coefficient
    analyses = base_analysis(vehicle)
    analyses.aerodynamics.settings.maximum_lift_coefficient_factor = 0.90

    # =====================================
    # Landing field length evaluation
    # =====================================
    w_vec = np.linspace(20000.,44000.,10)
    landing_field_length = np.zeros_like(w_vec)
    for id_w,weight in enumerate(w_vec):
        landing_config.mass_properties.landing = weight
        landing_field_length[id_w] = estimate_landing_field_length(landing_config,analyses)

    truth_LFL = np.array([ 843.74918006,  922.9157374 , 1002.08229474, 1081.24885208,
                            1160.41540942, 1239.58196676, 1318.7485241 , 1397.91508144,
                            1477.08163878, 1556.24819612])
>>>>>>> 6f04b72118c5f63837bcb8fbde3ed393ec4a9466
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

<<<<<<< HEAD
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
=======
    #  Aerodynamics Analysis
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
    aerodynamics.settings.number_of_spanwise_vortices    = 10 # reducing the number of vortices to speed up the test 
    aerodynamics.settings.number_of_chordwise_vortices   = 5  # reducing the number of vortices to speed up the test 
    analyses.append(aerodynamics)
>>>>>>> 6f04b72118c5f63837bcb8fbde3ed393ec4a9466

    # done!
    return analyses 
 

# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()
    plt.show()

