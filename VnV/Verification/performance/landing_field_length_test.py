# test_landing_field_length.py
#
# Created:  Tarik, Carlos, Celso, Jun 2014
# Modified: Emilio Botero 
#           Mar 2020, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUave Imports
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
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Embraer_190 import vehicle_setup, configs_setup  
  
def base_analysis(vehicle):
    
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    
    #  Aerodynamics Analysis
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.geometry = vehicle 
    analyses.append(aerodynamics)
    
    # done!
    return analyses     


def main():

    # ----------------------------------------------------------------------
    #   Main
    # ----------------------------------------------------------------------

    # --- Vehicle definition ---
    vehicle = vehicle_setup()
    for wing in vehicle.wings: 
         wing_planform(wing,overwrite_reference =  True) 
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

    truth_LFL = np.array([ 839.55613409,  918.16361864,  996.77110318, 1075.37858773,
       1153.98607228, 1232.59355682, 1311.20104137, 1389.80852591,
       1468.41601046, 1547.023495  ])
    LFL_error = np.max(np.abs(landing_field_length-truth_LFL))
    assert(LFL_error<1e-6)
    
    print('Maximum Landing Field Length Error= %.4e' % LFL_error)
    
    title = "LFL vs W"
    plt.figure(1); 
    plt.plot(w_vec,landing_field_length, 'k-', label = 'Landing Field Length') 
    plt.title(title)
    plt.grid(True)

    plt.figure(1); plt.plot(w_vec,truth_LFL, label = 'Landing Field Length (true)')
    legend = plt.legend(loc='lower right')
    plt.xlabel('Weight (kg)')
    plt.ylabel('Landing Field Length (m)')
    
    #assert( LFL_error   < 1e-5 )

    return 
    
# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()
    plt.show()
        
