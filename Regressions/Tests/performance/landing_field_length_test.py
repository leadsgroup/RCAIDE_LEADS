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
    aerodynamics.settings.drag_coefficient_increment = 0.0000
    analyses.append(aerodynamics)
    
    # done!
    return analyses     


def main():

    # ----------------------------------------------------------------------
    #   Main
    # ----------------------------------------------------------------------

    # --- Vehicle definition ---
    vehicle = vehicle_setup()
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

    truth_LFL = np.array([736.0921377 ,  800.90442272,  865.71670775,  930.52899278,
        995.3412778 , 1060.15356283, 1124.96584785, 1189.77813288,
       1254.59041791, 1319.40270293])
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
        
