# take_off_weight_from_tofl.py
#
# Created:  Feb 2020 , M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE
from RCAIDE.Framework.Core  import Data,Units 
from RCAIDE.Library.Methods.Performance.estimate_take_off_weight_given_TOFL import estimate_take_off_weight_given_TOFL

# package imports
import numpy as np
import pylab as plt 
import sys
import os
import numpy as np

# import vehicle file
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))

from Embraer_190 import vehicle_setup, configs_setup 

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main():   
    # define vehicle 
    vehicle   = vehicle_setup()   
  
    # Set up vehicle configs
    configs  = configs_setup(vehicle)

    # create analyses
    analyses = analyses_setup(configs)
    
    # specify target TOFL
    target_tofl= 1500
 
    # Compute take off weight given tofl
    MTOW = estimate_take_off_weight_given_TOFL(analyses = analyses.takeoff, 
                                               target_tofl= target_tofl )
    
    print('MTOW for Target TOFL of ', target_tofl, 'is ', MTOW, 'kg')
    
    truth_MTOW = 56980
    MTOW_error = np.max(np.abs(MTOW-truth_MTOW))
    assert(MTOW_error<1e-6)    

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

