# test_take_off_field_length.py
#
# Created: Dec 2024, M Clarke   

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUave Imports
import RCAIDE
from RCAIDE.Framework.Core   import Data 
from RCAIDE.Library.Methods.Performance  import generate_V_n_diagram
from RCAIDE.Library.Plots import *
import matplotlib.pyplot as plt

# package imports
import numpy as np 
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

from  Cessna_172 import vehicle_setup   as GA_vehicle_setup  
from  Cessna_172 import configs_setup   as GA_configs_setup
from  Boeing_737 import vehicle_setup   as Transport_vehicle_setup  
from  Boeing_737 import configs_setup   as Transport_configs_setup

def main():
    part_35_V_n_Diagram()
    part_23_V_n_Diagram()
    
    return

def part_35_V_n_Diagram():

    # Set up vehicle 
    vehicle    = Transport_vehicle_setup()   
  
    # Set up vehicle configs
    configs  = Transport_configs_setup(vehicle)

    # create analyses
    analyses = Transport_analyses_setup(configs)
    
    # Vn Diagram 
    V_n_data = generate_V_n_diagram(analyses=analyses.cruise)
    

    plot_V_n_diagram(V_n_data, vehicle)     


    print(V_n_data.Vs1.positive)
    print(V_n_data.Vs1.negative) 
    print(V_n_data.Va.positive) 
    print(V_n_data.Va.negative) 
    print(V_n_data.Vc)
    print(V_n_data.Vd)
    print(V_n_data.positive_limit_load)
    print(V_n_data.negative_limit_load)
    print(V_n_data.limit_loads.dive.positive)
    print(V_n_data.limit_loads.dive.negative)

    # regression values    
    actual                          = Data()
    actual.Vs1_pos                  = 113.20236642595314
    actual.Vs1_neg                  = 160.09232189231165
    actual.Va_pos                   = 178.9886572134933
    actual.Va_neg                   = 253.1281865422711
    actual.Vc                       = 515.9537531823821
    actual.Vd                       = 644.9421914779776
    actual.limit_load_pos           = 3.5651792948236882
    actual.limit_load_neg           = -2.5
    actual.dive_limit_load_pos      = 2.6032370592648055
    actual.dive_limit_load_neg      = -0.6032370592648053

    # error calculations
    error                         = Data()
    error.Vs1_pos                 = (actual.Vs1_pos - V_n_data.Vs1.positive)/actual.Vs1_pos
    error.Vs1_neg                 = (actual.Vs1_neg - V_n_data.Vs1.negative)/actual.Vs1_neg
    error.Va_pos                  = (actual.Va_pos - V_n_data.Va.positive)/actual.Va_pos
    error.Va_neg                  = (actual.Va_neg - V_n_data.Va.negative)/actual.Va_neg
    error.Vc                      = (actual.Vc - V_n_data.Vc)/actual.Vc
    error.Vd                      = (actual.Vd - V_n_data.Vd)/actual.Vd
    error.limit_load_pos          = (actual.limit_load_pos - V_n_data.positive_limit_load)/actual.limit_load_pos
    error.limit_load_neg          = (actual.limit_load_neg - V_n_data.negative_limit_load)/actual.limit_load_neg
    error.dive_limit_load_pos     = (actual.dive_limit_load_pos - V_n_data.limit_loads.dive.positive)/actual.dive_limit_load_pos
    error.dive_limit_load_neg     = (actual.dive_limit_load_neg - V_n_data.limit_loads.dive.negative)


    for k,v in error.items():
        assert(np.abs(v)<1E-6)  
    
 
    return    
    

def Transport_analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = Transport_base_analysis(config)
        analyses[tag] = analysis

    return analyses 

def Transport_base_analysis(vehicle):
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

    
    
def part_23_V_n_Diagram():

    # Set up vehicle 
    vehicle    = GA_vehicle_setup()   
  
    # Set up vehicle configs
    configs  = GA_configs_setup(vehicle)

    # create analyses
    analyses = GA_analyses_setup(configs)
    
    V_n_data = generate_V_n_diagram(analyses=analyses.cruise) 

    print(V_n_data.Vs1.positive)
    print(V_n_data.Vs1.negative) 
    print(V_n_data.Va.positive) 
    print(V_n_data.Va.negative) 
    print(V_n_data.Vc)
    print(V_n_data.Vd)
    print(V_n_data.positive_limit_load)
    print(V_n_data.negative_limit_load)
    print(V_n_data.limit_loads.dive.positive)
    print(V_n_data.limit_loads.dive.negative)

    # regression values    
    actual                          = Data()
    actual.Vs1_pos                  = 37.98585717834934
    actual.Vs1_neg                  = 53.720114399989036
    actual.Va_pos                   = 74.04806758573127
    actual.Va_neg                   = 104.71978144726074
    actual.Vc                       = 126.33084642567567
    actual.Vd                       = 176.86318499594594
    actual.limit_load_pos           = 4.075665978467823
    actual.limit_load_neg           = -3.8
    actual.dive_limit_load_pos      = 3.8
    actual.dive_limit_load_neg      = -1.1529661849274762

    # error calculations
    error                         = Data()
    error.Vs1_pos                 = (actual.Vs1_pos - V_n_data.Vs1.positive)/actual.Vs1_pos
    error.Vs1_neg                 = (actual.Vs1_neg - V_n_data.Vs1.negative)/actual.Vs1_neg
    error.Va_pos                  = (actual.Va_pos - V_n_data.Va.positive)/actual.Va_pos
    error.Va_neg                  = (actual.Va_neg - V_n_data.Va.negative)/actual.Va_neg
    error.Vc                      = (actual.Vc - V_n_data.Vc)/actual.Vc
    error.Vd                      = (actual.Vd - V_n_data.Vd)/actual.Vd
    error.limit_load_pos          = (actual.limit_load_pos - V_n_data.positive_limit_load)/actual.limit_load_pos
    error.limit_load_neg          = (actual.limit_load_neg - V_n_data.negative_limit_load)/actual.limit_load_neg
    error.dive_limit_load_pos     = (actual.dive_limit_load_pos - V_n_data.limit_loads.dive.positive)/actual.dive_limit_load_pos
    error.dive_limit_load_neg     = (actual.dive_limit_load_neg - V_n_data.limit_loads.dive.negative)


    for k,v in error.items():
        assert(np.abs(v)<1E-6)  

    return



def GA_analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag,config in configs.items():
        analysis = GA_base_analysis(config)
        analyses[tag] = analysis

    return analyses 

def GA_base_analysis(vehicle):
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
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_General_Aviation()    
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis  
    aerodynamics          = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method() 
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