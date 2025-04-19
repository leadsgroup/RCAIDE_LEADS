# test_take_off_field_length.py
#
# Created: Dec 2024, M Clarke   

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUave Imports
import RCAIDE
from RCAIDE.Framework.Core   import Data,Units 
from RCAIDE.Library.Methods.Performance  import generate_V_n_diagram
import matplotlib.pyplot as plt

# package imports
import numpy as np 
import sys
import os
import numpy as np 

# import vehicle file
sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
 
from  Cessna_172 import vehicle_setup  

def main():

    analyses = RCAIDE.Framework.Analyses.Vehicle()
    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)      
    
    altitude  = 0 * Units.m
    delta_ISA = 0 
 
    vehicle  = vehicle_setup()

    vehicle.flight_envelope.category                  = 'normal'
    vehicle.flight_envelope.FAR_part_number           = '23' 
    vehicle.flight_envelope.maximum_lift_coefficient  = 3
    vehicle.flight_envelope.minimum_lift_coefficient  = -1.5 
    
    V_n_data = generate_V_n_diagram(vehicle,analyses,altitude,delta_ISA) 

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
    actual.limit_load_pos           = 4.2830764602576545
    actual.limit_load_neg           = -3.8
    actual.dive_limit_load_pos      = 3.8
    actual.dive_limit_load_neg      = -1.2981535221803582
    
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
    error.dive_limit_load_neg     = (actual.dive_limit_load_neg - V_n_data.limit_loads.dive.negative)/actual.dive_limit_load_neg
      
      
    for k,v in error.items():
        assert(np.abs(v)<1E-6)  

    return 
# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    

if __name__ == '__main__':
    main()    
    plt.show()