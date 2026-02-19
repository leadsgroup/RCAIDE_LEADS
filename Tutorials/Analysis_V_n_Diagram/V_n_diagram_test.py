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
from RCAIDE.Library.Methods.Geometry.Planform import wing_planform
import matplotlib.pyplot as plt

# package imports
import numpy as np 
import sys
import os
import numpy as np 
 

def main():
    part_35_V_n_Diagram()
    part_23_V_n_Diagram()
    
    return

def part_35_V_n_Diagram():

    
    vehicle  = Transport_vehicle_setup() 

    vehicle.flight_envelope.category                  = 'normal'
    vehicle.flight_envelope.FAR_part_number           = '25' 
    vehicle.flight_envelope.maximum_lift_coefficient  = 3
    vehicle.flight_envelope.minimum_lift_coefficient  = -1.5 

    for wing in vehicle.wings: 
        wing_planform(wing) 
        if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
            vehicle.reference_area = wing.areas.reference

    analyses = RCAIDE.Framework.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    V_n_data = generate_V_n_diagram(vehicle,analyses)
    
    return    
    
    
def part_23_V_n_Diagram():
    
    vehicle  = GA_vehicle_setup() 

    vehicle.flight_envelope.category                  = 'normal'
    vehicle.flight_envelope.FAR_part_number           = '23' 
    vehicle.flight_envelope.maximum_lift_coefficient  = 3
    vehicle.flight_envelope.minimum_lift_coefficient  = -1.5 

    for wing in vehicle.wings: 
        wing_planform(wing) 
        if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing):
            vehicle.reference_area = wing.areas.reference

    analyses = RCAIDE.Framework.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)   

    V_n_data = generate_V_n_diagram(vehicle,analyses)
    
    return 
# ----------------------------------------------------------------------        
#   Call Main
# ----------------------------------------------------------------------    
if __name__ == '__main__':
    main()    
    plt.show()