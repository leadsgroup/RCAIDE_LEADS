# fuselage_planform_compute.py
# 
# Created: March 2025

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data,  Units 
from RCAIDE.Library.Methods.Geometry.Planform.fuselage_planform import fuselage_planform
from RCAIDE.Library.Plots import *

import numpy as np 
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------- 
#   Main
# ----------------------------------------------------------------------
def main():
     
     # Boeing 787 Fuselage  
     fuselage      = RCAIDE.Library.Components.Fuselages.Fuselage()
     
     fuselage.fineness.nose      = 1.6
     fuselage.fineness.tail      = 2.0      
     
     cabin         = RCAIDE.Library.Components.Fuselages.Cabins.Cabin() 
     cabin.wide_body  = True     
 
     first_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.First() 
     first_class.number_of_seats_abrest              = 4
     first_class.number_of_rows                      = 7
     first_class.seat_width                          = 35 *  Units.inches
     first_class.seat_arm_rest_width                 = 3 *  Units.inches
     first_class.seat_length                         = 45 *  Units.inches
     first_class.seat_pitch                          = 50 *  Units.inches
     first_class.aile_width                          = 18  *  Units.inches  
     first_class.galley_lavatory_percent_x_locations = [0, 1]       
     first_class.type_A_exit_percent_x_locations     = [0, 1]
     cabin.append_cabin_class(first_class) 
 
     business_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Business() 
     business_class.number_of_seats_abrest              = 8
     business_class.number_of_rows                      = 4  
     business_class.seat_arm_rest_width                 = 4 *  Units.inches 
     business_class.seat_width                          = 17 *  Units.inches
     business_class.aile_width                          = 18  *  Units.inches  
     cabin.append_cabin_class(business_class) 
     
     economy_class = RCAIDE.Library.Components.Fuselages.Cabins.Classes.Economy() 
     economy_class.number_of_seats_abrest              = 9
     economy_class.number_of_rows                      = 26
     economy_class.galley_lavatory_percent_x_locations = [0.35, 1.0]       
     economy_class.type_A_exit_percent_x_locations     = [0.35, 1.0]
     cabin.append_cabin_class(economy_class)
     
     fuselage.append_cabin(cabin)        
     plot_layout_of_passenger_accommodations(fuselage, show_figure=False)
     fuselage_planform(fuselage)
     
     # Truth Values
     nose_length_truth   = 7.924799999999999
     tail_length_truth   = 9.905999999999999
     cabin_length_truth  = 41.732200000000006
     total_length_truth  = 59.563
     wetted_area_truth   = 831.939763080519
     frontal_area_truth  = 19.26755189268235
     dia_effective_truth = 4.952999999999999
     
     # Compute Errors
     error             = Data() 
     error.nose        = np.abs(fuselage.lengths.nose-nose_length_truth)/nose_length_truth
     error.tail        = np.abs(fuselage.lengths.tail-tail_length_truth)/tail_length_truth
     error.cabin       = np.abs(fuselage.lengths.cabin-cabin_length_truth)/cabin_length_truth
     error.total       = np.abs(fuselage.lengths.total-total_length_truth)/total_length_truth
     error.wetted_area = np.abs(fuselage.areas.wetted-wetted_area_truth)/wetted_area_truth
     error.front_area  = np.abs(fuselage.areas.front_projected-frontal_area_truth)/frontal_area_truth
     error.diameter    = np.abs(fuselage.effective_diameter-dia_effective_truth)/dia_effective_truth
             
     for k,v in list(error.items()):
          assert np.any(np.abs(v)<1e-6)
          
if __name__ == '__main__': 
     main()
     plt.show()