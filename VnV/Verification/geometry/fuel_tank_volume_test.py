# Regression/scripts/Tests/emissions_test.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Core                          import Units , Data 
from RCAIDE.Library.Plots                           import *        

# python imports     
import numpy as np  
import sys
import os
import matplotlib.pyplot as plt  


sys.path.append(os.path.join( os.path.split(os.path.split(sys.path[0])[0])[0], 'Vehicles'))
from Boeing_737    import vehicle_setup, configs_setup 

# ----------------------------------------------------------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------------------------------------------------------

def main():
    
    # delete wing segments and add tank flag to wing
    vehicle = vehicle_setup()
    vehicle.wings.main_wing.segments.clear()
    

    vehicle.fuselages.fuselage.segments.segment_13.has_fuel_tank                       = True
    vehicle.fuselages.fuselage.segments.segment_14.has_fuel_tank                       = True     
    fuel_line = vehicle.networks.fuel.fuel_lines.fuel_line
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Main Wing Tanks
    #------------------------------------------------------------------------------------------------------------------------------------       
    refueling_tank_1 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    refueling_tank_1.fuel_selector_ratio  = 0.5
    refueling_tank_1.fuel                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(refueling_tank_1)
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Refuling Tanks
    #------------------------------------------------------------------------------------------------------------------------------------       
    refueling_tank_2 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(vehicle.fuselages.fuselage)
    refueling_tank_2.tag = 'refueling_tank_2'# for regression, aircraft has two tanks
    refueling_tank_2.fuel_selector_ratio  = 0
    refueling_tank_2.outer_diameter       = 2
    refueling_tank_2.wall_clearance       = 1 * Units.inches
    refueling_tank_2.wall_thickness       = 2 * Units.inches
    refueling_tank_2.length               = 5
    refueling_tank_2.origin               = [[18, 0, 0.5]] 
    refueling_tank_2.fuel                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(refueling_tank_2) 
    
    refueling_tank_3 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.fuselages.fuselage) 
    refueling_tank_3.tag = 'refueling_tank_3'# for regression, aircraft has two tanks 
    refueling_tank_3.fuel_selector_ratio  = 0
    refueling_tank_3.fuel                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(refueling_tank_3)        

    plot_3d_vehicle(vehicle,
                    save_filename               = "Boeing_737_Side_View", 
                    axis_limit                  = 100, 
                    top_view                    = False, 
                    side_view                   = True, 
                    front_view                  = False, 
                    show_figure=False)    
    
    return 

 
if __name__ == '__main__': 
    main()    
    plt.show()
        
