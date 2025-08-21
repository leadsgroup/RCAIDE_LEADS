# Regression/scripts/Tests/fuel_tank_volume.py
#
# 
# Created: Aug 2025, S. Shekar

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
from BWB         import vehicle_setup as BWB_vehicle_setup
from Boeing_737  import vehicle_setup as B737_vehicle_setup

# ----------------------------------------------------------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------------------------------------------------------

def main():
    integral_fuel_tank_volume_test()
    non_integral_fuel_tank_volume_test()
    return 
def integral_fuel_tank_volume_test():

    fuel_volume_true = [4.759146102553326,37.879246496514945]
    vehicle = B737_vehicle_setup()

    fuel_line = vehicle.networks.fuel.fuel_lines.fuel_line
    fuel_line.fuel_tanks.clear()
    #############################################################################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Main Wing Tanks
    #------------------------------------------------------------------------------------------------------------------------------------      
    vehicle.wings.main_wing.segments.root.has_fuel_tank   =  True
    vehicle.wings.main_wing.segments.yehudi.has_fuel_tank =  True
     
    wing_tank = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    wing_tank.fuel_selector_ratio  = 0.5
    wing_tank.fuel                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(wing_tank)
        
 
    plot_3d_vehicle(vehicle,
                    save_filename               = "Boeing_737_front_View", 
                    axis_limit                  = 100, 
                    top_view                    = False,  
                    front_view                  = True, 
                    show_figure=False)    
    
    error = (fuel_volume_true[0]- vehicle.total_fuel_volume)/fuel_volume_true[0]
    print(error)
    assert(abs(error)<1e-6)    
    vehicle.total_fuel_volume = 0 # Reset the fuel volume for regression and the next test
    ############################################################################################################################

    #(only for regression) delete wing segments and add tank flag to wing    
    vehicle.wings.main_wing.segments.clear()
    
    vehicle.fuselages.fuselage.segments.segment_13.has_fuel_tank                       = True
    vehicle.fuselages.fuselage.segments.segment_14.has_fuel_tank                       = True     
    
    #------------------------------------------------------------------------------------------------------------------------------------  
    # Main Wing Tanks
    #------------------------------------------------------------------------------------------------------------------------------------       
    refueling_tank_1                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    refueling_tank_1.fuel_selector_ratio  = 0.5
    refueling_tank_1.fuel                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(refueling_tank_1)
    
    
    refueling_tank_3                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.fuselages.fuselage) 
    refueling_tank_3.tag                  = 'refueling_tank_3'# for regression, aircraft has two tanks 
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


    error = (fuel_volume_true[1]- vehicle.total_fuel_volume)/fuel_volume_true[1]
    print(error)
    assert(abs(error)<1e-6)

    return


def non_integral_fuel_tank_volume_test():

    fuel_volume_true = 137.7973371553009
    
    vehicle = BWB_vehicle_setup()

    fuel_line = vehicle.networks.fuel.fuel_lines.fuel_line
    fuel_line.fuel_tanks.clear()
    #############################################################################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Main Wing Tanks
    #------------------------------------------------------------------------------------------------------------------------------------      
    vehicle.wings.main_wing.segments.fuel_wall.has_fuel_tank                 = True 
    vehicle.wings.main_wing.segments.fuel_wall.fuel_tank.percent_chord_start_location = 0.2  
    vehicle.wings.main_wing.segments.fuel_wall.fuel_tank.percent_chord_end_location   = 0.6   
     #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank_1                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(vehicle.wings.main_wing)
    fuel_tank_1.tag                             = 'H2_Fuel_Tank_1' 
    fuel_tank_1.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    fuel_tank_1.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    fuel_tank_1.material                      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_1.insulation_material           = RCAIDE.Library.Attributes.Materials.Vacuum_Gap_Multilayer_Insulation()
    fuel_tank_1.fuel.gravimetric_efficiency     = 0.5
    fuel_tank_1.wall_thickness                  = 2*Units.inches
    fuel_line.fuel_tanks.append(fuel_tank_1)

    fuel_tank_2                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(vehicle.wings.main_wing)
    fuel_tank_2.tag                             = 'H2_Fuel_Tank_2' 
    fuel_tank_2.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    fuel_tank_2.fuel.gravimetric_efficiency     = 0.5
    fuel_tank_2.wall_thickness                  = 2*Units.inches
    fuel_line.fuel_tanks.append(fuel_tank_2)

    fuel_tank_4                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(())
    fuel_tank_4.tag                           = 'H2_Fuel_Tank_4' 
    fuel_tank_4.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    fuel_tank_4.material                      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_4.insulation_material           = RCAIDE.Library.Attributes.Materials.Vacuum_Gap_Multilayer_Insulation()
    fuel_tank_4.fuel.gravimetric_efficiency   = 0.5
    fuel_tank_4.symmetric                     = False
    fuel_tank_4.orientation_euler_angles      = [0,0,np.pi/2]
    fuel_tank_4.bwb_aft_tank                  = True
    fuel_tank_4.aft_tank_start_root_chord     = 0.6
    fuel_tank_4.aft_tank_end_rood_chord       = 0.7
    fuel_tank_4.aft_tank_end_segment_tag      = 'cabin_wall' 
    fuel_tank_4.wing_root_tag                 = 'main_wing' 
    fuel_tank_4.radial_offset                 = 0.4

    fuel_line.fuel_tanks.append(fuel_tank_4)

    fuel_tank_5                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(())
    fuel_tank_5.tag                           = 'H2_Fuel_Tank_5' 
    fuel_tank_5.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    fuel_tank_5.fuel.gravimetric_efficiency   = 0.5
    fuel_tank_5.symmetric                     = False
    fuel_tank_5.orientation_euler_angles      = [0,0,np.pi/2]
    fuel_tank_5.bwb_aft_tank                  = True
    fuel_tank_5.aft_tank_start_root_chord     = 0.7
    fuel_tank_5.aft_tank_end_rood_chord       = 0.8
    fuel_tank_5.aft_tank_end_segment_tag      = 'cabin_wall' 
    fuel_tank_5.wing_root_tag                 = 'main_wing' 
    fuel_tank_5.radial_offset                 = 0.4

    fuel_line.fuel_tanks.append(fuel_tank_5)
        
 
    plot_3d_vehicle(vehicle,
                    save_filename               = "BWB", 
                    axis_limit                  = 100, 
                    top_view                    = False,  
                    front_view                  = True, 
                    show_figure=False)    
    
    error = (fuel_volume_true- vehicle.total_fuel_volume)/vehicle.total_fuel_volume
    print(error)
    
    assert(abs(error)<5e-2)
    return
 
if __name__ == '__main__': 
    main()    

        
