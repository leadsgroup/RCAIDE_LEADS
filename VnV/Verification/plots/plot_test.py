''' 
# plot_test.py
# 
# Created: Sep 25, M. Clarje

'''
#----------------------------------------------------------------------
#   Imports
# ---------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots  import *        
  
# python imports     
import numpy as np  
import sys
import matplotlib.pyplot as plt    
import os

# local imports 
base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Tiltrotor_EVTOL                import vehicle_setup as Tiltrotor_vehicle_setup   
from Tiltwing_EVTOL                 import vehicle_setup as Tiltwing_vehicle_setup  
from Embraer_190                    import vehicle_setup as E190_vehicle_setup 
from BWB                            import vehicle_setup as BWB_vehicle_setup
from ATR_72                         import vehicle_setup as ATR_72_vehicle_setup
from Concorde                       import vehicle_setup as Concorde_vehicle_setup
from Boeing_737                     import vehicle_setup as B737_vehicle_setup
from Hydrogen_Fuel_Cell_Twin_Otter  import vehicle_setup as HTO_vehicle_setup
from Navion                         import vehicle_setup as Navion_vehicle_setup

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    show_figure = False # must be false for C.I. on github
    evtol_aircraft_geometry_test(show_figure)
    conventional_turbofan_aircraft_geometry_test(show_figure)
    conventional_turboprop_aircraft_geometry_test(show_figure)
    conventional_turbojet_aircraft_geometry_test(show_figure)
    electric_rotor_aircraft_geometry_test(show_figure)
    general_aviation_aircraft_geometry_test(show_figure)
    bwb_aircraft_geometry_test(show_figure)
    orthogonal_view_test(show_figure)
    return 
    
def evtol_aircraft_geometry_test(show_figure):
    update_regression_values =  False
    TR_vehicle  = Tiltrotor_vehicle_setup(redesign_rotors=update_regression_values) 

    # plot vehicle 
    plot_3d_vehicle(TR_vehicle,  
                    wing_opacity                = 0.2,
                    save_filename               = "tilt_rotor", 
                    front_view                  = True, 
                    show_figure                 = show_figure 
                    )
    
    TW_vehicle  = Tiltwing_vehicle_setup(update_regression_values) 

    # plot vehicle 
    plot_3d_vehicle(TW_vehicle,   
                    wing_opacity                = 0.2,
                    save_filename               = "tilt_wing", 
                    front_view                  = True, 
                    show_figure                 = show_figure 
                    )
        
    return

def general_aviation_aircraft_geometry_test(show_figure):
    vehicle =  Navion_vehicle_setup() 

    fuel_line = vehicle.networks.fuel.fuel_lines.fuel_line
    fuel_line.fuel_tanks.clear()

    wing_tank = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.horizontal_stabilizer)  
    wing_tank.fuel                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(wing_tank)
    
    # plot vehicle 
    plot_3d_vehicle(vehicle,   
                    wing_opacity                = 0.2,
                    save_filename               = "navion", 
                    front_view                  = True, 
                    show_figure                 = show_figure 
                    )    
    return

def conventional_turbofan_aircraft_geometry_test(show_figure):     
    
    # conventional aircraft  
    vehicle                           = E190_vehicle_setup()
    vehicle.wings.main_wing.control_surfaces.flap.configuration_type = 'triple_slotted'  
    vehicle.wings.main_wing.high_lift = True 
    
    # plot aircraft
    plot_3d_vehicle(vehicle, 
                    save_filename               = "E_190",                     
                    show_figure=show_figure)
    return
def conventional_turboprop_aircraft_geometry_test(show_figure):
    

    # vehicle data
    vehicle  = ATR_72_vehicle_setup()
    fuel_line = vehicle.networks.fuel.fuel_lines.fuel_line
    fuel_line.fuel_tanks.clear()
    
    fuselage_tank = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.fuselages.fuselage)  
    fuselage_tank.fuel                          = RCAIDE.Library.Attributes.Propellants.Jet_A()
    fuselage_tank.segments_bounding_tank        = ['segment_11','segment_12' ] 
    fuel_line.fuel_tanks.append(fuselage_tank) 
     
    plot_3d_vehicle(vehicle,
                    save_filename  = "ATR_72", 
                    show_figure=show_figure)

    return

def orthogonal_view_test(show_figure):
    
    # -----------------------------------------
    # Multi-Point Mission Setup 
    # -----------------------------------------

    # vehicle data
    vehicle  = B737_vehicle_setup()

    # plot vehicle 
    plot_3d_vehicle(vehicle,  
                    show_figure                 = show_figure,
                    save_filename               = "Boeing_737", 
                    )

    # plot vehicle 
    plot_3d_vehicle(vehicle,
                    save_filename               = "Boeing_737_Top_View", 
                    top_view                    = True, 
                    side_view                   = False, 
                    front_view                  = False, 
                    show_figure=show_figure)


    plot_3d_vehicle(vehicle,
                    save_filename               = "Boeing_737_Side_View", 
                    top_view                    = False, 
                    side_view                   = True, 
                    front_view                  = False, 
                    show_figure=show_figure)


    plot_3d_vehicle(vehicle,
                    save_filename               = "Boeing_737_Front_View", 
                    top_view                    = False, 
                    side_view                   = False, 
                    front_view                  = True,
                    wing_opacity               = 0.2, 
                    show_figure=show_figure)
    
    return

def conventional_turbojet_aircraft_geometry_test(show_figure):

    # vehicle data
    vehicle  = Concorde_vehicle_setup() 

    # plot vehicle 
    plot_3d_vehicle(vehicle,  
                    wing_opacity          = 0.2,
                    front_view            = True, 
                    save_filename         = "Concorde", 
                    show_figure           = show_figure 
                    )    
    return

def electric_rotor_aircraft_geometry_test(show_figure): 

    vehicle  = HTO_vehicle_setup() 
     

    plot_3d_vehicle(vehicle,  
                    wing_opacity          = 0.2,
                    front_view            = True, 
                    save_filename         = "Hydrogen_Twin_Otter", 
                    show_figure           = show_figure 
                    )      
    
    return     
    
def bwb_aircraft_geometry_test(show_figure):
     
    vehicle  = BWB_vehicle_setup() 
    vehicle.wings.main_wing.cabins.side_cabin.tail.fineness_ratio  = 1      

    fuel_line = vehicle.networks.fuel.fuel_lines.fuel_line
    fuel_line.fuel_tanks.clear()
    #############################################################################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Main Wing Tanks
    #------------------------------------------------------------------------------------------------------------------------------------      
     #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank_1                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(vehicle.wings.main_wing)
    fuel_tank_1.tag                                    = 'H2_Fuel_Tank_1' 
    fuel_tank_1.fuel                                   = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()  
    fuel_tank_1.material                               = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_1.insulation_material                    = RCAIDE.Library.Attributes.Materials.Vacuum_Jacketed_Multilayer_Insulation()
    fuel_tank_1.fuel.gravimetric_efficiency            = 0.5 
    fuel_tank_1.segments_bounding_tank                 = ['fuel_wall', 'wing_section_2']  
    fuel_tank_1.segments_percent_chord_start           = [0.1,0.1] 
    fuel_tank_1.segments_percent_chord_end             = [0.55,0.55]
    fuel_tank_1.wall_thickness                         = 2*Units.inches
    fuel_line.fuel_tanks.append(fuel_tank_1)

    # -------------------------------------------------------------
    # Run test only if Python version >= 3.11
    # Shapely < 2.1 (and Python < 3.11) may not include functions
    # like 'maximum_inscribed_circle' required for this test.
    # -------------------------------------------------------------
    if sys.version_info >= (3, 11):

        fuel_tank_4                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(vehicle.wings.main_wing)
        fuel_tank_4.tag                           = 'H2_Fuel_Tank_4' 
        fuel_tank_4.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
        fuel_tank_4.material                      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
        fuel_tank_4.insulation_material           = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
        fuel_tank_4.fuel.gravimetric_efficiency   = 0.5
        fuel_tank_4.xz_plane_symmetric            = False
        fuel_tank_4.orientation_euler_angles      = [0,0,np.pi/2]
        fuel_tank_4.bwb_aft_tank                  = True
        fuel_tank_4.aft_tank_root_chord_bounds    = [0.65,0.9]
        fuel_tank_4.aft_tank_segment_bound        = 'cabin_wall'
        fuel_tank_4.radial_offset                 = 0.2
        
        fuel_line.fuel_tanks.append(fuel_tank_4)

        fuel_tank   = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(vehicle.wings.main_wing)
        fuel_tank.tag = 'wing_tanks'
        fuel_tank.geometry_type     = 'conformal'
        fuel_tank.fuel  = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
        fuel_tank.material              = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
        fuel_tank.insulation_material   = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
        fuel_tank.segments_bounding_tank    = ['fuel_wall', 'wing_section_1']  
        fuel_tank.segments_percent_chord_start  = [0.2,0.2] 
        fuel_tank.segments_percent_chord_end    = [0.6,0.6]  
        fuel_line.fuel_tanks.append(fuel_tank) 

        fuel_tank_2                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(vehicle.wings.main_wing)
        fuel_tank_2.tag                                    = 'aft_tank' 
        fuel_tank_2.geometry_type                          = 'conformal'
        fuel_tank_2.material                               = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
        fuel_tank_2.insulation_material                    = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
        fuel_tank_2.xz_plane_symmetric                     = False
        fuel_tank_2.orientation_euler_angles               = [0,0,np.pi/2]
        fuel_tank_2.bwb_aft_tank                           = True
        fuel_tank_2.aft_tank_root_chord_bounds             = [0.7,0.8]
        fuel_tank_2.aft_tank_segment_bound                 = 'cabin_wall'
        fuel_tank_2.radial_offset                          = 0.1
        fuel_tank_2.fuel.tag                               = '_lh2' 
        fuel_line.fuel_tanks.append(fuel_tank_2)    
   
    plot_3d_vehicle(vehicle, 
                    save_filename = "BWB_Additional_Tanks",  
                    show_figure=show_figure)

    return    
     
if __name__ == '__main__': 
    main()    
    plt.show()