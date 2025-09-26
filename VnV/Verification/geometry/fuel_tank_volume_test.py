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
from RCAIDE.Library.Mission.Common.Pre_Process import geometry
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

    fuel_volume_true = [19.363771803419546,38.2338699410805]
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

    configs = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    mission = mission_setup(analyses)
    geometry(mission)

    error = (fuel_volume_true[0]- mission.segments.cruise.analyses.geometry.vehicle.volume_properties.fuel)/fuel_volume_true[0]
    print(error)
    assert(abs(error)<1e-6)    
   
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

    refueling_tank_1a                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    refueling_tank_1a.fuel_selector_ratio  = 0.5
    refueling_tank_1a.fuel                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    refueling_tank_1a.fuel.mass_properties.mass = 0.1
    fuel_line.fuel_tanks.append(refueling_tank_1a)
    
    refueling_tank_2                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.fuselages.fuselage) 
    refueling_tank_2.tag                  = 'refueling_tank_2'# for regression, aircraft has two tanks 
    refueling_tank_2.fuel_selector_ratio  = 0
    refueling_tank_2.fuel                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    refueling_tank_2.fuel.mass_properties.mass = 0.1
    fuel_line.fuel_tanks.append(refueling_tank_2)     
 
    refueling_tank_2a                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing) 
    refueling_tank_2a.tag                  = 'refueling_tank_2a'# for regression, aircraft has two tanks 
    refueling_tank_2a.fuel_selector_ratio  = 0
    refueling_tank_2a.fuel                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    refueling_tank_2a.fuel.mass_properties.mass = 0.1
    fuel_line.fuel_tanks.append(refueling_tank_2a)     
 
    
    refueling_tank_3                      = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.fuselages.fuselage) 
    refueling_tank_3.tag                  = 'refueling_tank_3'# for regression, aircraft has two tanks 
    refueling_tank_3.fuel_selector_ratio  = 0
    refueling_tank_3.fuel                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(refueling_tank_3)     

    configs = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    mission = mission_setup(analyses)
    geometry(mission)   



    error = (fuel_volume_true[1]- mission.segments.cruise.analyses.geometry.vehicle.volume_properties.fuel)/fuel_volume_true[1]
    print(error)
    assert(abs(error)<1e-6)

    return


def non_integral_fuel_tank_volume_test():

    fuel_volume_true = 356.6486933842599
    
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
    fuel_tank_1.material                        = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_1.insulation_material             = RCAIDE.Library.Attributes.Materials.Vacuum_Gap_Multilayer_Insulation()
    fuel_tank_1.fuel.gravimetric_efficiency     = 0.5
    fuel_tank_1.wall_thickness                  = 2*Units.inches
    fuel_line.fuel_tanks.append(fuel_tank_1)
     # fuel tank
    fuel_tank_2                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank()
    fuel_tank_2.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()  
    fuel_tank_2.tag                             = 'H2_Fuel_Tank_2' 
    fuel_tank_2.geometry_type                   = 'prismatic'
    fuel_tank_2.outer_length                    = 1
    fuel_tank_2.outer_width                     = 1
    fuel_tank_2.outer_height                    = 1
    fuel_tank_2.wall_thickness                  = 2*Units.inches
    fuel_line.fuel_tanks.append(fuel_tank_2)

    fuel_tank_2a                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank()
    fuel_tank_2a.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()  
    fuel_tank_2a.tag                             = 'H2_Fuel_Tank_2a' 
    fuel_tank_2a.geometry_type                   = 'prismatic'
    fuel_tank_2a.outer_length                    = 1
    fuel_tank_2a.outer_width                     = 1
    fuel_tank_2a.outer_height                    = 1
    fuel_tank_2a.wall_thickness                  = 2*Units.inches
    fuel_tank_2a.fuel.mass_properties.mass       = 0.1
    fuel_line.fuel_tanks.append(fuel_tank_2a)


    fuel_tank_3                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(vehicle.wings.main_wing)
    fuel_tank_3.tag                             = 'H2_Fuel_Tank_3' 
    fuel_tank_3.fuel                            = RCAIDE.Library.Attributes.Propellants.Jet_A1()   
    fuel_tank_3.material                        = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_3.insulation_material             = RCAIDE.Library.Attributes.Materials.Vacuum_Gap_Multilayer_Insulation()
    fuel_tank_3.fuel.gravimetric_efficiency     = 0.5
    fuel_tank_3.wall_thickness                  = 2*Units.inches
    fuel_line.fuel_tanks.append(fuel_tank_3)


    fuel_tank_4                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(())
    fuel_tank_4.tag                           = 'H2_Fuel_Tank_4' 
    fuel_tank_4.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    fuel_tank_4.material                      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_4.insulation_material           = RCAIDE.Library.Attributes.Materials.Vacuum_Gap_Multilayer_Insulation()
    fuel_tank_4.fuel.gravimetric_efficiency   = 0.5
    fuel_tank_4.xz_plane_symmetric            = False
    fuel_tank_4.orientation_euler_angles      = [0,0,np.pi/2]
    fuel_tank_4.bwb_aft_tank                  = True
    fuel_tank_4.aft_tank_start_root_chord     = 0.6
    fuel_tank_4.aft_tank_end_rood_chord       = 0.8
    fuel_tank_4.aft_tank_end_segment_tag      = 'cabin_wall' 
    fuel_tank_4.wing_root_tag                 = 'main_wing' 
    fuel_tank_4.radial_offset                 = 0.2

    fuel_line.fuel_tanks.append(fuel_tank_4)

    fuel_tank_4a                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(())
    fuel_tank_4a.tag                           = 'H2_Fuel_Tank_4a' 
    fuel_tank_4a.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    fuel_tank_4a.material                      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_4a.insulation_material           = RCAIDE.Library.Attributes.Materials.Vacuum_Gap_Multilayer_Insulation()
    fuel_tank_4a.fuel.gravimetric_efficiency   = 0.5
    fuel_tank_4a.xz_plane_symmetric            = False
    fuel_tank_4a.orientation_euler_angles      = [0,0,np.pi/2]
    fuel_tank_4a.bwb_aft_tank                  = True
    fuel_tank_4a.aft_tank_start_root_chord     = 0.6
    fuel_tank_4a.aft_tank_end_rood_chord       = 0.8
    fuel_tank_4a.aft_tank_end_segment_tag      = 'cabin_wall' 
    fuel_tank_4a.wing_root_tag                 = 'main_wing' 
    fuel_tank_4a.radial_offset                 = 0.2
    fuel_line.fuel_tanks.append(fuel_tank_4a)

    fuel_tank_5                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(())
    fuel_tank_5.tag                           = 'H2_Fuel_Tank_5' 
    fuel_tank_5.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    fuel_tank_5.material                      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_5.insulation_material           = RCAIDE.Library.Attributes.Materials.Vacuum_Gap_Multilayer_Insulation()
    fuel_tank_5.fuel.gravimetric_efficiency   = 0.5
    fuel_tank_5.xz_plane_symmetric            = False
    fuel_tank_5.orientation_euler_angles      = [0,0,np.pi/2]
    fuel_tank_5.bwb_aft_tank                  = True
    fuel_tank_5.aft_tank_start_root_chord     = 0.4
    fuel_tank_5.aft_tank_end_rood_chord       = 0.6
    fuel_tank_5.aft_tank_end_segment_tag      = 'cabin_wall' 
    fuel_tank_5.wing_root_tag                 = 'main_wing' 
    fuel_tank_5.radial_offset                 = 0.5
    fuel_tank_5.fuel.mass_properties.mass     = 0.1

    fuel_line.fuel_tanks.append(fuel_tank_5)

    # fuel tank
    fuel_tank_6                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(vehicle.wings.main_wing)
    fuel_tank_6.tag                             = 'H2_Fuel_Tank_6' 
    fuel_tank_6.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()  
    fuel_tank_6.material                        = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_6.insulation_material             = RCAIDE.Library.Attributes.Materials.Vacuum_Gap_Multilayer_Insulation()
    fuel_tank_6.fuel.gravimetric_efficiency     = 0.5
    fuel_tank_6.wall_thickness                  = 2*Units.inches
    fuel_tank_6.fuel.mass_properties.mass       = 0.1
    fuel_line.fuel_tanks.append(fuel_tank_6)
 
    
    configs = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    mission = mission_setup(analyses)
    geometry(mission)   
    
    error = (fuel_volume_true- mission.segments.cruise.analyses.geometry.vehicle.volume_properties.fuel)/fuel_volume_true
    
    assert(abs(error)<5e-2) 

    return

def configs_setup(vehicle):
    """This function sets up vehicle configurations for use in different parts of the mission.
    Here, this is mostly in terms of high lift settings."""

    # ------------------------------------------------------------------
    #   Initialize Configurations
    # ------------------------------------------------------------------

    configs     = RCAIDE.Library.Components.Configs.Config.Container() 
    base_config = RCAIDE.Library.Components.Configs.Config(vehicle)
    base_config.tag = 'base' 
    configs.append(base_config)
    return configs

def analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):
    """This is the baseline set of analyses to be used with this vehicle. Of these, the most
    commonly changed are the weights and aerodynamics methods."""

    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = RCAIDE.Framework.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.vehicle = vehicle
    geometry.settings.update_fuel_volume = True
    analyses.append(geometry)
    return analyses    

def mission_setup(analyses):

    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------
    
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments 
    base_segment = Segments.Segment() 
    base_segment.state.numerics.number_of_control_points = 10 
  
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise" 
    segment.analyses.extend( analyses.base ) 
    segment.altitude                                                 = 35000 * Units['ft']  
    segment.air_speed                                                = 450 * Units['knots']
    segment.distance                                                 = 7370 * Units.km   
    mission.append_segment(segment)

    return mission    

if __name__ == '__main__': 
    main()    

        
