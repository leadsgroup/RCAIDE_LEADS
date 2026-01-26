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
from Navion      import vehicle_setup as Nav_vehicle_setup

# ----------------------------------------------------------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------------------------------------------------------

def main():
    integral_fuel_tank_volume_test()
    single_wing_segment_integral_fuel_tank_volume_test()
    # -------------------------------------------------------------
    # Run test only if Python version >= 3.11
    # Shapely < 2.1 (and Python < 3.11) may not include functions
    # like 'maximum_inscribed_circle' required for this test.
    # -------------------------------------------------------------
    if sys.version_info >= (3, 11):
        non_integral_fuel_tank_volume_test()
    else:
        print("Skipping non_integral_fuel_tank_volume_test(): Shapely lacks 'maximum_inscribed_circle' support for Python < 3.11.")
    return

def single_wing_segment_integral_fuel_tank_volume_test():

    fuel_volume_true = [0.16044005289435948]

    vehicle = Nav_vehicle_setup()
    
    fuel_line = vehicle.networks.fuel.fuel_lines.fuel_line
    fuel_line.fuel_tanks.clear()

    wing_tank = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.horizontal_stabilizer)  
    wing_tank.fuel                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(wing_tank)
    wing_tank = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.vertical_stabilizer)  
    wing_tank.fuel                 = RCAIDE.Library.Attributes.Propellants.Jet_A() 
    fuel_line.fuel_tanks.append(wing_tank)
    
    configs = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    for analysis in analyses:
        analysis.geometry.settings.compute_fuel_volume = True
    mission = mission_setup(analyses)
    geometry(mission)

    error = (fuel_volume_true[0]- mission.segments.cruise.analyses.vehicle.volume_properties.fuel)/fuel_volume_true[0]
    print(error)
    assert(abs(error)<1e-6)    
    return


def integral_fuel_tank_volume_test():

    fuel_volume_true = [19.53629338802644,77.88321682397498]
    vehicle          = B737_vehicle_setup() 
    fuel_line        = vehicle.networks.fuel.fuel_lines.fuel_line
    fuel_line.fuel_tanks.clear()
    
    #############################################################################################################################    
    #------------------------------------------------------------------------------------------------------------------------------------  
    #  Main Wing Tanks
    #------------------------------------------------------------------------------------------------------------------------------------       
    wing_tank_1                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    wing_tank_1.fuel_selector_ratio          = 0.5
    wing_tank_1.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    wing_tank_1.segments_bounding_tank       = ['root', 'yehudi']  
    wing_tank_1.segments_percent_chord_start = [0.1, 0.1]
    wing_tank_1.segments_percent_chord_end   = [0.7, 0.7] 
    fuel_line.fuel_tanks.append(wing_tank_1)
    
    wing_tank_2                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)  
    wing_tank_2.fuel_selector_ratio          = 0.5
    wing_tank_2.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    wing_tank_2.segments_bounding_tank       = ['yehudi', 'section_2']  
    wing_tank_2.segments_percent_chord_start = [0.1, 0.1]
    wing_tank_2.segments_percent_chord_end   = [0.7, 0.7] 
    fuel_line.fuel_tanks.append(wing_tank_2)    

    configs  = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    for analysis in analyses:
        analysis.geometry.settings.compute_fuel_volume = True
    mission  = mission_setup(analyses)
    geometry(mission)

    error = (fuel_volume_true[0]- mission.segments.cruise.analyses.vehicle.volume_properties.fuel)/fuel_volume_true[0]
    print(error)
    assert(abs(error)<1e-6)    

    fus_tank_1 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.fuselages.fuselage)  
    fus_tank_1.fuel_selector_ratio     = 0.5
    fus_tank_1.fuel                    = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    fus_tank_1.segments_bounding_tank  = ['segment_5','segment_6'] 
    fuel_line.fuel_tanks.append(fus_tank_1)
    
    fus_tank_2 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.fuselages.fuselage)  
    fus_tank_2.fuel_selector_ratio     = 0.5
    fus_tank_2.fuel                    = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    fus_tank_2.segments_bounding_tank  = ['segment_6','segment_7']  
    fuel_line.fuel_tanks.append(fus_tank_2)
    
    fus_tank_3 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.fuselages.fuselage)  
    fus_tank_3.fuel_selector_ratio     = 0.5
    fus_tank_3.fuel                    = RCAIDE.Library.Attributes.Propellants.Jet_A()  
    fus_tank_3.segments_bounding_tank  = ['segment_7','segment_8']  
    fuel_line.fuel_tanks.append(fus_tank_3) 

    configs  = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    for analysis in analyses:
        analysis.geometry.settings.compute_fuel_volume = True
    mission  = mission_setup(analyses)
    geometry(mission)

    error = (fuel_volume_true[1]- mission.segments.cruise.analyses.vehicle.volume_properties.fuel)/fuel_volume_true[0]
    print(error)
    assert(abs(error)<1e-6)    

    return


def non_integral_fuel_tank_volume_test():

    fuel_volume_true = 566.6589439371462
    vehicle          = BWB_vehicle_setup() 
    fuel_line        = vehicle.networks.fuel.fuel_lines.fuel_line
    fuel_line.fuel_tanks.clear()
    
    #############################################################################################################################    
     #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank_1                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(vehicle.wings.main_wing)
    fuel_tank_1.tag                             = 'H2_Fuel_Tank_1' 
    fuel_tank_1.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()  
    fuel_tank_1.material                        = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_1.insulation_material             = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_1.fuel.gravimetric_efficiency     = 0.5 
    fuel_tank_1.segments_bounding_tank          = ['fuselage_section_3', 'wing_section_2']        
    fuel_tank_1.segments_percent_chord_start    = [0.2,0.2]
    fuel_tank_1.segments_percent_chord_end      = [0.6,0.6]  
    fuel_tank_1.wall_thickness                  = 2*Units.inches
    fuel_line.fuel_tanks.append(fuel_tank_1)
     
     # fuel tank
    fuel_tank_2                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank()
    fuel_tank_2.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()  
    fuel_tank_2.tag                             = 'H2_Fuel_Tank_2' 
    fuel_tank_2.geometry_type                   = 'prismatic'
    fuel_tank_2.lengths.external                = 1
    fuel_tank_2.widths.external                 = 1
    fuel_tank_2.heights.external                = 1
    fuel_tank_2.segments_bounding_tank          = ['fuselage_section_3', 'wing_section_2']      
    fuel_tank_2.segments_percent_chord_start    = [0.2 ,0.2]
    fuel_tank_2.segments_percent_chord_end      = [0.6,0.6]  
    fuel_tank_2.wall_thickness                  = 2*Units.inches
    fuel_line.fuel_tanks.append(fuel_tank_2)

    fuel_tank_2a                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank()
    fuel_tank_2a.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()  
    fuel_tank_2a.tag                             = 'H2_Fuel_Tank_2a' 
    fuel_tank_2a.geometry_type                   = 'prismatic'
    fuel_tank_2a.lengths.external                = 1
    fuel_tank_2a.widths.external                 = 1
    fuel_tank_2a.heights.external                = 1
    fuel_tank_2a.wall_thickness                  = 2*Units.inches
    fuel_tank_2a.fuel.mass_properties.mass       = 0.1
    fuel_line.fuel_tanks.append(fuel_tank_2a)


    fuel_tank_3                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(vehicle.wings.main_wing)
    fuel_tank_3.tag                             = 'H2_Fuel_Tank_3' 
    fuel_tank_3.fuel                            = RCAIDE.Library.Attributes.Propellants.Jet_A1()   
    fuel_tank_3.material                        = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_3.insulation_material             = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_3.fuel.gravimetric_efficiency     = 0.5
    fuel_tank_3.wall_thickness                  = 2*Units.inches
    fuel_tank_3.segments_bounding_tank          = ['fuselage_section_3', 'wing_section_2']       
    fuel_tank_3.segments_percent_chord_start    = [0.2 ,0.2]
    fuel_tank_3.segments_percent_chord_end      = [0.6,0.6]  
    fuel_line.fuel_tanks.append(fuel_tank_3)


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
    fuel_tank_4.aft_tank_segment_bound        = 'fuel_wall'
    fuel_tank_4.radial_offset                 = 0.2

    fuel_line.fuel_tanks.append(fuel_tank_4)

    fuel_tank_4a                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(vehicle.wings.main_wing)
    fuel_tank_4a.tag                           = 'H2_Fuel_Tank_4a' 
    fuel_tank_4a.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    fuel_tank_4a.material                      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_4a.insulation_material           = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_4a.fuel.gravimetric_efficiency   = 0.5
    fuel_tank_4a.xz_plane_symmetric            = False
    fuel_tank_4a.orientation_euler_angles      = [0,0,np.pi/2]
    fuel_tank_4a.bwb_aft_tank                  = True
    fuel_tank_4a.aft_tank_root_chord_bounds    = [0.65,0.9]
    fuel_tank_4a.aft_tank_segment_bound        = 'cabin_wall'
    fuel_tank_4a.radial_offset                 = 0.2
    fuel_line.fuel_tanks.append(fuel_tank_4a)

    fuel_tank_5                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank(vehicle.wings.main_wing)
    fuel_tank_5.tag                           = 'H2_Fuel_Tank_5' 
    fuel_tank_5.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()   
    fuel_tank_5.material                      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_5.insulation_material           = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_5.fuel.gravimetric_efficiency   = 0.5
    fuel_tank_5.xz_plane_symmetric            = False
    fuel_tank_5.orientation_euler_angles      = [0,0,np.pi/2]
    fuel_tank_5.bwb_aft_tank                  = True
    fuel_tank_5.aft_tank_root_chord_bounds    = [0.65,0.9] 
    fuel_tank_5.aft_tank_segment_bound        = 'cabin_wall'
    fuel_tank_5.radial_offset                 = 0.5
    fuel_tank_5.fuel.mass_properties.mass     = 0.1

    fuel_line.fuel_tanks.append(fuel_tank_5)

    # fuel tank
    fuel_tank_6                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(vehicle.wings.main_wing)
    fuel_tank_6.tag                             = 'H2_Fuel_Tank_6' 
    fuel_tank_6.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()  
    fuel_tank_6.material                        = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_6.insulation_material             = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_6.fuel.gravimetric_efficiency     = 0.5
    fuel_tank_6.wall_thickness                  = 2*Units.inches
    fuel_tank_6.fuel.mass_properties.mass       = 0.1
    fuel_tank_6.segments_bounding_tank          = ['fuel_wall', 'wing_section_2']   
    fuel_line.fuel_tanks.append(fuel_tank_6)
  
    configs  = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    for analysis in analyses:
        analysis.geometry.settings.compute_fuel_volume = True
        analysis.geometry.settings.update_max_fuel = True
    mission  = mission_setup(analyses)

    geometry(mission)   
    
    error = (fuel_volume_true- mission.segments.cruise.analyses.vehicle.volume_properties.fuel)/fuel_volume_true
    
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
    analyses.vehicle = vehicle

    # ------------------------------------------------------------------
    #  Geometry
    # ------------------------------------------------------------------
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry() 
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
    base_segment.state.numerics.number_of_control_points = 16
  
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

        
