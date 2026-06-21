# Regression/scripts/Tests/analysis_weights/cg_and_moi_test.py
# 
# Created:  Oct 2024, A. Molloy
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# cg_and_moi_test.py

from RCAIDE.Framework.Core                                     import Units
from RCAIDE.Library.Methods.Geometry.Planform                  import wing_planform
from RCAIDE.Library.Mission.Common.Pre_Process                 import geometry, mass_properties
import numpy as  np
import RCAIDE
import sys
import os

base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)

# the analysis functions
from Lockheed_C5a           import vehicle_setup as transport_setup
from Cessna_172             import vehicle_setup as general_aviation_setup
from Stopped_Rotor_EVTOL    import vehicle_setup as EVTOL_setup
from BWB                    import vehicle_setup as BWB_vehicle_setup
def main(): 
    # make true only when resizing aircraft. should be left false for regression
    update_regression_values = False  
    Transport_Aircraft_Test()
    General_Aviation_Test()
    EVTOL_Aircraft_Test(update_regression_values)
    # -------------------------------------------------------------
    # Run test only if Python version >= 3.11
    # Shapely < 2.1 (and Python < 3.11) may not include functions
    # like 'maximum_inscribed_circle' required for this test.
    # -------------------------------------------------------------
    if sys.version_info >= (3, 11):
        BWB_Test()
    else:
        print("Skipping BWB_Test():\
            Shapely lacks 'maximum_inscribed_circle' support for Python < 3.11.")
    return

def BWB_Test():

    vehicle          = BWB_vehicle_setup() 
    fuel_line        = vehicle.networks.fuel.fuel_lines.fuel_line
    fuel_line.fuel_tanks.clear()
    
    #############################################################################################################################    
     #------------------------------------------------------------------------------------------------------------------------- 
    #  Energy Source: Fuel Tank
    #------------------------------------------------------------------------------------------------------------------------- 
    # fuel tank
    fuel_tank_1                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank(vehicle.wings.main_wing)
    fuel_tank_1.tag                             = 'LNG_Fuel_Tank_1'
    fuel_tank_1.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()
    fuel_tank_1.design_inlet_temperature        = 100
    fuel_tank_1.inner_structure.material        = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_1.insulation.material             = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_1.gravimetric_efficiency          = 0.5
    fuel_tank_1.segments_bounding_tank          = ['fuselage_section_3', 'wing_section_2']        
    fuel_tank_1.segments_percent_chord_start    = [0.2,0.2]
    fuel_tank_1.segments_percent_chord_end      = [0.6,0.6]  
    fuel_tank_1.wall_thickness                  = 2*Units.inches
    fuel_line.fuel_tanks.append(fuel_tank_1)


    fuel_tank_2                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank(vehicle.wings.main_wing)
    fuel_tank_2.tag                           = 'LNG_Fuel_Tank_2'
    fuel_tank_2.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()
    fuel_tank_2.design_inlet_temperature      = 100
    fuel_tank_2.inner_structure.material      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_2.insulation.material           = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_2.gravimetric_efficiency        = 0.5
    fuel_tank_2.xz_plane_symmetric            = False
    fuel_tank_2.orientation_euler_angles      = [0,0,np.pi/2]
    fuel_tank_2.transverse_tank               = True
    fuel_tank_2.transverse_tank_chord_bounds  = [0.65,0.9]
    fuel_tank_2.transverse_tank_segment_bound = 'fuel_wall'
    fuel_tank_2.radial_offset                 = 0.2

    fuel_line.fuel_tanks.append(fuel_tank_2)
   
    # define configs, analyses, and mission for the BWB test
    configs  = configs_setup(vehicle)
    analyses = BWB_analyses_setup(configs) 
    mission  = mission_setup(analyses) 
    
    # run geometry and mass properties analyses to get CG and MOI results for the BWB test
    geometry(mission)   
    mass_properties(mission)

    mission_vehicle = mission.segments[0].analyses.vehicle
    computed_moi    = mission_vehicle.mass_properties.moments_of_inertia.tensor

    print("\n\nBWB Aircraft Test Results:")
    print('BWB  OEW CG: ' + str(mission_vehicle.mass_properties.operating_empty_center_of_gravity))
    print('BWB  OEW CG Mass Percentage: ' + str(mission_vehicle.mass_properties.OEW_CG_mass_percentage) + ' %')
    print('BWB  Moment of Inertia')
    print(computed_moi)
    truth_OEW_CG_mass_percentage = 99.34
    truth_moi = np.array([[ 3718367.96918734,  1217750.86758788,  -863024.23812853],
                          [ 1217750.86758788, 13844895.97177752,    52136.11018868],
                          [ -863024.23812853,    52136.11018868, 16390186.23977657]])

    error_moi = abs((computed_moi - truth_moi) / truth_moi)
    assert np.all(error_moi < 1e-2),\
        f"MOI tensor mismatch.\nExpected:\n{truth_moi}\nGot:\n{computed_moi}"

    error_pct = abs(mission_vehicle.mass_properties.OEW_CG_mass_percentage - truth_OEW_CG_mass_percentage)
    assert error_pct < 0.1,\
        f"OEW CG mass percentage mismatch. Expected: {truth_OEW_CG_mass_percentage}%, Got: {mission_vehicle.mass_properties.OEW_CG_mass_percentage}%"

    print("****************************************************************")
    return

def Transport_Aircraft_Test():
    vehicle = transport_setup() 

    # define configs, analyses, and mission for the BWB test
    configs  = configs_setup(vehicle)
    analyses = Transport_analyses_setup(configs) 
    mission  = mission_setup(analyses) 
    
    # run geometry and mass properties analyses to get CG and MOI results
    geometry(mission)
    mass_properties(mission)

    mission_vehicle = mission.segments[0].analyses.vehicle
    MOI = mission_vehicle.mass_properties.moments_of_inertia.tensor

    print("\n\n Transport Aircraft Test Results:")
    print('Transport OEW CG: ' + str(mission_vehicle.mass_properties.operating_empty_center_of_gravity))
    print('Transport OEW CG Mass Percentage: ' + str(mission_vehicle.mass_properties.OEW_CG_mass_percentage) + ' %')
    print('Transport Moment of Inertia')
    print(MOI)

    truth_OEW_CG_mass_percentage = 111.88
    truth_moi  = np.array([[ 7317066.68118963,  -102481.15213347,    92929.8510052 ],
                           [ -102481.15213347, 34562824.23910017,   210365.5831478 ],
                           [   92929.8510052 ,   210365.5831478 , 37397106.90877086]])

    error_moi = abs((MOI - truth_moi) / np.where(truth_moi != 0, truth_moi, 1))
    assert np.all(error_moi < 1e-6),\
        f"MOI tensor mismatch.\nExpected:\n{truth_moi}\nGot:\n{MOI}"

    error_pct = abs(mission_vehicle.mass_properties.OEW_CG_mass_percentage - truth_OEW_CG_mass_percentage)
    assert error_pct < 0.1,\
        f"OEW CG mass percentage mismatch. Expected: {truth_OEW_CG_mass_percentage}%, Got: {mission_vehicle.mass_properties.OEW_CG_mass_percentage}%"

    print("****************************************************************")

    return

def General_Aviation_Test():
    vehicle = general_aviation_setup()

    # define configs, analyses, and mission
    configs  = configs_setup(vehicle)
    analyses = GA_analyses_setup(configs) 
    mission  = mission_setup(analyses) 
    
    # run geometry and mass properties analyses to get CG and MOI results
    geometry(mission)
    mass_properties(mission)

    mission_vehicle = mission.segments[0].analyses.vehicle
    MOI = mission_vehicle.mass_properties.moments_of_inertia.tensor

    print("\n\nGeneral Aviation Test Results:")
    print('GA OEW CG: ' + str(mission_vehicle.mass_properties.operating_empty_center_of_gravity))
    print('GA OEW CG Mass Percentage: ' + str(mission_vehicle.mass_properties.OEW_CG_mass_percentage) + ' %')
    print('GA Moment of Inertia')
    print(MOI)

    truth_OEW_CG_mass_percentage = 98.78
    truth_moi  = np.array([[ 3.74934496e+02, -6.95403814e-15, -4.00441102e+01],
                           [-6.95403814e-15,  3.75654482e+03, -6.13259031e-15],
                           [-4.00441102e+01, -6.13259031e-15,  3.58247327e+03]])

    error_moi = abs(MOI - truth_moi)
    assert np.all(error_moi < 1e-5),\
        f"MOI tensor mismatch.\nExpected:\n{truth_moi}\nGot:\n{MOI}"

    error_pct = abs(mission_vehicle.mass_properties.OEW_CG_mass_percentage - truth_OEW_CG_mass_percentage)
    assert error_pct < 0.1,\
        f"OEW CG mass percentage mismatch. Expected: {truth_OEW_CG_mass_percentage}%, Got: {mission_vehicle.mass_properties.OEW_CG_mass_percentage}%"

    print("****************************************************************")
    return

def EVTOL_Aircraft_Test(update_regression_values):
    vehicle = EVTOL_setup(update_regression_values)
    configs  = configs_setup(vehicle)
    analyses = EVTOL_analyses_setup(configs)
    mission  = mission_setup(analyses) 
    
    # run geometry and mass properties analyses to get CG and MOI results
    geometry(mission)
    mass_properties(mission)

    mission_vehicle = mission.segments[0].analyses.vehicle
    MOI = mission_vehicle.mass_properties.moments_of_inertia.tensor

    print("\n\nEVTOL Aircraft Test Results:")
    print('EVTOL OEW CG: ' + str(mission_vehicle.mass_properties.operating_empty_center_of_gravity))
    print('EVTOL OEW CG Mass Percentage: ' + str(mission_vehicle.mass_properties.OEW_CG_mass_percentage) + ' %')
    print('EVTOL Moment of Inertia')
    print(MOI)

    truth_OEW_CG_mass_percentage = 86.35
    truth_moi  = np.array([[ 9365.33138772,  -450.90746732,  -440.58026539],
                           [ -450.90746732,  9156.50731552,   -99.75208906],
                           [ -440.58026539,   -99.75208906, 16759.72413936]])

    error_moi = abs((MOI - truth_moi) / truth_moi)
    assert np.all(error_moi < 5e-2),\
        f"MOI tensor mismatch.\nExpected:\n{truth_moi}\nGot:\n{MOI}"

    error_pct = abs(mission_vehicle.mass_properties.OEW_CG_mass_percentage - truth_OEW_CG_mass_percentage)
    assert error_pct < 0.1,\
        f"OEW CG mass percentage mismatch. Expected: {truth_OEW_CG_mass_percentage}%, Got: {mission_vehicle.mass_properties.OEW_CG_mass_percentage}%"

    print("****************************************************************")
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

def BWB_analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = BWB_base_analysis(config)
        analyses[tag] = analysis

    return analyses

def Transport_analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = Transport_base_analysis(config)
        analyses[tag] = analysis

    return analyses

def GA_analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = GA_base_analysis(config)
        analyses[tag] = analysis

    return analyses

def EVTOL_analyses_setup(configs):
    """Set up analyses for each of the different configurations."""

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # Build a base analysis for each configuration. Here the base analysis is always used, but
    # this can be modified if desired for other cases.
    for tag,config in configs.items():
        analysis = EVTOL_base_analysis(config)
        analyses[tag] = analysis

    return analyses

def BWB_base_analysis(vehicle):
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
    geometry.settings.compute_fuel_volume = True
    geometry.settings.update_max_fuel = True
    analyses.append(geometry)
    

    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_BWB() 
    weights.aircraft_type                                                    = 'BWB'
    weights.settings.FLOPS.fidelity                                          = 'Complex' 
    weights.settings.run_weights_analysis                                    = True
    weights.settings.run_center_of_gravity_analysis                          = True
    weights.settings.run_moments_of_inertia_analysis                         = True
    weights.print_weight_analysis_report                  = False
    analyses.append(weights)

    return analyses  


def Transport_base_analysis(vehicle):
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
    geometry.settings.compute_fuel_volume = True
    geometry.settings.update_max_fuel = False
    analyses.append(geometry) 

    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------ 
    weights                                              = RCAIDE.Framework.Analyses.Weights.Conventional_Transport()
    weights.aircraft_type                                = "Transport"
    weights.method                                       = 'Raymer'
    weights.settings.use_max_fuel_weight                 = False
    weights.settings.cargo_doors_number                  = 2
    weights.settings.cargo_doors_clamshell               = True
    weights.settings.run_weights_analysis                = True
    weights.settings.run_center_of_gravity_analysis      = True
    weights.settings.run_moments_of_inertia_analysis     = True
    weights.print_weight_analysis_report                  = False
    analyses.append(weights)

    return analyses


def GA_base_analysis(vehicle):
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

    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------   
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_General_Aviation()
    weights.settings.run_weights_analysis                = True
    weights.settings.run_center_of_gravity_analysis      = True
    weights.settings.run_moments_of_inertia_analysis     = True
    weights.print_weight_analysis_report                  = False
    analyses.append(weights)

    return analyses


def EVTOL_base_analysis(vehicle):
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

    # ------------------------------------------------------------------
    #  Weights
    # ------------------------------------------------------------------  
    weights                                              = RCAIDE.Framework.Analyses.Weights.Electric_VTOL()
    weights.method                                       = 'Physics_Based'
    weights.aircraft_type                                = 'VTOL'
    weights.settings.safety_factor                       = 1.5
    weights.settings.miscelleneous_weight_factor         = 1.1
    weights.settings.disk_area_factor                    = 1.15
    weights.settings.max_thrust_to_weight_ratio          = 1.1
    weights.settings.max_g_load                          = 3.8
    weights.settings.run_weights_analysis                = True
    weights.settings.run_center_of_gravity_analysis      = True
    weights.settings.run_moments_of_inertia_analysis     = True
    weights.print_weight_analysis_report                  = False
    analyses.append(weights)

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


