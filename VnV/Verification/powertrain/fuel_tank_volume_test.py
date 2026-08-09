# VnV/Verification/powertrain/fuel_tank_volume_test.py
#
# 
# Created: Aug 2025, S. Shekar
# Modified: Apr 2026, S. Shekar, S. Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
import RCAIDE
from RCAIDE.Framework.Core                          import Units , Data 
from RCAIDE.Library.Mission.Common.Pre_Process import geometry
from RCAIDE.Library.Mission.Common.Pre_Process import mass_properties
from RCAIDE.Library.Plots                           import *        


# python imports     
import numpy as np  
import sys
import os
import matplotlib.pyplot as plt  


base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from BWB         import vehicle_setup as BWB_vehicle_setup
from Boeing_737  import vehicle_setup as B737_vehicle_setup
from Navion      import vehicle_setup as Nav_vehicle_setup
import time

# ----------------------------------------------------------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------------------------------------------------------

def main():
    ti = time.time()
    integral_fuel_tank_volume_test()
    non_integral_tank_test()
    pressurized_tank_test()
    # -------------------------------------------------------------
    # Run test only if Python version >= 3.11
    # Shapely < 2.1 (and Python < 3.11) may not include functions
    # like 'maximum_inscribed_circle' required for this test.
    # -------------------------------------------------------------
    if sys.version_info >= (3, 11):
        non_conformal_lh2_fuel_tank_volume_test()
        conformal_lh2_fuel_tank_volume_test()
        non_conformal_lng_fuel_tank_volume_test()
    else:
        print("Skipping non_conformal_lh2_fuel_tank_volume_test() and conformal_lh2_fuel_tank_volume_test():\
            Shapely lacks 'maximum_inscribed_circle' support for Python < 3.11.")

    elapsed_time = time.time() - ti
    elapsed_time_min = elapsed_time / 60
    print('Elapsed time (min): ', elapsed_time_min)
    return

def integral_fuel_tank_volume_test():

    print('\n----- Integral Fuel Tank Volume Test -----')

    wing_volume_true  = 21.519610119449926
    total_volume_true = 79.86653355539846

    vehicle   = B737_vehicle_setup()
    fuel_line = vehicle.networks.fuel.distributors.fuel_line
    vehicle.networks.fuel.sources.clear()

    # ---- Main Wing Tanks ----
    wing_tank_1                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)
    wing_tank_1.fuel_flow_split_ratio        = 0.5
    wing_tank_1.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A()
    wing_tank_1.segments_bounding_tank       = ['root', 'yehudi']
    wing_tank_1.segments_percent_chord_start = [0.1, 0.1]
    wing_tank_1.segments_percent_chord_end   = [0.7, 0.7]
    wing_tank_1.assigned_distributors        = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(wing_tank_1)

    wing_tank_2                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.wings.main_wing)
    wing_tank_2.fuel_flow_split_ratio        = 0.5
    wing_tank_2.fuel                         = RCAIDE.Library.Attributes.Propellants.Jet_A()
    wing_tank_2.segments_bounding_tank       = ['yehudi', 'section_2']
    wing_tank_2.segments_percent_chord_start = [0.1, 0.1]
    wing_tank_2.segments_percent_chord_end   = [0.7, 0.7]
    wing_tank_2.assigned_distributors        = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(wing_tank_2)

    configs  = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    for analysis in analyses:
        analysis.geometry.settings.compute_fuel_volume = True
    mission  = mission_setup(analyses)
    geometry(mission)

    wing_volume_computed = mission.segments.cruise.analyses.vehicle.volume_properties.max_fuel

    # ---- Fuselage Tanks ----
    fus_tank_1 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.fuselages.fuselage)
    fus_tank_1.fuel_flow_split_ratio    = 0.5
    fus_tank_1.fuel                     = RCAIDE.Library.Attributes.Propellants.Jet_A()
    fus_tank_1.segments_bounding_tank   = ['segment_5','segment_6']
    fus_tank_1.assigned_distributors    = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fus_tank_1)

    fus_tank_2 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.fuselages.fuselage)
    fus_tank_2.fuel_flow_split_ratio    = 0.5
    fus_tank_2.fuel                     = RCAIDE.Library.Attributes.Propellants.Jet_A()
    fus_tank_2.segments_bounding_tank   = ['segment_6','segment_7']
    fus_tank_2.assigned_distributors    = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fus_tank_2)

    fus_tank_3 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank(vehicle.fuselages.fuselage)
    fus_tank_3.fuel_flow_split_ratio    = 0.5
    fus_tank_3.fuel                     = RCAIDE.Library.Attributes.Propellants.Jet_A()
    fus_tank_3.segments_bounding_tank   = ['segment_7','segment_8']
    fus_tank_3.assigned_distributors    = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fus_tank_3)

    configs  = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    for analysis in analyses:
        analysis.geometry.settings.compute_fuel_volume = True
    mission  = mission_setup(analyses)
    geometry(mission) 

    total_volume_computed = mission.segments.cruise.analyses.vehicle.volume_properties.max_fuel

    # ---- Print Results ----
    error = Data()
    error.wing_tanks_only    = np.abs((wing_volume_true  - wing_volume_computed)  / wing_volume_true)
    error.wing_and_fus_tanks = np.abs((total_volume_true - total_volume_computed) / total_volume_true)

    print(f'  {"Quantity":<30s} {"Truth":>14s} {"Computed":>14s} {"Error":>12s}')
    print(f'  {"Wing tanks max fuel (m³)":<30s} {wing_volume_true:>14.6f} {wing_volume_computed:>14.6f} {error.wing_tanks_only:>12.4e}')
    print(f'  {"Wing + fus tanks max fuel (m³)":<30s} {total_volume_true:>14.6f} {total_volume_computed:>14.6f} {error.wing_and_fus_tanks:>12.4e}')

    for k, v in list(error.items()):
        assert np.abs(v) < 1e-6, f'Integral tank regression failed: {k}'

    return

def non_integral_tank_test():
    print('\n----- Non-Integral Tank Test -----')

    # --- Cylindrical MOI and CoG (lines 218 and 246 in Non_Integral_Tank) ---
    # These branches are never reached via Cryogenic_Tank (which overrides the methods).
    cyl_tank = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank()
    cyl_tank.geometry_type      = 'cylindrical'
    cyl_tank.lengths.external   = 8.0
    cyl_tank.diameters.external = 2.0
    cyl_tank.wall_thickness     = 0.05
    cyl_tank.fuel               = RCAIDE.Library.Attributes.Propellants.Jet_A1()
    cyl_tank.compute_moments_of_inertia(None)
    cyl_tank.compute_center_of_gravity(None)

    # --- Wing-mounted cylindrical and transverse Non_Integral_Tank (lines 174-181) ---
    # These use shapely so require Python >= 3.11.
    if sys.version_info >= (3, 11):
        # Wing-mounted cylindrical tank (lines 174-177)
        vehicle   = B737_vehicle_setup()
        fuel_line = vehicle.networks.fuel.distributors.fuel_line
        vehicle.networks.fuel.sources.clear()

        wing_tank = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(vehicle.wings.main_wing)
        wing_tank.tag                    = 'wing_cyl_tank'
        wing_tank.geometry_type          = 'cylindrical'
        wing_tank.segments_bounding_tank = ['root', 'yehudi']
        wing_tank.fuel                   = RCAIDE.Library.Attributes.Propellants.Jet_A()
        wing_tank.assigned_distributors = [[fuel_line.tag]]
        vehicle.networks.fuel.sources.append(wing_tank)

        wing_tank.compute_volume(vehicle.wings, vehicle.fuselages, vehicle.networks.fuel.sources)
        assert wing_tank.volume_properties.net_volume > 0, \
            f'Wing-mounted cylindrical Non_Integral_Tank volume should be > 0, got {wing_tank.volume_properties.net_volume}'

        # Transverse Non_Integral_Tank (lines 178-181)
        vehicle_bwb   = BWB_vehicle_setup()
        fuel_line_bwb = vehicle_bwb.networks.fuel.distributors.fuel_line
        vehicle_bwb.networks.fuel.sources.clear()

        trans_tank = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank(vehicle_bwb.wings.main_wing)
        trans_tank.tag                           = 'transverse_tank'
        trans_tank.transverse_tank               = True
        trans_tank.transverse_tank_chord_bounds  = [0.65, 0.9]
        trans_tank.transverse_tank_segment_bound = 'cabin_wall'
        trans_tank.radial_offset                 = 0.2
        trans_tank.orientation_euler_angles      = [0, 0, np.pi/2]
        trans_tank.xz_plane_symmetric            = False
        trans_tank.fuel                          = RCAIDE.Library.Attributes.Propellants.Jet_A()
        trans_tank.assigned_distributors = [[fuel_line_bwb.tag]]
        vehicle_bwb.networks.fuel.sources.append(trans_tank)

        trans_tank.compute_volume(vehicle_bwb.wings, vehicle_bwb.fuselages, vehicle_bwb.networks.fuel.sources)
        assert trans_tank.volume_properties.net_volume > 0, \
            f'Transverse Non_Integral_Tank volume should be > 0, got {trans_tank.volume_properties.net_volume}'
    else:
        print('  Skipping wing-mounted and transverse Non_Integral_Tank tests: shapely requires Python >= 3.11')

    print('  PASSED')


def pressurized_tank_test():
    print('\n----- Pressurized Tank Test -----')

    vehicle   = B737_vehicle_setup()
    fuel_line = vehicle.networks.fuel.distributors.fuel_line
    vehicle.networks.fuel.sources.clear()

    # ---- Standalone cylindrical LPG tank ----
    # Parameters match the regional-jet LPG configuration in RESEARCH/12_PtX_Boeing/Task_4/
    # Regional_Aircraft_Cryo.py: LPG is stored as a pressurized liquid near ISA standard-day
    # temperature, with design_pressure set by the max expected hot-day vapor pressure.
    tank                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Pressurized_Tank()
    tank.tag                    = 'lpg_pressurized_tank'
    tank.fuel                   = RCAIDE.Library.Attributes.Propellants.Liquid_Petroleum_Gas()
    tank.design_inlet_temperature = 288    # K, ISA standard-day delivery temperature
    tank.design_pressure        = 1.5e6    # Pa, 15 bar max operating pressure
    tank.design_altitude        = 35000 * Units.ft
    tank.ullage_volume_fraction = 0.07
    tank.diameters.external     = 2.25
    tank.lengths.external       = 3.5
    tank.assigned_distributors  = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(tank)

    tank.compute_volume(vehicle.wings, vehicle.fuselages, vehicle.networks.fuel.sources)
    assert tank.volume_properties.net_volume > 0, \
        f'Pressurized_Tank net volume should be > 0, got {tank.volume_properties.net_volume}'
    assert tank.wall_thickness > 0, \
        f'Pressurized_Tank wall thickness should be sized > 0, got {tank.wall_thickness}'

    tank.compute_moments_of_inertia(vehicle)
    tank.compute_center_of_gravity(vehicle)

    # ---- Standalone prismatic LPG tank ----
    prismatic_tank                       = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Pressurized_Tank()
    prismatic_tank.tag                   = 'lpg_prismatic_tank'
    prismatic_tank.geometry_type         = 'prismatic'
    prismatic_tank.fuel                  = RCAIDE.Library.Attributes.Propellants.Liquid_Petroleum_Gas()
    prismatic_tank.design_inlet_temperature = 288
    prismatic_tank.design_pressure       = 1.5e6
    prismatic_tank.design_altitude       = 35000 * Units.ft
    prismatic_tank.ullage_volume_fraction = 0.07
    prismatic_tank.lengths.external      = 1.0
    prismatic_tank.widths.external       = 0.8
    prismatic_tank.heights.external      = 0.6
    prismatic_tank.wall_thickness        = 5 * Units.mm
    prismatic_tank.assigned_distributors = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(prismatic_tank)

    prismatic_tank.compute_volume(vehicle.wings, vehicle.fuselages, vehicle.networks.fuel.sources)
    assert prismatic_tank.volume_properties.net_volume > 0, \
        f'Prismatic Pressurized_Tank net volume should be > 0, got {prismatic_tank.volume_properties.net_volume}'

    prismatic_tank.compute_moments_of_inertia(vehicle)
    prismatic_tank.compute_center_of_gravity(vehicle)

    print('  PASSED')
    return


def non_conformal_lh2_fuel_tank_volume_test():

    print('\n----- Non-Conformal LH2 Fuel Tank Volume Test -----')

    fuel_volume_true = 612.218299
    vehicle          = BWB_vehicle_setup()
    fuel_line        = vehicle.networks.fuel.distributors.fuel_line
    vehicle.networks.fuel.sources.clear()

    # ---- LH2 wing tank (non-conformal) ----
    fuel_tank_1                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank(vehicle.wings.main_wing)
    fuel_tank_1.tag                             = 'H2_Fuel_Tank_1'
    fuel_tank_1.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    fuel_tank_1.design_inlet_temperature        = 20
    fuel_tank_1.design_altitude                 = 30000 * Units.ft
    fuel_tank_1.design_heat_flux                = 20
    fuel_tank_1.design_total_heat_transfer      = 2000
    fuel_tank_1.ullage_volume_fraction          = 0.07
    fuel_tank_1.inner_structure.material        = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_1.insulation.material             = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_1.gravimetric_efficiency          = 0.5
    fuel_tank_1.segments_bounding_tank          = ['fuselage_section_3', 'wing_section_2']
    fuel_tank_1.segments_percent_chord_start    = [0.2,0.2]
    fuel_tank_1.segments_percent_chord_end      = [0.6,0.6]
    fuel_tank_1.wall_thickness                  = 2*Units.inches
    fuel_tank_1.assigned_distributors = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fuel_tank_1)

    # ---- Cylindrical non-integral tank ----
    fuel_tank_2                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank()
    fuel_tank_2.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    fuel_tank_2.tag                             = 'H2_Fuel_Tank_2'
    fuel_tank_2.design_inlet_temperature        = 20
    fuel_tank_2.design_altitude                 = 30000 * Units.ft
    fuel_tank_2.design_heat_flux                = 20
    fuel_tank_2.design_total_heat_transfer      = 2000
    fuel_tank_2.ullage_volume_fraction          = 0.07
    fuel_tank_2.geometry_type                   = 'cylindrical'
    fuel_tank_2.lengths.external                = 8
    fuel_tank_2.diameters.external              = 4
    fuel_tank_2.assigned_distributors = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fuel_tank_2)
 
    # ---- Prismatic non-integral tank ----
    fuel_tank_2a                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank()
    fuel_tank_2a.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    fuel_tank_2a.tag                             = 'H2_Fuel_Tank_2a'
    fuel_tank_2a.design_inlet_temperature        = 20
    fuel_tank_2a.design_altitude                 = 30000 * Units.ft
    fuel_tank_2a.design_heat_flux                = 20
    fuel_tank_2a.design_total_heat_transfer      = 2000
    fuel_tank_2a.ullage_volume_fraction          = 0.07
    fuel_tank_2a.geometry_type                   = 'prismatic'
    fuel_tank_2a.lengths.external                = 1
    fuel_tank_2a.widths.external                 = 1
    fuel_tank_2a.heights.external                = 1
    fuel_tank_2a.wall_thickness                  = 2*Units.inches
    fuel_tank_2a.fuel.mass_properties.mass       = 0.1
    fuel_tank_2a.assigned_distributors = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fuel_tank_2a)

    # ---- Wing-mounted prismatic non-integral tank ----
    fuel_tank_3                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank(vehicle.wings.main_wing)
    fuel_tank_3.tag                             = 'H2_Fuel_Tank_3'
    fuel_tank_3.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    fuel_tank_3.design_inlet_temperature        = 20
    fuel_tank_3.design_altitude                 = 30000 * Units.ft
    fuel_tank_3.design_heat_flux                = 20
    fuel_tank_3.design_total_heat_transfer      = 2000
    fuel_tank_3.ullage_volume_fraction          = 0.07
    fuel_tank_3.gravimetric_efficiency          = 0.5
    fuel_tank_3.wall_thickness                  = 2*Units.inches
    fuel_tank_3.segments_bounding_tank          = ['fuselage_section_3', 'wing_section_2']
    fuel_tank_3.segments_percent_chord_start    = [0.2 ,0.2]
    fuel_tank_3.segments_percent_chord_end      = [0.6,0.6]
    fuel_tank_3.assigned_distributors = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fuel_tank_3)

    # ---- LH2 BWB aft tank ----
    fuel_tank_4                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank(vehicle.wings.main_wing)
    fuel_tank_4.tag                           = 'H2_Fuel_Tank_4'
    fuel_tank_4.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    fuel_tank_4.design_inlet_temperature      = 20
    fuel_tank_4.design_altitude               = 30000 * Units.ft
    fuel_tank_4.design_heat_flux              = 20
    fuel_tank_4.design_total_heat_transfer    = 2000
    fuel_tank_4.ullage_volume_fraction        = 0.07
    fuel_tank_4.inner_structure.material      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_4.insulation.material           = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_4.gravimetric_efficiency        = 0.5
    fuel_tank_4.xz_plane_symmetric            = False
    fuel_tank_4.orientation_euler_angles      = [0,0,np.pi/2]
    fuel_tank_4.transverse_tank                  = True
    fuel_tank_4.transverse_tank_chord_bounds    = [0.65,0.9]
    fuel_tank_4.transverse_tank_segment_bound        = 'fuel_wall'
    fuel_tank_4.radial_offset                 = 0.2
    fuel_tank_4.assigned_distributors = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fuel_tank_4)

    # ---- Non-integral BWB aft tank ----
    fuel_tank_4a                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank(vehicle.wings.main_wing)
    fuel_tank_4a.tag                           = 'H2_Fuel_Tank_4a'
    fuel_tank_4a.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    fuel_tank_4a.design_inlet_temperature      = 20
    fuel_tank_4a.design_altitude               = 30000 * Units.ft
    fuel_tank_4a.design_heat_flux              = 20
    fuel_tank_4a.design_total_heat_transfer    = 2000
    fuel_tank_4a.ullage_volume_fraction        = 0.07
    fuel_tank_4a.gravimetric_efficiency        = 0.5
    fuel_tank_4a.xz_plane_symmetric            = False
    fuel_tank_4a.orientation_euler_angles      = [0,0,np.pi/2]
    fuel_tank_4a.transverse_tank               = True
    fuel_tank_4a.transverse_tank_chord_bounds  = [0.65,0.9]
    fuel_tank_4a.transverse_tank_segment_bound = 'cabin_wall'
    fuel_tank_4a.radial_offset                 = 0.2
    fuel_tank_4a.assigned_distributors = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fuel_tank_4a)

    # ---- LH2 BWB aft tank with specified mass ----
    fuel_tank_5                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank(vehicle.wings.main_wing)
    fuel_tank_5.tag                           = 'H2_Fuel_Tank_5'
    fuel_tank_5.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    fuel_tank_5.design_inlet_temperature      = 20
    fuel_tank_5.design_altitude               = 30000 * Units.ft
    fuel_tank_5.design_heat_flux              = 20
    fuel_tank_5.design_total_heat_transfer    = 2000
    fuel_tank_5.ullage_volume_fraction        = 0.07
    fuel_tank_5.inner_structure.material      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_5.insulation.material           = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_5.gravimetric_efficiency        = 0.5
    fuel_tank_5.xz_plane_symmetric            = False
    fuel_tank_5.orientation_euler_angles      = [0,0,np.pi/2]
    fuel_tank_5.transverse_tank                  = True
    fuel_tank_5.transverse_tank_chord_bounds    = [0.65,0.9]
    fuel_tank_5.transverse_tank_segment_bound        = 'cabin_wall'
    fuel_tank_5.radial_offset                 = 0.5
    fuel_tank_5.fuel.mass_properties.mass     = 0.1
    fuel_tank_5.assigned_distributors = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fuel_tank_5)

    # ---- Non-integral wing tank with specified mass ----
    fuel_tank_6                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank(vehicle.wings.main_wing)
    fuel_tank_6.tag                             = 'H2_Fuel_Tank_6'
    fuel_tank_6.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    fuel_tank_6.design_inlet_temperature        = 20
    fuel_tank_6.design_altitude                 = 30000 * Units.ft
    fuel_tank_6.design_heat_flux                = 20
    fuel_tank_6.design_total_heat_transfer      = 2000
    fuel_tank_6.ullage_volume_fraction          = 0.07
    fuel_tank_6.gravimetric_efficiency          = 0.5
    fuel_tank_6.wall_thickness                  = 2*Units.inches
    fuel_tank_6.fuel.mass_properties.mass       = 0.1
    fuel_tank_6.segments_bounding_tank          = ['fuel_wall', 'wing_section_2']
    fuel_tank_6.assigned_distributors = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fuel_tank_6)

    configs  = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    for analysis in analyses:
        analysis.geometry.settings.compute_fuel_volume = True
        analysis.geometry.settings.update_max_fuel = True
    mission  = mission_setup(analyses)

    geometry(mission) 

    fuel_volume_computed = mission.segments.cruise.analyses.vehicle.volume_properties.max_fuel

    # ---- Print Results ----
    error = Data()
    error.max_fuel_volume = np.abs((fuel_volume_true - fuel_volume_computed) / fuel_volume_true)

    print(f'  {"Quantity":<30s} {"Truth":>14s} {"Computed":>14s} {"Error":>12s}')
    print(f'  {"Max fuel volume (m³)":<30s} {fuel_volume_true:>14.6f} {fuel_volume_computed:>14.6f} {error.max_fuel_volume:>12.4e}')

    for k, v in list(error.items()):
        assert np.abs(v) < 5e-2, f'Non-conformal LH2 regression failed: {k}'

    return



def conformal_lh2_fuel_tank_volume_test():

    print('\n----- Conformal LH2 Fuel Tank Volume Test -----')

    fuel_volume_true = 275.10296942
    vehicle          = BWB_vehicle_setup()
    fuel_line        = vehicle.networks.fuel.distributors.fuel_line
    vehicle.networks.fuel.sources.clear()

    # ---- Conformal wing tank ----
    fuel_tank                                          = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank(vehicle.wings.main_wing)
    fuel_tank.tag                                      = 'wing_tanks'
    fuel_tank.geometry_type                            = 'conformal'
    fuel_tank.fuel                                     = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    fuel_tank.design_inlet_temperature                 = 20
    fuel_tank.design_altitude                          = 30000 * Units.ft
    fuel_tank.design_heat_flux                         = 20
    fuel_tank.design_total_heat_transfer               = 2000
    fuel_tank.ullage_volume_fraction                   = 0.07
    fuel_tank.inner_structure.material                 = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank.insulation.material                      = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank.segments_bounding_tank                   = ['fuel_wall', 'wing_section_1']
    fuel_tank.segments_percent_chord_start             = [0.2,0.2]
    fuel_tank.segments_percent_chord_end               = [0.6,0.6]
    fuel_tank.assigned_distributors = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fuel_tank)

    # ---- Conformal aft tank ----
    fuel_tank_2                                        = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank(vehicle.wings.main_wing)
    fuel_tank_2.tag                                    = 'aft_tank'
    fuel_tank_2.geometry_type                          = 'conformal'
    fuel_tank_2.fuel                                   = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    fuel_tank_2.design_inlet_temperature               = 20
    fuel_tank_2.design_altitude                        = 30000 * Units.ft
    fuel_tank_2.design_heat_flux                       = 20
    fuel_tank_2.design_total_heat_transfer             = 2000
    fuel_tank_2.ullage_volume_fraction                 = 0.07
    fuel_tank_2.inner_structure.material               = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_2.insulation.material                    = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_2.xz_plane_symmetric                     = False
    fuel_tank_2.orientation_euler_angles               = [0,0,np.pi/2]
    fuel_tank_2.transverse_tank                        = True
    fuel_tank_2.transverse_tank_chord_bounds             = [0.7,0.8]
    fuel_tank_2.transverse_tank_segment_bound                 = 'cabin_wall'
    fuel_tank_2.radial_offset                          = 0.1
    fuel_tank_2.fuel.tag                               = '_lh2'
    fuel_tank_2.assigned_distributors = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fuel_tank_2)

    configs  = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    for analysis in analyses:
        analysis.geometry.settings.compute_fuel_volume = True
        analysis.geometry.settings.update_max_fuel = True
    mission  = mission_setup(analyses)

    geometry(mission)

    fuel_volume_computed = mission.segments.cruise.analyses.vehicle.volume_properties.max_fuel

    # ---- Print Results ----
    error = Data()
    error.max_fuel_volume = np.abs((fuel_volume_true - fuel_volume_computed) / fuel_volume_true)

    print(f'  {"Quantity":<30s} {"Truth":>14s} {"Computed":>14s} {"Error":>12s}')
    print(f'  {"Max fuel volume (m³)":<30s} {fuel_volume_true:>14.6f} {fuel_volume_computed:>14.6f} {error.max_fuel_volume:>12.4e}')

    for k, v in list(error.items()):
        assert np.abs(v) < 5e-2, f'Conformal LH2 regression failed: {k}'

    return

def non_conformal_lng_fuel_tank_volume_test():

    print('\n----- Non-Conformal LNG Fuel Tank Volume Test -----')

    fuel_volume_true = 289.5878436558366
    vehicle          = BWB_vehicle_setup()
    fuel_line        = vehicle.networks.fuel.distributors.fuel_line
    vehicle.networks.fuel.sources.clear()

    # ---- LNG wing tank ----
    fuel_tank_1                                 = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank(vehicle.wings.main_wing)
    fuel_tank_1.tag                             = 'LNG_Fuel_Tank_1'
    fuel_tank_1.fuel                            = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()
    fuel_tank_1.design_inlet_temperature        = 100
    fuel_tank_1.design_altitude                 = 30000 * Units.ft
    fuel_tank_1.design_heat_flux                = 20
    fuel_tank_1.design_total_heat_transfer      = 2000
    fuel_tank_1.ullage_volume_fraction          = 0.07
    fuel_tank_1.inner_structure.material        = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_1.insulation.material             = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_1.gravimetric_efficiency     = 0.5
    fuel_tank_1.segments_bounding_tank          = ['fuselage_section_3', 'wing_section_2']
    fuel_tank_1.segments_percent_chord_start    = [0.2,0.2]
    fuel_tank_1.segments_percent_chord_end      = [0.6,0.6]
    fuel_tank_1.wall_thickness                  = 2*Units.inches
    fuel_tank_1.assigned_distributors = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fuel_tank_1)

    # ---- LNG BWB aft tank ----
    fuel_tank_2                               = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank(vehicle.wings.main_wing)
    fuel_tank_2.tag                           = 'LNG_Fuel_Tank_2'
    fuel_tank_2.fuel                          = RCAIDE.Library.Attributes.Propellants.Liquid_Natural_Gas()
    fuel_tank_2.design_inlet_temperature      = 100
    fuel_tank_2.design_altitude               = 30000 * Units.ft
    fuel_tank_2.design_heat_flux              = 20
    fuel_tank_2.design_total_heat_transfer    = 2000
    fuel_tank_2.ullage_volume_fraction        = 0.07
    fuel_tank_2.inner_structure.material      = RCAIDE.Library.Attributes.Materials.Aluminum_2219()
    fuel_tank_2.insulation.material           = RCAIDE.Library.Attributes.Materials.Vacuum_Cellular_Multilayer_Insulation()
    fuel_tank_2.gravimetric_efficiency        = 0.5
    fuel_tank_2.xz_plane_symmetric            = False
    fuel_tank_2.orientation_euler_angles      = [0,0,np.pi/2]
    fuel_tank_2.transverse_tank               = True
    fuel_tank_2.transverse_tank_chord_bounds    = [0.65,0.9]
    fuel_tank_2.transverse_tank_segment_bound        = 'fuel_wall'
    fuel_tank_2.radial_offset                 = 0.2
    fuel_tank_2.assigned_distributors = [[fuel_line.tag]]
    vehicle.networks.fuel.sources.append(fuel_tank_2)

    configs  = configs_setup(vehicle)
    analyses = analyses_setup(configs)
    for analysis in analyses:
        analysis.geometry.settings.compute_fuel_volume = True
        analysis.geometry.settings.update_max_fuel = True
    mission  = mission_setup(analyses)

    geometry(mission)

    fuel_volume_computed = mission.segments.cruise.analyses.vehicle.volume_properties.max_fuel

    # ---- Print Results ----
    error = Data()
    error.max_fuel_volume = np.abs((fuel_volume_true - fuel_volume_computed) / fuel_volume_true)

    print(f'  {"Quantity":<30s} {"Truth":>14s} {"Computed":>14s} {"Error":>12s}')
    print(f'  {"Max fuel volume (m³)":<30s} {fuel_volume_true:>14.6f} {fuel_volume_computed:>14.6f} {error.max_fuel_volume:>12.4e}')

    for k, v in list(error.items()):
        assert np.abs(v) < 5e-2, f'Non-conformal LNG regression failed: {k}'

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

        
