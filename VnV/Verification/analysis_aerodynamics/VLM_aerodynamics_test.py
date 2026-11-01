# VLM_aerodynamics_test.py

import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots import *
from RCAIDE.Library.Methods.Performance                                    import aircraft_aerodynamic_analysis
from RCAIDE.Library.Methods.Performance.generate_cruise_drag_buildup_table import generate_cruise_drag_buildup_table
from RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks               import Cryogenic_Tank
import numpy as np
import sys
import os
import time

base_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(base_dir, "..", "..", "Vehicles")))
from BWB import vehicle_setup as bwb_vehicle_setup, configs_setup

# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------
def main():
    ti = time.time()
    surrogate_test()
    cruise_drag_build_up_test()
    non_surrogate_test()
    print('Elapsed time (min): ', (time.time() - ti) / 60)
    return

def surrogate_test():
    """Cruise coefficients from the VLM surrogate and the BWB cabin layout."""
    mission = mission_setup(analyses_setup(configs_setup(vehicle_setup())))
    results = mission.evaluate()

    coefficients = results.segments.cruise.conditions.aerodynamics.coefficients
    computed     = {'CL' : coefficients.lift.total[2, 0],
                    'CDi': coefficients.drag.induced.total[2, 0],
                    'CDc': coefficients.drag.compressible.total[2, 0],
                    'CD' : coefficients.drag.total[2, 0]}
    truth        = {'CL' : 0.5416187161940065,
                    'CDi': 0.011779193576296771,
                    'CDc': 0.002054383830799684,
                    'CD' : 0.025762447861669334}
    for name in truth:
        error = abs(computed[name] - truth[name]) / truth[name]
        print(f'{name}: {computed[name]}, error: {error}')
        assert error < 1e-3, f"{name} mismatch: got {computed[name]}, expected {truth[name]}"

    plot_3d_vehicle_vlm_panelization(vortex_distribution      = results.segments.cruise.analyses.aerodynamics.settings.vortex_distribution,
                                     save_filename            = "BWB_Top_View",
                                     show_wing_control_points = True,
                                     show_figure              = False)

    # cabin layout: (row, column, truth) samples of the object coordinates
    LOPA_coords = results.segments.cruise.analyses.vehicle.wings.main_wing.layout_of_passenger_accommodations.object_coordinates
    for row, column, value_true in [(22, 2, 10.0584), (34, 3, 0.4699), (124, 2, 10.0584), (68, 3, 0.9779), (0, 2, 2.9972), (201, 3, -1.9431)]:
        assert abs(LOPA_coords[row][column] - value_true) < 1e-3, f"LOPA coordinate [{row}][{column}]: got {LOPA_coords[row][column]}, expected {value_true}"
    return

def cruise_drag_build_up_test():
    mission = mission_setup(analyses_setup(configs_setup(vehicle_setup())))
    generate_cruise_drag_buildup_table(mission = mission, cruise_segment_tag = "cruise", save_filepath = base_dir)
    for filename in ("cruise_drag_buildup_parasite_zoom.png", "cruise_drag_buildup.xlsx", "cruise_drag_buildup_pie.png", "cruise_drag_buildup.png"):
        file_path = os.path.join(base_dir, filename)
        if os.path.exists(file_path):
            os.remove(file_path)
    return

def non_surrogate_test():
    """Direct VLM evaluation over an angle-of-attack sweep and its pressure coefficient plots."""
    analyses = analyses_setup(configs_setup(vehicle_setup()), use_surrogate = False)
    alpha    = np.atleast_2d(np.linspace(-5, 15, 3)).T * Units.degrees
    results  = aircraft_aerodynamic_analysis(analyses                         = analyses.cruise,
                                             angle_of_attacks                 = alpha,
                                             non_dimensional_reynolds_numbers = np.ones_like(alpha) * 1E7,
                                             temperatures                     = np.ones_like(alpha) * 340,
                                             mach_numbers                     = np.ones_like(alpha) * 0.78)

    save_path = os.path.join(base_dir, "Pressure_Coefficient_Distribution")
    plot_pressure_coefficient_distribution(results, save_figure = True, save_filename = save_path)
    for i in range(len(results.aerodynamics.coefficients.differential_surface_pressure)):
        file_path = save_path + '_' + str(i + 1) + '.png'
        assert os.path.exists(file_path), f"Pressure coefficient plot not created: {file_path}"
        os.remove(file_path)
    return

# ----------------------------------------------------------------------
#   Vehicle, Analyses and Mission
# ----------------------------------------------------------------------
def vehicle_setup():
    """BWB with tank boil-off off; this test covers aerodynamics only."""
    vehicle = bwb_vehicle_setup()
    for network in vehicle.networks:
        for source in network.sources:
            if isinstance(source, Cryogenic_Tank):
                source.boil_off_model = 'none'
    return vehicle

def analyses_setup(configs, use_surrogate = True):
    analyses = RCAIDE.Framework.Analyses.Analysis.Container()
    for tag, config in configs.items():
        analyses[tag] = base_analysis(config, use_surrogate)
    return analyses

def base_analysis(vehicle, use_surrogate = True):
    analyses         = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle

    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    geometry.settings.update_fuselage_properties = True
    analyses.append(geometry)

    weights = RCAIDE.Framework.Analyses.Weights.Conventional_BWB()
    weights.settings.FLOPS.fidelity              = 'Complex'
    weights.settings.overwrite_center_of_gravity = True
    analyses.append(weights)

    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.use_surrogate = use_surrogate
    analyses.append(aerodynamics)

    analyses.append(RCAIDE.Framework.Analyses.Energy.Energy())
    analyses.append(RCAIDE.Framework.Analyses.Planets.Earth())
    analyses.append(RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976())
    return analyses

def mission_setup(analyses):
    mission     = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'base_mission'
    Segments    = RCAIDE.Framework.Mission.Segments

    segment          = Segments.Cruise.Constant_Speed_Constant_Altitude(Segments.Segment())
    segment.tag      = "cruise"
    segment.analyses.extend(analyses.cruise)
    segment.altitude  = 40000 * Units['ft']
    segment.air_speed = 450 * Units['knots']
    segment.distance  = 7370 * Units.km

    segment.flight_dynamics.force_x                                 = True
    segment.flight_dynamics.force_z                                 = True
    segment.assigned_control_variables.throttle.active              = True
    segment.assigned_control_variables.throttle.assigned_propulsors = [['port_propulsor','starboard_propulsor','center_propulsor']]
    segment.assigned_control_variables.pitch_angle.active           = True
    mission.append_segment(segment)
    return mission

if __name__ == '__main__':
    main()
