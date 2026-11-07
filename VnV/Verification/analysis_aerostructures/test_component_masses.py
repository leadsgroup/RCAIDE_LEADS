# VnV/Verification/analysis_aerostructures/test_component_masses.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.discretize_wing         import discretize_wing
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.assign_component_masses import assign_component_masses
import numpy as np
import sys
import os

base_dir      = os.path.dirname(os.path.abspath(__file__))
vehicles_path = os.path.abspath(os.path.join(base_dir, "..", "..", "Vehicles"))
if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)

from Boeing_737 import vehicle_setup

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    check_integral_tank_fuel_is_spread_over_its_bay()
    check_components_on_the_other_half_are_skipped()
    check_tail_carries_no_main_wing_components()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_vehicle():
    """Boeing 737 with two integral wing tanks and two wing-mounted turbofans, plus two wing battery modules,
    one on each side, to check the half-wing assignment."""
    vehicle = vehicle_setup()
    network = list(vehicle.networks.values())[0]
    pack    = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack()
    for tag, y in [('starboard_module', 6.0), ('port_module', -6.0)]:
        module          = RCAIDE.Library.Components.Powertrain.Sources.Batteries.Modules.Lithium_Ion_NMC()
        module.tag      = tag
        module.origin   = [[15.0, y, -0.5]]
        module.wing_tag = 'main_wing'
        pack.append_module(module)
    network.sources.append(pack)
    return vehicle


def masses_on(vehicle, wing_tag):
    wing      = vehicle.wings[wing_tag]
    node_data = discretize_wing(wing, 200)
    return assign_component_masses(vehicle, wing, node_data), node_data

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def check_integral_tank_fuel_is_spread_over_its_bay():
    vehicle         = build_vehicle()
    masses, nodes   = masses_on(vehicle, 'main_wing')
    wing            = vehicle.wings.main_wing
    assert len(masses.fuel_spread) == 2, len(masses.fuel_spread)

    # each half of a symmetric wing carries half of each tank, spread only between its bounding segments
    mean_Y = []
    for (fuel_tag, tank_mass, share, fraction), bounds in zip(masses.fuel_spread, [('root', 'yehudi'), ('yehudi', 'section_2')]):
        assert share == 0.5
        assert abs(np.sum(fraction) - 1.0) < 1e-12
        y_in  = wing.segments[bounds[0]].percent_span_location * nodes.total_span
        y_out = wing.segments[bounds[1]].percent_span_location * nodes.total_span
        outside = (nodes.Y_elems < y_in) | (nodes.Y_elems > y_out)
        assert np.all(fraction[outside] == 0.0)
        assert np.all(fraction[~outside] > 0.0)
        mean_Y.append(np.sum(fraction * nodes.Y_elems))

    # the inboard tank's fuel sits inboard of the outboard tank's, and both off the root
    assert 0.0 < mean_Y[0] < mean_Y[1], mean_Y
    return


def check_components_on_the_other_half_are_skipped():
    vehicle       = build_vehicle()
    masses, nodes = masses_on(vehicle, 'main_wing')

    # the starboard turbofan (y = 4.86 m) and starboard battery module are carried; the port ones are not
    assert len(masses.point_masses) == 2, masses.point_pts
    assert np.all(masses.point_pts[:, 1] > 0.0), masses.point_pts
    return


def check_tail_carries_no_main_wing_components():
    vehicle   = build_vehicle()
    masses, _ = masses_on(vehicle, 'horizontal_stabilizer')
    assert len(masses.point_masses) == 0 and len(masses.fuel_points) == 0 and len(masses.fuel_spread) == 0
    return


if __name__ == '__main__':
    main()
