# VnV/Verification/analysis_weights/test_compute_system_center_of_gravity.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import pytest

import RCAIDE
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity import compute_system_center_of_gravity

Systems = RCAIDE.Library.Components.Powertrain.Systems

# ----------------------------------------------------------------------------------------------------------------------
#  Vehicles
# ----------------------------------------------------------------------------------------------------------------------
def bwb_vehicle(aft_center_body_length=6.0):
    """Blended wing body only: nose at x = 2, root chord 30, LEMAC 10, MAC 12, spars at 10 and 60 percent chord."""
    bwb                             = RCAIDE.Library.Components.Wings.Blended_Wing_Body()
    bwb.origin                      = [[2.0, 0.0, 0.5]]
    bwb.chords.root                 = 30.0
    bwb.chords.mean_aerodynamic     = 12.0
    bwb.LEMAC                       = 10.0
    bwb.aft_center_body.length      = aft_center_body_length
    vehicle                         = RCAIDE.Vehicle()
    vehicle.append_component(bwb)
    return vehicle


def fuselage_vehicle():
    fuselage               = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.lengths.total = 30.0
    vehicle                = RCAIDE.Vehicle()
    vehicle.append_component(fuselage)
    return vehicle


def place(system, vehicle):
    compute_system_center_of_gravity(system, vehicle)
    return system.mass_properties.center_of_gravity[0]


# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_bwb_environmental_controls_between_spars():
    x, y, z = place(Systems.Environmental_Controls(), bwb_vehicle())
    assert (x, y, z) == pytest.approx((10.0 + 0.35 * 12.0, 0.0, 0.5))


def test_bwb_auxiliary_power_unit_in_aft_center_body():
    assert place(Systems.Auxiliary_Power_Unit(), bwb_vehicle())[0] == pytest.approx(2.0 + 30.0 - 6.0 / 2)


def test_bwb_without_aft_center_body_uses_95_percent_chord():
    assert place(Systems.Auxiliary_Power_Unit(), bwb_vehicle(0.0))[0] == pytest.approx(2.0 + 0.95 * 30.0)


def test_furnishings_without_seat_layout_use_generic_cabin():
    # cabin spans 20 to 80 percent of the body; furnishings at 52.5 percent of it
    assert place(Systems.Furnishings(), bwb_vehicle())[0] == pytest.approx(2.0 + 6.0 + 0.525 * 18.0)


def test_unplaced_system_type_is_unchanged():
    system = Systems.Water_Tank()
    assert compute_system_center_of_gravity(system, bwb_vehicle()) is None
    np.testing.assert_array_equal(system.mass_properties.center_of_gravity, [[0.0, 0.0, 0.0]])


def test_vehicle_without_body_is_unchanged():
    assert compute_system_center_of_gravity(Systems.Avionics(), RCAIDE.Vehicle()) is None


def test_vehicle_without_main_wing_is_unchanged():
    assert compute_system_center_of_gravity(Systems.Avionics(), fuselage_vehicle()) is None
