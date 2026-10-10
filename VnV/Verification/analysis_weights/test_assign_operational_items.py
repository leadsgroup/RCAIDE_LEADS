# VnV/Verification/analysis_weights/test_assign_operational_items.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import pytest

import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.Common import assign_operational_items

OPERATIONAL_ITEMS = Data(total=1000.0, flight_crew=200.0)

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_vehicle_without_body_is_unchanged():
    assert assign_operational_items(RCAIDE.Vehicle(), OPERATIONAL_ITEMS) is None


def test_bodies_without_seats_split_mass_and_sit_at_mid_length():
    fuselage               = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.origin        = [[1.0, 0.0, 0.2]]
    fuselage.lengths.total = 30.0
    bwb                    = RCAIDE.Library.Components.Wings.Blended_Wing_Body()
    bwb.origin             = [[2.0, 0.0, 0.5]]
    bwb.chords.root        = 30.0
    vehicle                = RCAIDE.Vehicle()
    vehicle.append_component(fuselage)
    vehicle.append_component(bwb)

    assign_operational_items(vehicle, OPERATIONAL_ITEMS)

    assert fuselage.operational_items.mass_properties.mass == pytest.approx(500.0)
    assert bwb.operational_items.mass_properties.mass      == pytest.approx(500.0)
    assert fuselage.operational_items.mass_properties.center_of_gravity == [[16.0, 0.0, 0.2]]
    assert bwb.operational_items.mass_properties.center_of_gravity      == [[17.0, 0.0, 0.5]]


def test_user_origin_overrides_automatic_location():
    fuselage                          = RCAIDE.Library.Components.Fuselages.Fuselage()
    fuselage.lengths.total            = 30.0
    fuselage.operational_items.origin = [[5.0, 0.0, 0.0]]
    vehicle                           = RCAIDE.Vehicle()
    vehicle.append_component(fuselage)

    assign_operational_items(vehicle, OPERATIONAL_ITEMS)

    assert fuselage.operational_items.mass_properties.mass == pytest.approx(1000.0)
    assert fuselage.operational_items.mass_properties.center_of_gravity == [[0.0, 0.0, 0.0]]
