# VnV/Verification/analysis_weights/test_compute_propulsor_center_of_gravity.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import pytest

import RCAIDE
from RCAIDE.Library.Methods.Mass_Properties.Center_of_Gravity import compute_propulsor_center_of_gravity

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_engine_length_places_group_at_52_5_percent():
    turbofan        = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()
    turbofan.origin = [[10.0, 0.0, 0.0]]
    turbofan.length = 4.0
    compute_propulsor_center_of_gravity(turbofan)
    assert turbofan.mass_properties.center_of_gravity[0] == pytest.approx([0.525 * 4.0, 0.0, 0.0])


def test_no_engine_or_nacelle_length_is_unchanged():
    turbofan = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()
    compute_propulsor_center_of_gravity(turbofan)
    np.testing.assert_array_equal(turbofan.mass_properties.center_of_gravity, [[0.0, 0.0, 0.0]])
