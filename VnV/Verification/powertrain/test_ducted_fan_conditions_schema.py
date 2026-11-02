# VnV/Verification/powertrain/test_ducted_fan_conditions_schema.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys

import numpy as np
import pytest

import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Powertrain import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan import compute_ducted_fan_performance, ducted_fan_conditions_schema

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "Vehicles"))
from NASA_X48 import vehicle_setup

METHODS = ["Blade_Element_Momentum_Theory", "Rankine_Froude_Momentum_Theory"]
INPUTS  = {"orientation", "commanded_thrust_vector_angle", "throttle", "omega"}


@pytest.mark.parametrize("method", METHODS)
def test_every_method_fills_the_same_schema(method):
    network    = list(vehicle_setup(True, method).networks.values())[0]
    ducted_fan = list(network.propulsors.values())[0].ducted_fan
    distributor = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    ducted_fan.assigned_distributors = [[distributor.tag]]
    state      = setup_operating_conditions(ducted_fan, distributor, velocity_range=np.array([30., 50.]), altitude=1000.)
    conditions = state.conditions.energy.converters[ducted_fan.tag]
    conditions.omega[:, 0] = 6000 * Units.rpm
    compute_ducted_fan_performance(ducted_fan, state.conditions)

    schema  = ducted_fan_conditions_schema(ducted_fan)
    results = state.conditions.energy.converters[ducted_fan.tag]
    for key, shape in schema.items():
        assert key in results and np.shape(results[key]) == (2,) + shape, key
    for key in ("thrust", "torque", "power", "thrust_coefficient", "efficiency"):
        assert not np.any(np.isnan(results[key])), key                       # every method computes the basics
    for key in INPUTS:
        assert key in results, key                                            # inputs set before the model survive
    written = {key for key, value in results.items() if isinstance(value, np.ndarray) and np.ndim(value) >= 2}
    assert not written - set(schema) - INPUTS, written - set(schema) - INPUTS
