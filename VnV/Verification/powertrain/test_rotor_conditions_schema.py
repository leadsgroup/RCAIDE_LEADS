# VnV/Verification/powertrain/test_rotor_conditions_schema.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
from copy import deepcopy

import numpy as np
import pytest

from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Performance.rotor_aerodynamic_analysis import rotor_aerodynamic_analysis
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor import rotor_conditions_schema

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "Vehicles"))
from Cessna_172 import vehicle_setup

METHODS = ["Blade_Element_Momentum_Theory_Helmholtz_Wake", "Actuator_Disk_Theory"]


def propeller(method):
    network = list(vehicle_setup().networks.values())[0]
    rotor   = deepcopy(list(network.propulsors.values())[0].propeller)
    rotor.fidelity = method
    return rotor


@pytest.mark.parametrize("method", METHODS)
def test_every_method_fills_the_same_schema(method):
    rotor   = propeller(method)
    results = rotor_aerodynamic_analysis(rotor, np.array([30., 50.]), angular_velocity=2400 * Units.rpm)
    schema  = rotor_conditions_schema(rotor)
    for key, shape in schema.items():
        assert key in results, key
        assert np.shape(results[key]) == (2,) + shape, key
    computed = [key for key in schema if not np.all(np.isnan(results[key]))]
    for key in ("thrust", "torque", "power", "thrust_coefficient", "efficiency"):
        assert key in computed, key                                           # every method computes the basics
    written = [key for key, value in results.items() if isinstance(value, np.ndarray) and np.ndim(value) >= 2]
    unknown = set(written) - set(schema) - {"orientation", "commanded_thrust_vector_angle", "blade_pitch_command",
                                            "throttle", "omega"}
    assert not unknown, unknown                                                # methods write only schema fields


def test_rotor_without_blade_geometry_has_empty_distributions():
    rotor = propeller(METHODS[1])
    rotor.chord_distribution = 0.0
    schema = rotor_conditions_schema(rotor)
    assert schema["blade_thrust_distribution"] == (0,) and schema["disc_circulation"] == (0, rotor.number_azimuthal_stations)
    assert schema["thrust"] == (3,)
