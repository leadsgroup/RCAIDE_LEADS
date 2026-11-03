# VnV/Verification/network_internal_combustion_engine/test_identical_propulsor_reuse.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from copy import deepcopy

import numpy as np
import pytest

import ICE_test
import ICE_constant_speed_test
from Cessna_172 import vehicle_setup, configs_setup


# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def twin(vehicle, identical):
    """The vehicle with a second, mirrored copy of its engine; identical sets whether it reuses the first's results."""
    network          = list(vehicle.networks.values())[0]
    first            = list(network.propulsors.values())[0]
    second           = deepcopy(first)
    second.tag       = first.tag + "_2"
    second.engine.tag, second.propeller.tag = first.engine.tag + "_2", first.propeller.tag + "_2"
    second.origin    = [[first.origin[0][0], 1.0, first.origin[0][2]]]
    second.identical_propulsors = identical
    network.propulsors.append(second)
    return vehicle, first, second


def fly(engine_type, identical):
    """One evaluation of the engine type's VnV cruise mission with the twin-engine vehicle."""
    vehicle, first, second = twin(vehicle_setup() if engine_type == "ICE" else
                                  ICE_constant_speed_test.ICE_CS(vehicle_setup()), identical)
    if engine_type == "ICE":
        analyses = ICE_test.analyses_setup(configs_setup(vehicle))
        mission  = ICE_test.missions_setup(ICE_test.mission_setup(analyses)).base_mission
    else:
        analyses = ICE_constant_speed_test.analyses_setup(ICE_constant_speed_test.configs_setup(vehicle))
        mission  = ICE_constant_speed_test.missions_setup(analyses).base_mission
    mission.segments.cruise.assigned_control_variables.throttle.assigned_propulsors = [[first.tag, second.tag]]
    return mission.evaluate().segments.cruise.state.conditions, first, second


def leaves(D, prefix=""):
    """path -> array for every numpy array leaf of a nested Data."""
    out = {}
    for k, v in D.items():
        path = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            out.update(leaves(v, path))
        elif isinstance(v, np.ndarray):
            out[path] = v
    return out


# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("engine_type", ["ICE", "constant_speed_ICE"])
def test_reused_results_equal_the_stored_engine(engine_type, monkeypatch):
    calls = []
    import RCAIDE.Library.Components.Powertrain.Propulsors as Propulsors
    cls   = Propulsors.Internal_Combustion_Engine if engine_type == "ICE" else Propulsors.Constant_Speed_Internal_Combustion_Engine
    reuse = cls.reuse_stored_data
    monkeypatch.setattr(cls, "reuse_stored_data", lambda self, *a, **k: calls.append(self.tag) or reuse(self, *a, **k))

    conditions, first, second = fly(engine_type, identical=True)
    assert calls and set(calls) == {second.tag}

    for container, a, b in ((conditions.energy.propulsors, first.tag, second.tag),
                            (conditions.energy.converters, first.engine.tag, second.engine.tag),
                            (conditions.energy.converters, first.propeller.tag, second.propeller.tag)):
        stored, reused = leaves(container[a]), leaves(container[b])
        assert set(stored) == set(reused)
        for path in stored:
            assert not np.shares_memory(stored[path], reused[path]), path
            if path != "outputs.moment":                                   # depends on each engine's position
                assert np.array_equal(stored[path], reused[path], equal_nan=True), path


@pytest.mark.parametrize("engine_type", ["ICE", "constant_speed_ICE"])
def test_reuse_matches_computing_each_engine(engine_type):
    # with reuse off the second engine has its own unknowns, so the two runs agree to the solver tolerance (1e-6)
    reused,   _, second = fly(engine_type, identical=True)
    computed, _, _      = fly(engine_type, identical=False)
    for path in ("weights.vehicle.mass_rate", "frames.inertial.position_vector", "frames.body.total_force_vector"):
        assert np.allclose(reused.deep_get(path), computed.deep_get(path), rtol=1e-5, atol=1e-8), path
    assert np.allclose(reused.energy.converters[second.engine.tag].power.propulsive,
                       computed.energy.converters[second.engine.tag].power.propulsive, rtol=1e-5)
