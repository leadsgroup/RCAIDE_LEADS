# VnV/Verification/network_internal_combustion_engine/test_identical_propulsor_reuse.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
from copy import deepcopy

import numpy as np

import RCAIDE.Library.Components.Powertrain.Propulsors as Propulsors

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Vehicles"))
import ICE_test
import ICE_constant_speed_test
from Cessna_172 import vehicle_setup, configs_setup

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # With identical_propulsors, a second engine reuses the first engine's results instead of computing its own.
    # This adds a second engine to the Cessna 172 and checks both piston engine types.
    for engine_type in ["ICE", "constant_speed_ICE"]:
        check_reused_results_equal_the_stored_engine(engine_type)
        check_reuse_matches_computing_each_engine(engine_type)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_reused_results_equal_the_stored_engine(engine_type):
    # record which engines call reuse_stored_data during the mission
    propulsor_class = Propulsors.Internal_Combustion_Engine if engine_type == "ICE" else Propulsors.Constant_Speed_Internal_Combustion_Engine
    original_reuse  = propulsor_class.reuse_stored_data
    calls           = []

    def recording_reuse(self, *args, **kwargs):
        calls.append(self.tag)
        return original_reuse(self, *args, **kwargs)

    propulsor_class.reuse_stored_data = recording_reuse
    try:
        conditions, first, second = fly(engine_type, identical=True)
    finally:
        propulsor_class.reuse_stored_data = original_reuse

    # only the second engine reuses
    assert calls and set(calls) == {second.tag}

    # the second engine, its engine core and its propeller hold the same values in their own memory
    for container, first_tag, second_tag in ((conditions.energy.propulsors, first.tag, second.tag),
                                             (conditions.energy.converters, first.engine.tag, second.engine.tag),
                                             (conditions.energy.converters, first.propeller.tag, second.propeller.tag)):
        stored, reused = leaves(container[first_tag]), leaves(container[second_tag])
        assert set(stored) == set(reused)
        for path in stored:
            assert not np.shares_memory(stored[path], reused[path]), path
            if path != "outputs.moment":   # the moment depends on each engine's position
                assert np.array_equal(stored[path], reused[path], equal_nan=True), path
    return


def check_reuse_matches_computing_each_engine(engine_type):
    # with reuse off the second engine has its own unknowns, so the two runs agree to the solver tolerance (1e-6)
    reused,   _, second = fly(engine_type, identical=True)
    computed, _, _      = fly(engine_type, identical=False)
    for path in ("weights.vehicle.mass_rate", "frames.inertial.position_vector", "frames.body.total_force_vector"):
        assert np.allclose(reused.deep_get(path), computed.deep_get(path), rtol=1e-5, atol=1e-8), path
    assert np.allclose(reused.energy.converters[second.engine.tag].power.propulsive,
                       computed.energy.converters[second.engine.tag].power.propulsive, rtol=1e-5)
    return

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


def leaves(tree, prefix=""):
    """path -> array for every numpy array in a nested Data."""
    out = {}
    for key, value in tree.items():
        path = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict):
            out.update(leaves(value, path))
        elif isinstance(value, np.ndarray):
            out[path] = value
    return out


if __name__ == '__main__':
    main()
