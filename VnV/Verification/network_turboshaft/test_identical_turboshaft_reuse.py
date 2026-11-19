# VnV/Verification/network_turboshaft/test_identical_turboshaft_reuse.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from copy import deepcopy

import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Container, Data
from RCAIDE.Library.Methods.Powertrain import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Turboshaft import compute_turboshaft_performance

from turboshaft_network_test import turboshaft_Boeing_502_14

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # A second identical turboshaft reuses the first's results instead of computing its own.
    # Same throttle and flight condition, so the reused values must equal computing the second engine exactly.
    check_reused_turboshaft_equals_computing_it()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_reused_turboshaft_equals_computing_it():
    components = ["ram", "inlet_nozzle", "compressor", "combustor", "high_pressure_turbine", "low_pressure_turbine",
                  "core_nozzle"]

    # a designed turboshaft and an identical copy with its own tags
    first  = turboshaft_Boeing_502_14()
    second = deepcopy(first)
    second.tag = first.tag + "_2"
    for name in components:
        second[name].tag = first[name].tag + "_2"

    network            = Data()
    network.converters = Container()
    network.converters.append(first)
    network.converters.append(second)

    # run 1: compute the first engine, the second reuses it
    reused = operate([first, second])
    compute_turboshaft_performance(first, reused.conditions)
    second.reuse_stored_data(reused, network, stored_converter_tag=first.tag)

    # run 2: compute the second engine directly
    computed = operate([second])
    compute_turboshaft_performance(second, computed.conditions)

    # the turboshaft and all of its components hold the same fields and values in both runs
    for tag in [second.tag] + [second[name].tag for name in components]:
        reused_leaves   = leaves(reused.conditions.energy.converters[tag])
        computed_leaves = leaves(computed.conditions.energy.converters[tag])
        assert set(reused_leaves) == set(computed_leaves), (tag, set(reused_leaves) ^ set(computed_leaves))
        for path in reused_leaves:
            assert np.array_equal(reused_leaves[path], computed_leaves[path], equal_nan=True), (tag, path)

    # the second engine has its own memory
    first_leaves = leaves(reused.conditions.energy.converters[first.tag])
    for path, value in leaves(reused.conditions.energy.converters[second.tag]).items():
        assert not np.shares_memory(value, first_leaves[path]), path
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def operate(turboshafts):
    """One state holding the conditions of every turboshaft, at full throttle."""
    fuel_line = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    state     = setup_operating_conditions(turboshafts[0], fuel_line)
    segment   = RCAIDE.Framework.Mission.Segments.Segment()
    segment.state = state
    for turboshaft in turboshafts[1:]:
        turboshaft.append_operating_conditions(segment)
    state.expand_rows(state.conditions._size)
    for turboshaft in turboshafts:
        state.conditions.energy.converters[turboshaft.tag].throttle[:, 0] = 1.0
    return state


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
