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

COMPONENTS = ["ram", "inlet_nozzle", "compressor", "combustor", "high_pressure_turbine", "low_pressure_turbine",
              "core_nozzle"]


def twin():
    """A designed turboshaft and an identical copy with its own tags."""
    first  = turboshaft_Boeing_502_14()
    second = deepcopy(first)
    second.tag = first.tag + "_2"
    for name in COMPONENTS:
        second[name].tag = first[name].tag + "_2"
    return first, second


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


def leaves(D, prefix=""):
    out = {}
    for k, v in D.items():
        path = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            out.update(leaves(v, path))
        elif isinstance(v, np.ndarray):
            out[path] = v
    return out


def test_reused_turboshaft_equals_computing_it():
    first, second = twin()
    network            = Data()
    network.converters = Container()
    network.converters.append(first)
    network.converters.append(second)

    reused = operate([first, second])
    compute_turboshaft_performance(first, reused.conditions)
    second.reuse_stored_data(reused, network, stored_converter_tag=first.tag)

    computed = operate([second])
    compute_turboshaft_performance(second, computed.conditions)

    for tag in [second.tag] + [second[name].tag for name in COMPONENTS]:
        a, b = leaves(reused.conditions.energy.converters[tag]), leaves(computed.conditions.energy.converters[tag])
        assert set(a) == set(b), (tag, set(a) ^ set(b))
        for path in a:
            assert np.array_equal(a[path], b[path], equal_nan=True), (tag, path)
    first_leaves = leaves(reused.conditions.energy.converters[first.tag])
    for path, value in leaves(reused.conditions.energy.converters[second.tag]).items():
        assert not np.shares_memory(value, first_leaves[path]), path
