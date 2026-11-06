# VnV/Verification/powertrain/test_ducted_fan_conditions_schema.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys

import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Powertrain import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan import compute_ducted_fan_performance, ducted_fan_conditions_schema

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Vehicles"))
from NASA_X48 import vehicle_setup

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The schema is the list of output fields and shapes a ducted fan allocates before a mission solve.
    # Both analysis methods must fill the same fields, so the method can be swapped without changing the allocation.
    for method in ["Blade_Element_Momentum_Theory", "Rankine_Froude_Momentum_Theory"]:
        check_method_fills_the_schema(method)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_method_fills_the_schema(method):
    # build the X-48 ducted fan with this method and run it at two speeds
    network     = list(vehicle_setup(True, method).networks.values())[0]
    ducted_fan  = list(network.propulsors.values())[0].ducted_fan
    distributor = RCAIDE.Library.Components.Powertrain.Distributors.Electrical_Bus()
    ducted_fan.assigned_distributors = [[distributor.tag]]
    state       = setup_operating_conditions(ducted_fan, distributor, velocity_range=np.array([30., 50.]), altitude=1000.)
    conditions  = state.conditions.energy.converters[ducted_fan.tag]
    conditions.omega[:, 0] = 6000 * Units.rpm
    compute_ducted_fan_performance(ducted_fan, state.conditions)

    # every schema field exists with its shape, for 2 control points
    schema  = ducted_fan_conditions_schema(ducted_fan)
    results = state.conditions.energy.converters[ducted_fan.tag]
    for key, shape in schema.items():
        assert key in results and np.shape(results[key]) == (2,) + shape, key

    # every method computes the basic performance outputs
    for key in ("thrust", "torque", "power", "thrust_coefficient", "efficiency"):
        assert not np.any(np.isnan(results[key])), key

    # inputs set by the propulsor before the model runs are still there afterwards
    inputs = {"orientation", "commanded_thrust_vector_angle", "throttle", "omega"}
    for key in inputs:
        assert key in results, key

    # the method wrote nothing outside the schema
    written = {key for key, value in results.items() if isinstance(value, np.ndarray) and np.ndim(value) >= 2}
    assert not written - set(schema) - inputs, written - set(schema) - inputs
    return


if __name__ == '__main__':
    main()
