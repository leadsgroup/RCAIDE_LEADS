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

from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Performance.rotor_aerodynamic_analysis import rotor_aerodynamic_analysis
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor import rotor_conditions_schema

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Vehicles"))
from Cessna_172 import vehicle_setup

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The schema is the list of output fields and shapes a rotor allocates before a mission solve.
    # Every analysis method must fill the same fields, so the method can be swapped without changing the allocation.
    # Fields a method does not compute stay NaN.
    for method in ["Blade_Element_Momentum_Theory_Helmholtz_Wake", "Actuator_Disk_Theory"]:
        check_method_fills_the_schema(method)
    check_rotor_without_blade_geometry_has_empty_distributions()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_method_fills_the_schema(method):
    # run the Cessna 172 propeller with this method at two speeds
    rotor   = propeller(method)
    results = rotor_aerodynamic_analysis(rotor, np.array([30., 50.]), angular_velocity=2400 * Units.rpm)
    schema  = rotor_conditions_schema(rotor)

    # every schema field exists with its shape, for 2 control points
    for key, shape in schema.items():
        assert key in results, key
        assert np.shape(results[key]) == (2,) + shape, key

    # every method computes the basic performance outputs
    computed = [key for key in schema if not np.all(np.isnan(results[key]))]
    for key in ("thrust", "torque", "power", "thrust_coefficient", "efficiency"):
        assert key in computed, key

    # the method wrote nothing outside the schema, apart from the inputs set by the propulsor
    inputs  = {"orientation", "commanded_thrust_vector_angle", "blade_pitch_command", "throttle", "omega"}
    written = [key for key, value in results.items() if isinstance(value, np.ndarray) and np.ndim(value) >= 2]
    unknown = set(written) - set(schema) - inputs
    assert not unknown, unknown
    return


def check_rotor_without_blade_geometry_has_empty_distributions():
    # an actuator disk needs no blade geometry, so its spanwise fields have zero length
    rotor = propeller("Actuator_Disk_Theory")
    rotor.chord_distribution = 0.0
    schema = rotor_conditions_schema(rotor)
    assert schema["blade_thrust_distribution"] == (0,)
    assert schema["disc_circulation"] == (0, rotor.number_azimuthal_stations)
    assert schema["thrust"] == (3,)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def propeller(method):
    """A copy of the Cessna 172 propeller set to use this analysis method."""
    network = list(vehicle_setup().networks.values())[0]
    rotor   = deepcopy(list(network.propulsors.values())[0].propeller)
    rotor.fidelity = method
    return rotor


if __name__ == '__main__':
    main()
