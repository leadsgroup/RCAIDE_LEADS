# VnV/Verification/derivatives/test_core_functions_jax.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import set_at, angles_to_dcms, orientation_product, orientation_transpose

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # Core functions written once for NumPy and JAX: the array write helper, the rotation utilities, and the US 1976
    # atmosphere with its gas model. NumPy results are unchanged (the 787 benchmark is bit-identical); these checks
    # cover the JAX path: values, compiled values and derivatives against finite differences of the NumPy path.
    check_set_at_numpy_in_place()
    try:
        import jax
    except ImportError:
        print("JAX is not installed (pip install RCAIDE[ad]); JAX checks skipped")
        return
    jax.config.update("jax_enable_x64", True)
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives
    check_set_at_jax()
    check_rotations(check_derivatives)
    check_atmosphere(check_derivatives)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_set_at_numpy_in_place():
    A = np.zeros((4, 3))
    B = set_at(A, np.s_[:, 1], 2.0)
    assert B is A and np.all(A[:, 1] == 2.0)
    return


def check_set_at_jax():
    import jax.numpy as jnp
    A = jnp.zeros((4, 3))
    B = set_at(A, np.s_[:, 1], 2.0)
    assert B is not A and np.all(np.asarray(B)[:, 1] == 2.0) and np.all(np.asarray(A) == 0.0)   # JAX arrays are immutable
    C = set_at(np.zeros((4, 3)), np.s_[:, 2], jnp.ones(4))                                        # NumPy array, JAX value
    assert type(C).__module__.startswith(('jax', 'jaxlib')) and np.all(np.asarray(C)[:, 2] == 1.0)
    return


def check_rotations(check_derivatives):
    # body angles (roll, pitch, heading) at 16 control points, and a vector rotated by the transposed rotation
    rng    = np.random.default_rng(0)
    angles = rng.uniform(-0.3, 0.3, (16, 3))
    vector = rng.normal(size=(16, 3))
    def rotate(angles, vector):
        return orientation_product(orientation_transpose(angles_to_dcms(angles, (2, 1, 0))), vector)
    report = check_derivatives(rotate, (angles, vector))
    assert report.passed, report
    return


def check_atmosphere(check_derivatives):
    # altitudes across the troposphere, the isothermal layer and the layers above, kept off the layer breaks where
    # the temperature profile has a kink
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    altitude   = np.array([500., 3000., 6000., 9000., 10500., 12000., 15000., 19000., 21000., 25000., 30000.,
                           35000., 40000., 45000., 50000., 60000.])[:, None]
    def properties(altitude):
        data = atmosphere.compute_values(altitude, 10.0)
        return (data.pressure, data.temperature, data.density, data.speed_of_sound, data.dynamic_viscosity,
                data.thermal_conductivity, data.prandtl_number, data.constant_pressure_specific_heat)
    report = check_derivatives(properties, (altitude,), steps=(1e-4, 1e-5, 1e-6, 1e-7))
    assert report.passed, report
    return


if __name__ == '__main__':
    main()
