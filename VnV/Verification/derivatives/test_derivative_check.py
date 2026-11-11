# VnV/Verification/derivatives/test_derivative_check.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

from RCAIDE.Framework.Core import array_namespace

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # check_derivatives compares a function written once for NumPy and JAX against itself and against finite
    # differences. It must pass a correct function and flag a wrong derivative and a wrong value.
    try:
        import jax
    except ImportError:
        print("JAX is not installed (pip install RCAIDE[ad]); derivative checks skipped")
        return
    jax.config.update("jax_enable_x64", True)
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives

    # 16 control points in RCAIDE's (n_control_points, 1) column shape
    x = np.linspace(-1.0, 1.5, 16)[:, None]
    y = np.linspace(0.5, 2.0, 16)[:, None]

    check_correct_function_passes(check_derivatives, x, y)
    check_wrong_derivative_is_flagged(check_derivatives, x)
    check_wrong_value_is_flagged(check_derivatives, x)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_correct_function_passes(check_derivatives, x, y):
    report = check_derivatives(smooth_function, (x, y))
    assert report.passed, report
    assert report.value_error <= 1e-15 and report.compiled_error <= 1e-15
    assert report.jvp_error <= 1e-8 and report.vjp_consistency_error <= 1e-12
    return


def check_wrong_derivative_is_flagged(check_derivatives, x):
    # right values, but the JAX path cuts the gradient
    report = check_derivatives(detached_function, (x,))
    assert report.value_error <= 1e-15
    assert report.jvp_error > 0.5 and not report.passed
    return


def check_wrong_value_is_flagged(check_derivatives, x):
    # right derivative, but the JAX value is offset
    report = check_derivatives(offset_function, (x,))
    assert report.value_error > 1e-7 and report.jvp_error <= 1e-8 and not report.passed
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Test functions, each written once for both backends
# ----------------------------------------------------------------------------------------------------------------------
def smooth_function(x, y):
    xp = array_namespace(x, y)
    return xp.sin(x) * y**2 + xp.exp(0.1 * x * y), xp.tanh(x - y)


def detached_function(x):
    xp = array_namespace(x)
    if xp is np:
        return np.sin(x)
    import jax
    return xp.sin(jax.lax.stop_gradient(x))


def offset_function(x):
    xp = array_namespace(x)
    return xp.sin(x) + (0.0 if xp is np else 1e-6)


if __name__ == '__main__':
    main()
