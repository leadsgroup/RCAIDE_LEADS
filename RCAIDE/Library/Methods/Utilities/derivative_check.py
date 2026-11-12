# RCAIDE/Library/Methods/Utilities/derivative_check.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

from RCAIDE.Framework.Core import Data, array_namespace

# ----------------------------------------------------------------------------------------------------------------------
#  check_derivatives
# ----------------------------------------------------------------------------------------------------------------------
def check_derivatives(function, inputs, steps=(1e-4, 1e-5, 1e-6), n_directions=3, value_tolerance=1e-12,
                      derivative_tolerance=1e-6, seed=0):
    """Checks a function written once for NumPy and JAX against itself and against finite differences.

    Parameters
    ----------
    function : callable
        ``function(*arrays)`` returning an array or a tuple of arrays. It reads its backend from its inputs with
        ``array_namespace``, so NumPy inputs run NumPy and JAX inputs run jax.numpy.
    inputs : tuple of numpy.ndarray
        Point at which the function is checked.
    steps : tuple of float
        Relative finite-difference steps, scaled for each input element by max(1, |x|), so inputs of different
        magnitude (a temperature and a throttle) each get a step suited to them; the smallest error over the steps
        is compared with the tolerance, so truncation (large steps) and round-off (small steps) both show.
    n_directions : int
        Number of random input directions for the forward and reverse mode checks.
    value_tolerance : float
        Largest relative difference allowed between NumPy, JAX and compiled JAX values.
    derivative_tolerance : float
        Largest relative difference allowed between the JAX directional derivative and central finite differences
        of the NumPy path, and between u . (J v) from forward mode and (J^T u) . v from reverse mode.
    seed : int
        Seed for the random directions.

    Returns
    -------
    report : Data
        value_error, compiled_error, jvp_errors (per direction, per step), jvp_error, vjp_consistency_error, passed.

    Notes
    -----
    Finite differences use the NumPy path, so a JAX path that detaches from or diverges from the NumPy path is
    caught. Needs the optional dependency ``pip install RCAIDE[ad]`` and 64-bit JAX
    (``jax.config.update("jax_enable_x64", True)``).
    """
    try:
        import jax
        import jax.numpy as jnp
    except ImportError as error:
        raise ImportError("check_derivatives needs JAX: pip install RCAIDE[ad]") from error
    if not jax.config.jax_enable_x64:
        raise ValueError('check_derivatives needs 64-bit JAX: jax.config.update("jax_enable_x64", True)')

    rng      = np.random.default_rng(seed)
    x_np     = tuple(np.asarray(x, dtype=np.float64) for x in inputs)
    x_jax    = tuple(jnp.asarray(x) for x in x_np)
    flat     = lambda *x: flatten(function(*x))

    report                = Data()
    y_np                  = flat(*x_np)
    y_jax                 = np.asarray(flat(*x_jax))
    report.value_error    = relative_error(y_jax, y_np)
    report.compiled_error = relative_error(np.asarray(jax.jit(flat)(*x_jax)), y_np)

    jvp_errors = []
    vjp_errors = []
    for _ in range(n_directions):
        # random direction, each element scaled by the size of its input
        v      = tuple(rng.standard_normal(x.shape) * np.maximum(1.0, np.abs(x)) for x in x_np)
        u      = rng.standard_normal(y_np.shape)
        jv     = np.asarray(jax.jvp(flat, x_jax, tuple(jnp.asarray(d) for d in v))[1])
        errors = []
        for h in steps:
            plus  = flat(*(x + h * d for x, d in zip(x_np, v)))
            minus = flat(*(x - h * d for x, d in zip(x_np, v)))
            errors.append(relative_error(jv, (plus - minus) / (2 * h)))
        jvp_errors.append(errors)

        y, pullback = jax.vjp(flat, *x_jax)
        jtu         = pullback(jnp.asarray(u))
        lhs         = float(np.dot(u, jv))
        rhs         = float(sum(np.sum(np.asarray(g) * d) for g, d in zip(jtu, v)))
        vjp_errors.append(abs(lhs - rhs) / max(abs(lhs), abs(rhs), 1e-300))

    report.jvp_errors            = jvp_errors
    report.jvp_error             = max(min(e) for e in jvp_errors)
    report.vjp_consistency_error = max(vjp_errors)
    report.passed = bool(report.value_error <= value_tolerance and report.compiled_error <= value_tolerance
                         and report.jvp_error <= derivative_tolerance
                         and report.vjp_consistency_error <= derivative_tolerance)
    return report


def flatten(outputs):
    """Concatenates a function's outputs into one vector."""
    if not isinstance(outputs, (tuple, list)):
        outputs = (outputs,)
    xp = array_namespace(*outputs)
    return xp.concatenate([xp.reshape(y, (-1,)) for y in outputs])


def relative_error(a, b):
    """Largest absolute difference relative to the largest magnitude of b."""
    return float(np.max(np.abs(a - b)) / max(np.max(np.abs(b)), 1e-300))
