# RCAIDE/Library/Methods/Utilities/derivative_check.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

from RCAIDE.Framework.Core import Data

# ----------------------------------------------------------------------------------------------------------------------
#  check_derivatives
# ----------------------------------------------------------------------------------------------------------------------
def check_derivatives(kernel, inputs, steps=(1e-4, 1e-5, 1e-6), n_directions=3, value_tolerance=1e-12,
                      derivative_tolerance=1e-6, check_graph_breaks=True, seed=0):
    """Checks a pure kernel's NumPy and Torch values and its Torch derivatives against central finite differences.

    Parameters
    ----------
    kernel : callable
        Pure function ``kernel(*arrays, xp=module)`` returning an array or a tuple of arrays, written once for
        ``xp=numpy`` and ``xp=torch``.
    inputs : tuple of numpy.ndarray
        Point at which the kernel is checked.
    steps : tuple of float
        Relative finite-difference steps; the smallest error over the steps is compared with the tolerance, so
        truncation (large steps) and round-off (small steps) can both be seen in the report.
    n_directions : int
        Number of random input directions for the JVP and VJP checks.
    value_tolerance : float
        Largest relative difference allowed between the NumPy and Torch values.
    derivative_tolerance : float
        Largest relative difference allowed between the JVP and central finite differences, and between
        u . (J v) from the JVP and (J^T u) . v from the VJP.
    check_graph_breaks : bool
        Whether ``torch._dynamo.explain`` must report no graph breaks, so the kernel can compile as one graph.
    seed : int
        Seed for the random directions.

    Returns
    -------
    report : Data
        value_error, jvp_errors (per direction, per step), jvp_error, vjp_consistency_error, graph_breaks and
        passed.

    Notes
    -----
    Derivatives come from ``torch.func.jvp`` and ``torch.func.vjp``; finite differences use the NumPy path, so a
    kernel whose Torch path detaches or diverges from the NumPy path is caught. Requires the optional ``ad``
    dependencies (``pip install RCAIDE[ad]``).
    """
    try:
        import torch
    except ImportError as error:  # pragma: no cover -- torch is optional (RCAIDE[ad]) and CI installs it
        raise ImportError("check_derivatives needs torch: pip install RCAIDE[ad]") from error

    rng      = np.random.default_rng(seed)
    x_np     = tuple(np.asarray(x, dtype=np.float64) for x in inputs)
    x_torch  = tuple(torch.as_tensor(x) for x in x_np)
    numpy_f  = lambda *x: flatten(kernel(*x, xp=np), np)
    torch_f  = lambda *x: flatten(kernel(*x, xp=torch), torch)

    report             = Data()
    y_np               = numpy_f(*x_np)
    y_torch            = torch_f(*x_torch).detach().numpy()
    report.value_error = relative_error(y_torch, y_np)

    jvp_errors  = []
    vjp_errors  = []
    for _ in range(n_directions):
        v       = tuple(rng.standard_normal(x.shape) for x in x_np)
        u       = rng.standard_normal(y_np.shape)
        jv      = torch.func.jvp(torch_f, x_torch, tuple(torch.as_tensor(d) for d in v))[1].detach().numpy()
        scale   = max(1.0, max(np.max(np.abs(x)) for x in x_np))
        errors  = []
        for h in steps:
            step = h * scale
            plus  = numpy_f(*(x + step * d for x, d in zip(x_np, v)))
            minus = numpy_f(*(x - step * d for x, d in zip(x_np, v)))
            errors.append(relative_error(jv, (plus - minus) / (2 * step)))
        jvp_errors.append(errors)

        y, pullback = torch.func.vjp(torch_f, *x_torch)
        jtu         = pullback(torch.as_tensor(u))
        lhs         = float(np.dot(u, jv))
        rhs         = float(sum(np.sum(g.detach().numpy() * d) for g, d in zip(jtu, v)))
        vjp_errors.append(abs(lhs - rhs) / max(abs(lhs), abs(rhs), 1e-300))

    report.jvp_errors            = jvp_errors
    report.jvp_error             = max(min(e) for e in jvp_errors)
    report.vjp_consistency_error = max(vjp_errors)

    report.graph_breaks = None
    if check_graph_breaks:
        torch._dynamo.reset()
        report.graph_breaks = torch._dynamo.explain(torch_f)(*x_torch).graph_break_count
        torch._dynamo.reset()

    report.passed = bool(report.value_error <= value_tolerance
                         and report.jvp_error <= derivative_tolerance
                         and report.vjp_consistency_error <= derivative_tolerance
                         and (report.graph_breaks is None or report.graph_breaks == 0))
    return report


def flatten(outputs, xp):
    """Concatenates a kernel's outputs into one vector."""
    if not isinstance(outputs, (tuple, list)):
        outputs = (outputs,)
    return xp.concatenate([xp.reshape(y, (-1,)) for y in outputs])


def relative_error(a, b):
    """Largest absolute difference relative to the largest magnitude of b."""
    return float(np.max(np.abs(a - b)) / max(np.max(np.abs(b)), 1e-300))
