# _numpy_bridge.py
# NumPy-only helpers for SciPy solver callbacks (outside JAX/Torch AD tracing).

import numpy as np


def to_numpy(x):
    """Cast any array-like to a plain NumPy ndarray (bypasses RNUMPY in-place guards)."""
    try:
        import torch as tr
        if isinstance(x, tr.Tensor):
            try:
                return x.detach().cpu().numpy()
            except RuntimeError:
                return np.asarray(x.detach().cpu().tolist(), dtype=float)
    except ImportError:
        pass
    if isinstance(x, np.ndarray) and type(x) is np.ndarray:
        return x
    return np.asarray(x)


def finite_diff_jacobian(fun, x, eps=None):
    """Central-difference Jacobian of ``fun`` at ``x`` using plain NumPy arrays."""
    x = to_numpy(x).astype(float, copy=False)
    if eps is None:
        eps = max(1e-5, np.finfo(float).eps ** 0.5)
    f0 = to_numpy(fun(x)).ravel()
    n = x.size
    m = f0.size
    jac = np.empty((m, n), dtype=float)
    for i in range(n):
        xp = x.copy()
        xm = x.copy()
        xp[i] += eps
        xm[i] -= eps
        jac[:, i] = (to_numpy(fun(xp)).ravel() - to_numpy(fun(xm)).ravel()) / (2.0 * eps)
    return jac
