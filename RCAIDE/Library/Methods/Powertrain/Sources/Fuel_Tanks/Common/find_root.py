# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Common/find_root.py
#
# Created: Sep 2026, RCAIDE Team

import numpy as np
from scipy.optimize import brentq, minimize_scalar

# ----------------------------------------------------------------------------------------------------------------------
#  Bounded 1D root finder
#
#  Tries brentq first (fast, guaranteed convergence when a sign change exists).
#  Falls back to minimizing f(x)^2 with minimize_scalar when brentq fails
#  (no sign change in the bracket, e.g. the root is a tangent zero).
# ----------------------------------------------------------------------------------------------------------------------
def _find_root(func, a, b, args=(), xtol=1e-9):
    try:
        return brentq(func, a, b, xtol=xtol, args=args)
    except ValueError:
        res = minimize_scalar(lambda x: func(x, *args)**2,
                              bounds=(a, b), method="bounded",
                              options={"xatol": xtol})
        return float(res.x)


# ----------------------------------------------------------------------------------------------------------------------
#  Bracket a root by geometric expansion of the search interval
# ----------------------------------------------------------------------------------------------------------------------
def _bracket_root(func, start=1e-6, factor=10, limit=1e2, args=()):
    a  = start
    fa = func(a, *args)
    b  = a * factor
    fb = func(b, *args)
    while np.sign(fa) == np.sign(fb) and b < limit:
        a, fa = b, fb
        b    *= factor
        fb    = func(b, *args)
    if np.sign(fa) == np.sign(fb):
        return None
    return a, b
