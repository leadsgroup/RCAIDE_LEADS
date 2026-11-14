# RCAIDE/Library/Methods/Powertrain/Propulsors/Common/pack_fields.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from RCAIDE.Framework.Core import array_namespace

# ----------------------------------------------------------------------------------------------------------------------
#  pack_fields
# ----------------------------------------------------------------------------------------------------------------------
def pack_fields(values):
    """Scalars as one float array, on the backend they are on (a JAX array when the design is a JAX input)."""
    xp = array_namespace(*values)
    if xp is np:
        return np.array([float(value) for value in values])
    return xp.stack([xp.reshape(xp.asarray(value, dtype=float), ()) for value in values])
