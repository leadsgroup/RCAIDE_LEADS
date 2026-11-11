# RCAIDE/Library/Methods/Powertrain/reset_power_conditions.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import set_at
import numpy as np

POWER_TYPES = ('propulsive', 'mechanical', 'electrical', 'chemical', 'pneumatic', 'hydraulic', 'thermal')

# ----------------------------------------------------------------------------------------------------------------------
#  reset_power_conditions
# ----------------------------------------------------------------------------------------------------------------------
def reset_power_conditions(conditions):
    """
    Sets every input and output power of a component's conditions to zero, at every control point, before the
    network sums the powers of a new evaluation.

    Inputs:
       conditions   conditions of a component (or of the network), with inputs.power and outputs.power   [-]

    Outputs:
       None; the powers are reset in conditions
    """
    for power in (conditions.inputs.power, conditions.outputs.power):
        for name in POWER_TYPES:
            power[name] = set_at(power[name], np.s_[:, 0], 0.0)
    return
