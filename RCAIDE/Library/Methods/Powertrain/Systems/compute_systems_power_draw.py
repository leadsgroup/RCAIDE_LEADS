# RCAIDE/Library/Methods/Powertrain/Systems/compute_systems_power_draw.py
#
# Created:  Jul 2024, RCAIDE Team

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import set_at
import numpy as np

def compute_systems_power_draw(system, state, vehicle):
    """Computes the power draw of a generic system.

    Parameters
    ----------
    system : System
        The system component with power_draw attribute [W].
    state : State
        Mission segment state containing conditions.
    vehicle : Vehicle
        The aircraft vehicle (available for systems that need vehicle-level data).

    Returns
    -------
    inputs : Conditions
        Input power conditions for the system.
    outputs : Conditions
        Output power conditions for the system.
    """
    system_conditions                               = state.conditions.energy.systems[system.tag]
    system_conditions.inputs.power.electrical  = set_at(system_conditions.inputs.power.electrical, np.s_[:,0], system.power_draw)
    system_conditions.outputs.power.electrical = set_at(system_conditions.outputs.power.electrical, np.s_[:,0], 0.0)

    return system_conditions.inputs, system_conditions.outputs
