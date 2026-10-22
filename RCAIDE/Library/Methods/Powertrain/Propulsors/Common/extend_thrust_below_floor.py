# RCAIDE/Library/Methods/Powertrain/Propulsors/Common/extend_thrust_below_floor.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  extend_thrust_below_floor
# ----------------------------------------------------------------------------------------------------------------------
def extend_thrust_below_floor(floor_thrust, floor_slope, throttle, floor_fraction):
    """
    Thrust below the matching floor. It decays from the floor thrust toward zero, matching the floor thrust
    and slope at the floor, so it is smooth and never negative and the mission solver keeps a gradient.
    A non-positive floor thrust or slope is held.

    Parameters
    ----------
    floor_thrust : float
        Thrust at the matching floor [N].
    floor_slope : float
        Thrust slope with throttle just above the floor [N].
    throttle : float
        Requested combustor exit temperature fraction [-].
    floor_fraction : float
        Matching floor combustor exit temperature fraction [-].

    Returns
    -------
    thrust : float
        Thrust below the floor [N].
    """
    if floor_thrust <= 0.0 or floor_slope <= 0.0:
        return floor_thrust
    return floor_thrust * np.exp(floor_slope * (throttle - floor_fraction) / floor_thrust)
