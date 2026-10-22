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
def extend_thrust_below_floor(floor_thrust, floor_slope, throttle, floor_fraction, lower_bound=0.0):
    """
    Thrust below the matching floor. It decays from the floor thrust toward a lower bound, matching the floor
    thrust and slope at the floor, so it is smooth, bounded and keeps a gradient for the mission solver.
    A floor thrust at or below the lower bound, or a non-positive slope, is held.

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
    lower_bound : float, optional
        Thrust approached far below the floor [N]: zero, or the ram drag -mdot*V0 of an engine with no gross
        thrust in flight.

    Returns
    -------
    thrust : float
        Thrust below the floor [N].
    """
    span = floor_thrust - lower_bound
    if span <= 0.0 or floor_slope <= 0.0:
        return floor_thrust
    return lower_bound + span * np.exp(floor_slope * (throttle - floor_fraction) / span)
