# RCAIDE/Library/Methods/Powertrain/Propulsors/Common/extend_thrust_below_floor.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import array_namespace

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
    floor_thrust : float or array
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
    xp       = array_namespace(floor_thrust, floor_slope, throttle, floor_fraction)
    span     = floor_thrust - lower_bound
    usable   = (span > 0.0) & (floor_slope > 0.0)
    span     = xp.where(usable, span, 1.0)
    extended = lower_bound + span * xp.exp(xp.where(usable, floor_slope, 0.0) * (throttle - floor_fraction) / span)
    return xp.where(usable, extended, floor_thrust)
