# RCAIDE/Library/Methods/Stability/Common/lift_carries_weight.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# assumption: aerodynamic lift carries at least this share of the weight for the small-disturbance model to apply
MINIMUM_LIFT_FRACTION = 0.9

# ----------------------------------------------------------------------------------------------------------------------
#  lift_carries_weight
# ----------------------------------------------------------------------------------------------------------------------
def lift_carries_weight(conditions, vehicle):
    """
    True at control points where aerodynamic lift carries the weight. The stability derivatives have no rotor
    contributions and Etkin & Reid's equations assume steady flight with thrust balancing drag, so a rotor-borne
    vehicle (hover, eVTOL transition) is outside the model. With no weight defined, every point counts as supported.

    Parameters
    ----------
    conditions : Conditions
    vehicle : Vehicle

    Returns
    -------
    supported : numpy.ndarray of bool, shape (ctrl_pts, 1)
    """
    lift   = conditions.aerodynamics.coefficients.lift.total * conditions.freestream.dynamic_pressure * vehicle.reference_area
    weight = conditions.weights.vehicle.mass * conditions.freestream.gravity
    return (weight <= 0) | (lift >= MINIMUM_LIFT_FRACTION * weight)
