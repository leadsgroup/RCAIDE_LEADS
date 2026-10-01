# RCAIDE/Library/Methods/Powertrain/Converters/Fan/compute_fan_angular_velocity.py
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
# Imports
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_fan_angular_velocity
# ----------------------------------------------------------------------------------------------------------------------
def compute_fan_angular_velocity(fan, total_temperature_rise):
    """
    Computes the operating angular velocity of a fan from the total temperature rise across it,
    scaled from the fan's design point.

    Parameters
    ----------
    fan : RCAIDE.Library.Components.Powertrain.Converters.Fan
        Fan component with the following attributes:
            - design_angular_velocity : float
                Fan angular velocity at the design point [rad/s]
            - design_total_temperature_rise : float
                Total temperature rise across the fan at the design point [K], set by design_turbofan
    total_temperature_rise : numpy.ndarray
        Total temperature rise across the fan at each operating point [K]

    Returns
    -------
    angular_velocity : numpy.ndarray
        Fan angular velocity at each operating point [rad/s]

    Notes
    -----
    The total enthalpy rise across a fan is proportional to the square of its rotational speed
    (Euler turbomachinery equation at constant work coefficient), so for a calorically perfect gas

    .. math::
        \\frac{N}{N_R} = \\sqrt{\\frac{\\Delta T_t}{\\Delta T_{t,R}}}

    which is the physical-speed form of Ref. [1] Eq. (8-92a). This ties fan speed to the cycle
    solution: with fixed component pressure ratios (analytical cycle) it varies only with flight
    condition, while the off-design matching solution also captures its variation with throttle.

    References
    ----------
    [1] Mattingly, J. D., "Elements of Gas Turbine Propulsion", McGraw-Hill, 1996, Sec. 8.7, Eq. (8-92a).
    """
    if fan.design_total_temperature_rise <= 0:
        raise ValueError(f"Fan '{fan.tag}' has no design_total_temperature_rise; the turbofan must be sized "
                         "with design_turbofan before its fan angular velocity can be computed.")
    speed_ratio      = np.sqrt(np.maximum(total_temperature_rise, 0.0) / fan.design_total_temperature_rise)
    angular_velocity = fan.design_angular_velocity * speed_ratio
    return angular_velocity
