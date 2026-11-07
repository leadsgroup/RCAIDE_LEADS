# RCAIDE/Library/Methods/Powertrain/Propulsors/Turboprop/compute_actuator_disk_propeller_thrust.py
#
#
# Created:  Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_OffDesign_Matching import njit

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  actuator_disk_propeller_thrust
# ----------------------------------------------------------------------------------------------------------------------
@njit(cache=True)
def actuator_disk_propeller_thrust(shaft_power, velocity, density, disk_area, polytropic_efficiency):
    """
    Thrust of a propeller modelled as an actuator disk with losses, for a given shaft power and
    flight speed (Ref. [1], Ch. 6).

    Parameters
    ----------
    shaft_power : float
        Shaft power delivered to the propeller [W].
    velocity : float
        Flight speed [m/s]; zero gives the static thrust.
    density : float
        Freestream density [kg/m^3].
    disk_area : float
        Propeller disk area [m^2].
    polytropic_efficiency : float
        Propeller polytropic efficiency, the fraction of the shaft power that becomes kinetic energy
        of the propeller stream (Ref. [1] Eq. 6.18).

    Returns
    -------
    thrust : float
        Propeller thrust [N]; zero for no shaft power.

    Notes
    -----
    Froude's theorem puts the disk velocity midway between the freestream velocity U0 and the
    far-wake velocity U1 (Ref. [1] Eq. 6.7), so the propeller mass flow is rho A (U0 + U1)/2 and the
    thrust is that mass flow times U1 - U0 (Eqs. 6.3, 6.15). The kinetic energy added to the
    stream is the polytropic efficiency times the shaft power (Eq. 6.18). With s = U0 + U1 these
    give

    .. math::
        s^3 - 2 U_0 s^2 - \\frac{4 \\eta_{pc} W_p}{\\rho A} = 0, \\qquad T = \\frac{\\rho A}{2} s (s - 2 U_0)

    whose single root above 2 U0 is found by Newton iteration from above (the cubic is convex
    there). The propeller efficiency T U0 / W_p is then the Froude efficiency 2 U0 / (U0 + U1)
    times the polytropic efficiency (Eq. 6.17): it falls with disk loading and to zero at zero
    speed, where the thrust stays finite, unlike a constant-efficiency T = eta W_p / U0.

    References
    ----------
    [1] Cantwell, B. J., "AA283 Aircraft and Rocket Propulsion", Stanford University, Ch. 6
        ("The Turboprop Cycle"), Eqs. 6.2-6.18.
    """
    if shaft_power <= 0.0:
        return 0.0
    c = 4.0 * polytropic_efficiency * shaft_power / (density * disk_area)
    s = 2.0 * velocity + c ** (1.0 / 3.0)
    for iteration in range(100):
        residual   = s * s * (s - 2.0 * velocity) - c
        derivative = 3.0 * s * s - 4.0 * velocity * s
        step       = residual / derivative
        s          = s - step
        if abs(step) < 1e-12 * s:
            break
    return 0.5 * density * disk_area * s * (s - 2.0 * velocity)

# ----------------------------------------------------------------------------------------------------------------------
#  compute_actuator_disk_propeller_thrust
# ----------------------------------------------------------------------------------------------------------------------
def compute_actuator_disk_propeller_thrust(shaft_power, velocity, density, disk_area, polytropic_efficiency):
    """
    Element-wise actuator_disk_propeller_thrust over arrays of operating points.

    Parameters
    ----------
    shaft_power, velocity, density : numpy.ndarray
        Shaft power delivered to the propeller [W], flight speed [m/s] and freestream density
        [kg/m^3], broadcast against each other.
    disk_area : float
        Propeller disk area [m^2].
    polytropic_efficiency : float
        Propeller polytropic efficiency.

    Returns
    -------
    thrust : numpy.ndarray
        Propeller thrust [N], in the broadcast shape of the inputs.
    """
    shaft_power, velocity, density = np.broadcast_arrays(np.asarray(shaft_power, dtype=float),
                                                         np.asarray(velocity, dtype=float),
                                                         np.asarray(density, dtype=float))
    thrust = np.empty(shaft_power.shape)
    for index in np.ndindex(shaft_power.shape):
        thrust[index] = actuator_disk_propeller_thrust(shaft_power[index], velocity[index], density[index],
                                                       disk_area, polytropic_efficiency)
    return thrust

# ----------------------------------------------------------------------------------------------------------------------
#  propeller_polytropic_efficiency_from_design_point
# ----------------------------------------------------------------------------------------------------------------------
def propeller_polytropic_efficiency_from_design_point(propeller_efficiency, velocity, density, disk_area, shaft_power):
    """
    Polytropic efficiency that gives a propeller its design efficiency at its design point, under
    the actuator-disk model of actuator_disk_propeller_thrust.

    Parameters
    ----------
    propeller_efficiency : float
        Design propeller efficiency, thrust power over shaft power [-].
    velocity : float
        Design flight speed [m/s].
    density : float
        Design freestream density [kg/m^3].
    disk_area : float
        Propeller disk area [m^2].
    shaft_power : float
        Design shaft power delivered to the propeller [W].

    Returns
    -------
    polytropic_efficiency : float
        The propeller efficiency divided by the Froude efficiency at the design disk loading.

    Notes
    -----
    The design thrust is propeller_efficiency * shaft_power / velocity; momentum theory gives the
    far-wake velocity for it, U1 = sqrt(U0^2 + 2 T / (rho A)), and the polytropic efficiency is the
    propeller efficiency over the Froude efficiency 2 U0 / (U0 + U1) (Cantwell, AA283, Eq. 6.17).
    """
    thrust              = propeller_efficiency * shaft_power / velocity
    far_wake_velocity   = np.sqrt(velocity ** 2 + 2.0 * thrust / (density * disk_area))
    froude_efficiency   = 2.0 * velocity / (velocity + far_wake_velocity)
    return propeller_efficiency / froude_efficiency
