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
from RCAIDE.Framework.Core import array_namespace

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

    whose single real root (above 2 U0) is taken in closed form: with s = t + 2 U0/3 the cubic is
    t^3 + p t + q = 0 with p = -4 U0^2/3, q = -16 U0^3/27 - c, whose discriminant
    (q/2)^2 + (p/3)^3 = 8 U0^3 c/27 + c^2/4 is positive (Ref. [2]); t = u + 4 U0^2/(9 u) with
    u = cbrt(-q/2 + sqrt(discriminant)) avoids cancellation. The thrust is then rho A c/(2 s). The propeller efficiency T U0 / W_p is then the Froude efficiency 2 U0 / (U0 + U1)
    times the polytropic efficiency (Eq. 6.17): it falls with disk loading and to zero at zero
    speed, where the thrust stays finite, unlike a constant-efficiency T = eta W_p / U0.

    References
    ----------
    [1] Cantwell, B. J., "AA283 Aircraft and Rocket Propulsion", Stanford University, Ch. 6
        ("The Turboprop Cycle"), Eqs. 6.2-6.18.
    [2] Abramowitz, M. and Stegun, I. A., "Handbook of Mathematical Functions", Dover, 1965, Sec. 3.8.2
        (solution of cubic equations).
    """
    if shaft_power <= 0.0:
        return 0.0
    return actuator_disk_thrust_closed_form_kernel(shaft_power, velocity, density, disk_area, polytropic_efficiency)

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

    Notes
    -----
    The closed-form root needs no iteration, so the same expression runs on NumPy and JAX and differentiates
    directly.
    """
    xp      = array_namespace(shaft_power, velocity, density)
    powered = shaft_power > 0.0
    thrust  = actuator_disk_thrust_closed_form(xp.where(powered, shaft_power, 1.0), velocity, density, disk_area,
                                               polytropic_efficiency)
    return xp.where(powered, thrust, 0.0)

def actuator_disk_thrust_closed_form(shaft_power, velocity, density, disk_area, polytropic_efficiency):
    """
    Actuator disk thrust at positive shaft power from the closed-form root of its cubic (see
    actuator_disk_propeller_thrust), for arrays on NumPy or JAX.
    """
    xp = array_namespace(shaft_power, velocity, density)
    c  = 4.0 * polytropic_efficiency * shaft_power / (density * disk_area)
    u  = xp.cbrt(8.0 * velocity * velocity * velocity / 27.0 + 0.5 * c + xp.sqrt(8.0 * velocity * velocity * velocity * c / 27.0 + 0.25 * c * c))
    s  = u + 4.0 * velocity * velocity / (9.0 * u) + 2.0 * velocity / 3.0
    return 0.5 * density * disk_area * c / s

@njit(cache=True)
def actuator_disk_thrust_closed_form_kernel(shaft_power, velocity, density, disk_area, polytropic_efficiency):
    """actuator_disk_thrust_closed_form on floats, compiled for the numba matching kernel."""
    c  = 4.0 * polytropic_efficiency * shaft_power / (density * disk_area)
    u  = np.cbrt(8.0 * velocity * velocity * velocity / 27.0 + 0.5 * c + np.sqrt(8.0 * velocity * velocity * velocity * c / 27.0 + 0.25 * c * c))
    s  = u + 4.0 * velocity * velocity / (9.0 * u) + 2.0 * velocity / 3.0
    return 0.5 * density * disk_area * c / s

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
