# RCAIDE/Library/Methods/Powertrain/Converters/EHD_Electrode_Array/compute_peek_inception.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# Peek reference state: 25 degC and 76 cmHg (spec 3.3 step 1, [VERIFY] against Peek 1929)
PEEK_REFERENCE_PRESSURE    = 101325.0  # Pa, 76 cmHg
PEEK_REFERENCE_TEMPERATURE = 298.15    # K, 25 degC

# ----------------------------------------------------------------------------------------------------------------------
#  compute_relative_air_density
# ----------------------------------------------------------------------------------------------------------------------
def compute_relative_air_density(pressure, temperature):
    """
    Computes the relative air density used in Peek's law (R19).

    Parameters
    ----------
    pressure : float or numpy.ndarray
        Freestream static pressure [Pa]
    temperature : float or numpy.ndarray
        Freestream static temperature [K]

    Returns
    -------
    delta : float or numpy.ndarray
        Relative air density, delta = (p/p_0)(T_0/T) [-]

    Notes
    -----
    The reference state (p_0 = 76 cmHg, T_0 = 25 degC) is the one used in the secondary
    statement of Peek's law quoted in the spec (step 1 of Part 3.3); the spec flags it [VERIFY].

    References
    ----------
    [1] Peek, F. W., Dielectric Phenomena in High Voltage Engineering, McGraw-Hill, 1929.
    """
    return (pressure / PEEK_REFERENCE_PRESSURE) * (PEEK_REFERENCE_TEMPERATURE / temperature)

# ----------------------------------------------------------------------------------------------------------------------
#  compute_peek_inception_field
# ----------------------------------------------------------------------------------------------------------------------
def compute_peek_inception_field(wire_radius, relative_air_density=1.0, surface_factor=1.0):
    """
    Computes the corona inception field at the emitter-wire surface with Peek's law (R19).

    Parameters
    ----------
    wire_radius : float
        Emitter wire radius a [m]
    relative_air_density : float or numpy.ndarray, optional
        Relative air density delta [-], default 1.0
    surface_factor : float, optional
        Peek surface factor m [-] (1 smooth, 0.93-0.98 weathered), default 1.0

    Returns
    -------
    E_i : float or numpy.ndarray
        Inception field at the wire surface [V/m]

    Notes
    -----
    Evaluated in Peek's textbook form E_i = 30·m·delta·(1 + 0.301/sqrt(delta·r_cm)) kV/cm and
    converted to V/m inside this function only. This is identical to the SI form
    E_i = 3.0e6·m·delta·(1 + 0.0301/sqrt(delta·a)) V/m.

    **Major Assumptions**
        * Empirical AC visual-corona constants; DC positive-corona inception on a real electrode
          array should be measured (R19 validity)

    References
    ----------
    [1] Peek, F. W., Dielectric Phenomena in High Voltage Engineering, McGraw-Hill, 1929;
        restated in Kuffel & Zaengl, High Voltage Engineering Fundamentals, and arXiv 1011.1393.
    """
    r_cm         = wire_radius * 100.0
    E_i_kV_cm    = 30.0 * surface_factor * relative_air_density * (1.0 + 0.301 / np.sqrt(relative_air_density * r_cm))
    kV_cm_to_V_m = 1.0e5
    return E_i_kV_cm * kV_cm_to_V_m

# ----------------------------------------------------------------------------------------------------------------------
#  compute_inception_voltage
# ----------------------------------------------------------------------------------------------------------------------
def compute_inception_voltage(inception_field, wire_radius, gap, k_Vi=1.0):
    """
    Estimates the corona inception voltage from the inception field (R20).

    Parameters
    ----------
    inception_field : float or numpy.ndarray
        Inception field at the wire surface E_i [V/m] (R19)
    wire_radius : float
        Emitter wire radius a [m]
    gap : float
        Wire centre to collector leading edge d [m]
    k_Vi : float, optional
        Inception calibration multiplier [-], default 1.0

    Returns
    -------
    V_i : float or numpy.ndarray
        Inception voltage [V], V_i = k_Vi·E_i·a·ln(d/a)

    Notes
    -----
    Approximation, not a sourced wire-to-airfoil law: it borrows the coaxial-cylinder potential with
    the outer radius replaced by the gap. It ignores the airfoil shape and neighbour shielding.
    Override with a measured inception voltage whenever one is available.

    References
    ----------
    [1] Coaxial electrostatics, e.g. arXiv 2511.11401 Eq. 3.1 (spec R20).
    """
    return k_Vi * inception_field * wire_radius * np.log(gap / wire_radius)
