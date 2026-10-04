# RCAIDE/Library/Methods/Powertrain/Converters/EHD_Electrode_Array/compute_space_charge_performance.py
#
# Created:  Oct 2026, RCAIDE EHD MVP

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from scipy.constants import epsilon_0

# ----------------------------------------------------------------------------------------------------------------------
#  compute_dimensionless_current
# ----------------------------------------------------------------------------------------------------------------------
def compute_dimensionless_current(normalized_voltage):
    """
    Computes the dimensionless 1-D space-charge current with an inception boundary condition (R21).

    Parameters
    ----------
    normalized_voltage : float or numpy.ndarray
        V_hat = V_a / V_i [-]

    Returns
    -------
    j_hat : numpy.ndarray
        Dimensionless current [-]:
        j_hat = [9 - 12·V_hat^-2 + sqrt((3 - 2·V_hat^-1)^3·(3 + 6·V_hat^-1))] / 16 for V_hat > 1,
        else 0

    Notes
    -----
    Limits: j_hat ~ 2(V_hat - 1) near inception, j_hat -> 9/8 (Mott-Gurney) as V_hat -> inf.
    j_hat is continuous (zero) at V_hat = 1.

    **Major Assumptions**
        * 1-D planar drift region, Kaptsov/Peek boundary condition at the emitter, no diffusion, still air

    References
    ----------
    [1] Kahol, Belan, Pacchiani & Montenero, J. Electrostatics 123, 103815 (2023), Appendix
        Eqs. (A.7)-(A.9).
    """
    V_hat  = np.asarray(normalized_voltage, dtype=float)
    j_hat  = np.zeros_like(V_hat)
    above  = V_hat > 1.0
    x      = 1.0 / V_hat[above]
    j_hat[above] = (9.0 - 12.0 * x**2 + np.sqrt((3.0 - 2.0 * x)**3 * (3.0 + 6.0 * x))) / 16.0
    return j_hat

# ----------------------------------------------------------------------------------------------------------------------
#  compute_current_density
# ----------------------------------------------------------------------------------------------------------------------
def compute_current_density(applied_voltage, gap, ion_mobility, dimensionless_current):
    """
    Converts the dimensionless current to a current density (R22).

    Parameters
    ----------
    applied_voltage : float or numpy.ndarray
        V_a [V]
    gap : float
        d [m]
    ion_mobility : float
        mu [m^2/(V·s)]
    dimensionless_current : float or numpy.ndarray
        j_hat [-] (R21)

    Returns
    -------
    j : float or numpy.ndarray
        Current density [A/m^2], j = eps_0·mu·V_a^2/d^3·j_hat

    References
    ----------
    [1] Kahol et al., J. Electrostatics 123, 103815 (2023), Eq. (10).
    """
    return epsilon_0 * ion_mobility * applied_voltage**2 / gap**3 * dimensionless_current

# ----------------------------------------------------------------------------------------------------------------------
#  compute_unit_thrust_and_power
# ----------------------------------------------------------------------------------------------------------------------
def compute_unit_thrust_and_power(applied_voltage, gap, ion_mobility, dimensionless_current,
                                  unit_spacing, span, k_T=1.0, k_P=1.0):
    """
    Computes electrical thrust, power and current of one wire-to-collector unit treated as a 1-D
    planar cell of frontal area S·b (spec 2.4 derived equations; combines R1, R21, R22).

    Parameters
    ----------
    applied_voltage : float or numpy.ndarray
        V_a [V]
    gap : float
        d [m]
    ion_mobility : float
        mu [m^2/(V·s)]
    dimensionless_current : float or numpy.ndarray
        j_hat [-] (R21)
    unit_spacing : float
        S [m]
    span : float
        b [m]
    k_T, k_P : float, optional
        Thrust and power calibration multipliers [-], default 1.0

    Returns
    -------
    T_unit : float or numpy.ndarray
        Electrical thrust per unit [N], k_T·eps_0·(V_a/d)^2·j_hat·S·b
    P_unit : float or numpy.ndarray
        Electrical power per unit [W], k_P·eps_0·mu·V_a^3/d^3·j_hat·S·b
    I_unit : float or numpy.ndarray
        Current per unit [A], P_unit/V_a (zero where V_a = 0)

    Notes
    -----
    With k_T = k_P = 1 the ratio T_unit/P_unit = d/(mu·V_a) reproduces R2 exactly. The 1-D cell
    spreads current evenly over S, so it over-predicts absolute thrust and current until k_T and k_P
    are calibrated (Stage 2). Thrust per unit is proportional to S here, which is the wrong spacing
    trend (spec 3.3.2); unit_spacing must stay a fixed design value.

    **Theory**
    T = I·d/mu (R1) with I = j·S·b (R22) gives T_unit; P = I·V_a gives P_unit.

    References
    ----------
    [1] Masuyama & Barrett, Proc. R. Soc. A 469, 20120623 (2013) (R1, R2).
    [2] Kahol et al., J. Electrostatics 123, 103815 (2023) (R21, R22, R23).
    """
    frontal_area = unit_spacing * span
    T_unit = k_T * epsilon_0 * (applied_voltage / gap)**2 * dimensionless_current * frontal_area
    P_unit = k_P * epsilon_0 * ion_mobility * applied_voltage**3 / gap**3 * dimensionless_current * frontal_area
    V_a    = np.asarray(applied_voltage, dtype=float)
    I_unit = np.divide(P_unit, V_a, out=np.zeros_like(np.asarray(P_unit, dtype=float)), where=V_a > 0)
    return T_unit, P_unit, I_unit
