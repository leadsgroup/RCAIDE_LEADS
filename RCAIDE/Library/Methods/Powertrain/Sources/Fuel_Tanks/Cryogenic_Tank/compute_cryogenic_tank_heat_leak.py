# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank/compute_cryogenic_tank_heat_leak.py
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from scipy.optimize import brentq, minimize_scalar

# ----------------------------------------------------------------------------------------------------------------------
#  Cryogenic Tank Environmental Heat Leak
# ----------------------------------------------------------------------------------------------------------------------
def compute_cryogenic_tank_heat_leak(t_ins, T_env, T_cold, k_mat, k_ins_mat, k_air, nu, alpha_th, Pr, ro, ri, li):
    """
    Heat leak through a rounded-end cylindrical tank's structural wall + insulation,
    from natural convection/radiation at the outer surface balanced against conduction
    through the wall and insulation to the cryogen.

    Shared by the design-time insulation-thickness solve
    (``compute_cryogenic_cylindrical_tank_volume.py``, which searches over candidate
    ``t_ins`` at a single fixed design point) and the runtime in-flight boil-off model
    (``compute_cryogenic_tank_performance.py``, which evaluates this at the tank's
    already-sized, fixed ``t_ins``/geometry but at the actual time-varying flight
    ambient/cryogen temperatures).

    Parameters
    ----------
    t_ins : float
        Insulation thickness [m].
    T_env : float
        Ambient (hot-side) temperature [K].
    T_cold : float
        Cryogen (cold-side) temperature [K].
    k_mat, k_ins_mat : float
        Structural wall / insulation thermal conductivity [W/m-K].
    k_air, nu, alpha_th, Pr : float
        Ambient air thermal conductivity [W/m-K], kinematic viscosity [m^2/s],
        thermal diffusivity [m^2/s], and Prandtl number.
    ro, ri, li : float
        Structural wall outer radius, inner radius, and inner cylindrical length [m].

    Returns
    -------
    Te : float
        Equilibrium insulation outer-surface temperature [K].
    Q : float
        Total heat leak into the tank [W].

    Notes
    -----
    * Convection correlations: Churchill & Chu (1975) for the cylindrical section,
      Churchill (1983) for the hemispherical end caps.
    * Conduction uses concentric-cylinder and concentric-sphere resistance networks
      through the structural wall and insulation layer.
    """
    if abs(T_env - T_cold) < 1e-9:
        Te = T_cold
    else:
        lo, hi = (T_cold, T_env) if T_env > T_cold else (T_env, T_cold)
        args = (t_ins, T_env, T_cold, k_mat, k_ins_mat, k_air, nu, alpha_th, Pr, ro, ri, li)
        Te = _find_root(lambda x, *a: _heat_balance_residual(x, *a)[0], lo, hi, args=args)

    _, Q = _heat_balance_residual(Te, t_ins, T_env, T_cold, k_mat, k_ins_mat, k_air, nu, alpha_th, Pr, ro, ri, li)
    return Te, Q


# ----------------------------------------------------------------------------------------------------------------------
#  Heat balance at the insulation outer surface
#
#  At steady state the heat arriving at the outer surface (convection + radiation
#  from the warm ambient) must equal the heat conducted inward through the wall
#  and insulation layers to the cold cryogen:
#
#      Q_convection + Q_radiation - Q_conduction = 0
#
#  The root of this equation gives the equilibrium surface temperature Te.
# ----------------------------------------------------------------------------------------------------------------------
def _heat_balance_residual(Te, t_ins, Ta, Ti, k_mat, k_ins_mat, k_air, nu, alpha_th, Pr, ro, ri, li):
    g     = 9.81
    D_out = 2 * (ro + t_ins)                                          # outer diameter including insulation
    Ra    = (g / Ta) * (Ta - Te) * D_out**3 / (alpha_th * nu)         # Rayleigh number

    # ---- Cylindrical section ----
    Nu_cyl = (0.60 + 0.387 * Ra**(1 / 6) / (1 + (0.559 / Pr)**(9 / 16))**(8 / 27))**2
    h_cyl  = Nu_cyl * k_air / D_out
    A_cyl  = np.pi * D_out * li                                       # lateral surface area
    Qv_cyl = h_cyl * A_cyl * (Ta - Te)                                # convective heat gain
    Qr_cyl = 5.67e-8 * 0.03 * A_cyl * (Ta**4 - Te**4)                # radiative heat gain (emissivity = 0.03)
    Qc_cyl = (Te - Ti) / (np.log(ro / ri)         / (2 * np.pi * li * k_mat) +       # conduction: structural wall
                           np.log((ro + t_ins) / ro) / (2 * np.pi * li * k_ins_mat))  # conduction: insulation layer

    # ---- Spherical end caps (two hemispheres = one sphere) ----
    Nu_sph = 2 + 0.589 * Ra**(1 / 4) / (1 + (0.469 / Pr)**(9 / 16))**(4 / 9)
    h_sph  = Nu_sph * k_air / D_out
    A_sph  = np.pi * D_out**2                                         # surface area of full sphere
    Qv_sph = h_sph * A_sph * (Ta - Te)
    Qr_sph = 5.67e-8 * 0.03 * A_sph * (Ta**4 - Te**4)
    Qc_sph = (Te - Ti) / ((ro - ri) / (4 * np.pi * k_mat * ri * ro) +                # conduction: structural wall
                           t_ins     / (4 * np.pi * k_ins_mat * ro * (ro + t_ins)))    # conduction: insulation layer

    Qc = Qc_cyl + Qc_sph
    residual = (Qv_cyl + Qv_sph) + (Qr_cyl + Qr_sph) - Qc

    return residual, Qc


# ----------------------------------------------------------------------------------------------------------------------
#  Bounded 1D root finder
#
#  Tries brentq first (fast, guaranteed convergence when a sign change exists).
#  Falls back to minimizing f(x)^2 with minimize_scalar when brentq fails
#  (no sign change in the bracket, e.g. the root is a tangent zero).
# ----------------------------------------------------------------------------------------------------------------------
def _find_root(func, a, b, args=(), xtol=1e-9):
    try:
        return brentq(func, a, b, xtol=xtol, args=args)
    except ValueError:
        res = minimize_scalar(lambda x: func(x, *args)**2,
                              bounds=(a, b), method="bounded",
                              options={"xatol": xtol})
        return float(res.x)
