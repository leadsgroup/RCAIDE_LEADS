# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank/compute_cryogenic_tank_heat_leak.py
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from RCAIDE.Framework.Core.Physical_Constants import STEFAN_BOLTZMANN
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Common.find_root import _find_root
try:
    from numba import njit
except ImportError:  # pragma: no cover -- numba is optional: without it the kernels below run as plain Python
    def njit(*args, **kwargs):
        if len(args) == 1 and callable(args[0]):
            return args[0]
        return lambda function: function

CYLINDER = 0
CUBOID   = 1

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
    return solve_heat_leak(CYLINDER, (t_ins, T_env, T_cold, k_mat, k_ins_mat, k_air, nu, alpha_th, Pr, ro, ri, li))


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
@njit(cache=True)
def heat_balance_residual_cylinder(Te, p):
    t_ins, Ta, Ti, k_mat, k_ins_mat, k_air, nu, alpha_th, Pr, ro, ri, li = p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11]
    g     = 9.81
    D_out = 2 * (ro + t_ins)                                          # outer diameter including insulation
    Ra    = (g / Ta) * (Ta - Te) * D_out**3 / (alpha_th * nu)         # Rayleigh number

    # ---- Cylindrical section ----
    Nu_cyl = (0.60 + 0.387 * Ra**(1 / 6) / (1 + (0.559 / Pr)**(9 / 16))**(8 / 27))**2
    h_cyl  = Nu_cyl * k_air / D_out
    A_cyl  = np.pi * D_out * li                                       # lateral surface area
    Qv_cyl = h_cyl * A_cyl * (Ta - Te)                                # convective heat gain
    Qr_cyl = STEFAN_BOLTZMANN * 0.03 * A_cyl * (Ta**4 - Te**4)      # radiative heat gain (emissivity = 0.03)
    Qc_cyl = (Te - Ti) / (np.log(ro / ri)         / (2 * np.pi * li * k_mat) +       # conduction: structural wall
                           np.log((ro + t_ins) / ro) / (2 * np.pi * li * k_ins_mat))  # conduction: insulation layer

    # ---- Spherical end caps (two hemispheres = one sphere) ----
    Nu_sph = 2 + 0.589 * Ra**(1 / 4) / (1 + (0.469 / Pr)**(9 / 16))**(4 / 9)
    h_sph  = Nu_sph * k_air / D_out
    A_sph  = np.pi * D_out**2                                         # surface area of full sphere
    Qv_sph = h_sph * A_sph * (Ta - Te)
    Qr_sph = STEFAN_BOLTZMANN * 0.03 * A_sph * (Ta**4 - Te**4)
    Qc_sph = (Te - Ti) / ((ro - ri) / (4 * np.pi * k_mat * ri * ro) +                # conduction: structural wall
                           t_ins     / (4 * np.pi * k_ins_mat * ro * (ro + t_ins)))    # conduction: insulation layer

    Qc = Qc_cyl + Qc_sph
    residual = (Qv_cyl + Qv_sph) + (Qr_cyl + Qr_sph) - Qc

    return residual, Qc


# ----------------------------------------------------------------------------------------------------------------------
#  Cryogenic Tank Environmental Heat Leak (Prismatic/Cuboid)
# ----------------------------------------------------------------------------------------------------------------------
def compute_cryogenic_tank_heat_leak_cuboid(t_ins, T_env, T_cold, k_mat, k_ins_mat, k_air, nu, alpha_th, Pr, l_o, w_o, h_o, th):
    """
    Heat leak through a prismatic (cuboid) tank's structural wall + insulation,
    from natural convection/radiation at the six outer faces balanced against
    planar conduction through the wall and insulation to the cryogen.

    Companion to ``compute_cryogenic_tank_heat_leak`` (cylindrical case): same
    equilibrium-outer-surface-temperature solution strategy, but with flat-plate
    free-convection correlations per face orientation instead of the
    cylinder/sphere correlations, and planar (not concentric-shell) conduction
    resistance. ``heights.external`` is taken as the vertical (gravity-aligned)
    dimension. Shared by the design-time insulation-thickness solve
    (``compute_cryogenic_conformal_tank_volume.py``) and the runtime in-flight
    boil-off model (``compute_cryogenic_tank_performance.py``), the same way
    ``compute_cryogenic_tank_heat_leak`` is shared for the cylindrical case.

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
    l_o, w_o, h_o : float
        Structural wall outer length, width, height [m] (pre-insulation).
    th : float
        Structural wall thickness [m].

    Returns
    -------
    Te : float
        Equilibrium insulation outer-surface temperature [K].
    Q : float
        Total heat leak into the tank [W].

    Notes
    -----
    * Convection correlations (Incropera & DeWitt / Bergman et al. free
      convection): Churchill-Chu for the four vertical side faces (same
      correlation already used for the cylindrical case's mid-section, applied
      here with the tank height as the characteristic length); the horizontal
      top/bottom faces use the up/down-facing cold-plate correlations, since a
      cold surface facing up (stably stratified) transfers markedly less heat
      than one facing down (buoyancy-favorable) at the same Rayleigh number.
    * Conduction uses a single planar wall/insulation resistance network,
      referenced to the structural (pre-insulation) outer surface area.
    """
    return solve_heat_leak(CUBOID, (t_ins, T_env, T_cold, k_mat, k_ins_mat, k_air, nu, alpha_th, Pr, l_o, w_o, h_o, th))


# ----------------------------------------------------------------------------------------------------------------------
#  Heat balance at the insulation outer surface (cuboid case)
# ----------------------------------------------------------------------------------------------------------------------
@njit(cache=True)
def heat_balance_residual_cuboid(Te, p):
    t_ins, Ta, Ti, k_mat, k_ins_mat, k_air, nu, alpha_th, Pr, l_o, w_o, h_o, th = p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12]
    g    = 9.81
    l_oo = l_o + 2 * t_ins
    w_oo = w_o + 2 * t_ins
    h_oo = h_o + 2 * t_ins

    # ---- Vertical side faces (2 x l_oo*h_oo, 2 x w_oo*h_oo), Churchill-Chu ----
    Ra_v    = (g / Ta) * (Ta - Te) * h_oo**3 / (alpha_th * nu)
    Nu_v    = (0.60 + 0.387 * Ra_v**(1 / 6) / (1 + (0.559 / Pr)**(9 / 16))**(8 / 27))**2
    h_v     = Nu_v * k_air / h_oo
    A_side  = 2 * l_oo * h_oo + 2 * w_oo * h_oo
    Qv_side = h_v * A_side * (Ta - Te)
    Qr_side = STEFAN_BOLTZMANN * 0.03 * A_side * (Ta**4 - Te**4)

    # ---- Horizontal top/bottom faces ----
    A_horiz = l_oo * w_oo
    L_c     = A_horiz / (2 * (l_oo + w_oo))                          # A/P characteristic length
    Ra_h    = (g / Ta) * (Ta - Te) * L_c**3 / (alpha_th * nu)

    # Bottom face: cold surface facing down (buoyancy-favorable, same
    # correlation family as a hot plate facing up)
    Nu_bot  = 0.54 * Ra_h**(1 / 4) if Ra_h <= 1e7 else 0.15 * Ra_h**(1 / 3)
    h_bot   = Nu_bot * k_air / L_c
    Qv_bot  = h_bot * A_horiz * (Ta - Te)
    Qr_bot  = STEFAN_BOLTZMANN * 0.03 * A_horiz * (Ta**4 - Te**4)

    # Top face: cold surface facing up (stably stratified, same correlation
    # family as a hot plate facing down)
    Nu_top  = 0.27 * Ra_h**(1 / 4)
    h_top   = Nu_top * k_air / L_c
    Qv_top  = h_top * A_horiz * (Ta - Te)
    Qr_top  = STEFAN_BOLTZMANN * 0.03 * A_horiz * (Ta**4 - Te**4)

    # ---- Conduction through wall + insulation (planar, area-referenced to
    #      the structural outer surface) ----
    A_ref = 2 * (l_o * w_o + l_o * h_o + w_o * h_o)
    Qc    = (Te - Ti) * A_ref / (th / k_mat + t_ins / k_ins_mat)

    residual = (Qv_side + Qv_bot + Qv_top) + (Qr_side + Qr_bot + Qr_top) - Qc
    return residual, Qc


# ----------------------------------------------------------------------------------------------------------------------
#  Shared solve: equilibrium outer-surface temperature (Te), then heat leak (Q) at Te
#
#  Common to the cylindrical and cuboid cases -- only the residual function and its
#  geometry args differ between them.
# ----------------------------------------------------------------------------------------------------------------------
@njit(cache=True)
def heat_balance_residual(geometry, Te, p):
    if geometry == CYLINDER:
        return heat_balance_residual_cylinder(Te, p)
    return heat_balance_residual_cuboid(Te, p)


@njit(cache=True)
def brent_heat_leak(geometry, xa, xb, xtol, rtol, maxiter, p):
    """Brent's method, ported line for line from scipy's brentq (scipy/optimize/Zeros/brentq.c), on the
    heat-balance residual. Returns (root, status): 0 converged, 1 no sign change, 2 not converged."""
    xpre, xcur = xa, xb
    xblk, fblk, spre, scur = 0., 0., 0., 0.
    fpre = heat_balance_residual(geometry, xpre, p)[0]
    fcur = heat_balance_residual(geometry, xcur, p)[0]
    if fpre == 0:
        return xpre, 0
    if fcur == 0:
        return xcur, 0
    if np.signbit(fpre) == np.signbit(fcur):
        return 0., 1
    for i in range(maxiter):
        if fpre != 0 and fcur != 0 and np.signbit(fpre) != np.signbit(fcur):
            xblk = xpre
            fblk = fpre
            spre = scur = xcur - xpre
        if abs(fblk) < abs(fcur):
            xpre = xcur
            xcur = xblk
            xblk = xpre
            fpre = fcur
            fcur = fblk
            fblk = fpre
        delta = (xtol + rtol*abs(xcur))/2
        sbis  = (xblk - xcur)/2
        if fcur == 0 or abs(sbis) < delta:
            return xcur, 0
        if abs(spre) > delta and abs(fcur) < abs(fpre):
            if xpre == xblk:
                stry = -fcur*(xcur - xpre)/(fcur - fpre)                       # interpolate
            else:
                dpre = (fpre - fcur)/(xpre - xcur)                             # extrapolate
                dblk = (fblk - fcur)/(xblk - xcur)
                stry = -fcur*(fblk*dblk - fpre*dpre)/(dblk*dpre*(fblk - fpre))
            if 2*abs(stry) < min(abs(spre), 3*abs(sbis) - delta):
                spre = scur
                scur = stry
            else:
                spre = sbis
                scur = sbis
        else:
            spre = sbis
            scur = sbis
        xpre = xcur
        fpre = fcur
        if abs(scur) > delta:
            xcur += scur
        else:
            xcur += delta if sbis > 0 else -delta
        fcur = heat_balance_residual(geometry, xcur, p)[0]
    return xcur, 2


def solve_heat_leak(geometry, args):
    """Equilibrium outer-surface temperature Te, then the heat leak Q at Te. args = (t_ins, T_env, T_cold,
    material/air properties, geometry...) as in the public functions; scalars or 1-element arrays."""
    p = np.array([np.asarray(a, dtype=float).item() for a in args])
    T_env, T_cold = p[1], p[2]
    if abs(T_env - T_cold) < 1e-9:
        Te = T_cold
    else:
        lo, hi = (T_cold, T_env) if T_env > T_cold else (T_env, T_cold)
        if not (np.isfinite(lo) and np.isfinite(hi)):
            _find_root(None, lo, hi)                                           # raises the non-finite-bracket error
        Te, status = brent_heat_leak(geometry, lo, hi, 1e-9, 4 * np.finfo(float).eps, 100, p)
        if status == 1:                                                         # no sign change: same fallback as _find_root
            Te = _find_root(lambda x: heat_balance_residual(geometry, x, p)[0], lo, hi)
        elif status == 2:
            raise RuntimeError("solve_heat_leak: Brent's method failed to converge after 100 iterations.")
    _, Q = heat_balance_residual(geometry, Te, p)
    return Te, Q
