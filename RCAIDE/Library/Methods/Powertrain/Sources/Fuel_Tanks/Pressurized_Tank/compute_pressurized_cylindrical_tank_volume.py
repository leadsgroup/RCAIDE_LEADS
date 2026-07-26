# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Pressurized_Tank/compute_pressurized_cylindrical_tank_volume.py
#
# Created: Jul 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from scipy.optimize import brentq, minimize_scalar

# ----------------------------------------------------------------------------------------------------------------------
#  Pressurized Cylindrical Tank Volume
# ----------------------------------------------------------------------------------------------------------------------
def compute_pressurized_cylindrical_tank_volume(fuel_tank, fuel_tanks=None):
    """
    Sizes a pressurized rounded-end cylindrical tank (cylinder + hemispherical caps).

    The outer envelope is set by the upstream geometry function
    (e.g. ``compute_rounded_end_cylindrical_tank_volume`` or
    ``compute_wing_non_integral_tank_volume``).  This function works inward:
    it solves for the structural wall thickness required to contain
    ``design_pressure`` using thick-walled cylinder theory (Lamé equations with
    a von Mises failure criterion), then computes the resulting net fuel volume.

    Unlike the cryogenic version there is no insulation layer, so no thermal
    iteration is needed — the sizing is a single closed-form solve.

    Parameters
    ----------
    fuel_tank : Pressurized_Tank
        Tank object.  ``diameters.external`` and ``lengths.external`` must
        already be set by the upstream geometry function.
    fuel_tanks : Container, optional
        Parent container.  Tank is removed if sizing produces invalid results.

    Updates (in-place)
    ------------------
    fuel_tank.wall_thickness                     : structural wall thickness [m]
    fuel_tank.volume_properties.gross_volume     : internal volume incl. ullage [m³]
    fuel_tank.volume_properties.net_volume       : usable fuel volume [m³]
    fuel_tank.inner_structure.thickness          : wall thickness [m]
    fuel_tank.inner_structure.diameters.external : outer wall diameter [m]
    fuel_tank.inner_structure.diameters.internal : inner (fuel-side) diameter [m]
    fuel_tank.inner_structure.lengths.external   : outer cylindrical length [m]
    fuel_tank.inner_structure.lengths.internal   : inner cylindrical length [m]
    fuel_tank.inner_structure.mass_properties.mass : structural shell mass [kg]
    fuel_tank.mass_properties.mass               : total tank mass with accessories [kg]
    """

    # ------------------------------------------------------------------
    #  Unpack
    # ------------------------------------------------------------------
    safety_factor  = fuel_tank.safety_factor
    P_internal     = fuel_tank.design_pressure
    P_external     = fuel_tank.design_external_pressure
    sigma_allow    = fuel_tank.inner_structure.material.yield_tensile_strength / safety_factor
    ullage_frac    = fuel_tank.ullage_volume_fraction

    R_o   = fuel_tank.diameters.external / 2
    L_o   = fuel_tank.lengths.external
    L_cyl = L_o - 2 * R_o  # cylindrical section length (excluding hemispherical caps)

    # ------------------------------------------------------------------
    #  Structural wall sizing — Lamé / von Mises
    #
    #  Solve for ro/ri such that the von Mises stress at the inner wall
    #  equals sigma_allow.  Here ro = R_o (the external radius IS the
    #  pressure-vessel outer radius — no insulation on top).
    # ------------------------------------------------------------------
    ro_ri  = _find_root(_tank_stress, 1 + 1e-5, 2.0,
                        args=(P_internal, P_external, sigma_allow), xtol=1e-8)
    r_inner = R_o / ro_ri
    t_wall  = R_o - r_inner

    # Inner cylindrical section and spherical cap geometry
    L_cyl_i = L_cyl - 2 * t_wall  # inner cylindrical section length

    if L_cyl_i < 0:
        if fuel_tanks is not None:
            print(f"[WARNING] Tank '{fuel_tank.tag}': wall thickness exceeds cylinder half-length. "
                  "Removing from list.")
            fuel_tanks.pop(fuel_tank.tag)
        return

    # ------------------------------------------------------------------
    #  Volumes
    # ------------------------------------------------------------------
    # Gross internal volume (fuel + ullage)
    V_gross = np.pi * r_inner**2 * L_cyl_i + (4 / 3) * np.pi * r_inner**3
    V_net   = V_gross * (1 - ullage_frac)

    # ------------------------------------------------------------------
    #  Structural shell mass  (outer shell volume minus inner shell volume)
    # ------------------------------------------------------------------
    V_shell_o  = np.pi * R_o**2    * L_cyl   + (4 / 3) * np.pi * R_o**3
    V_shell_i  = np.pi * r_inner**2 * L_cyl_i + (4 / 3) * np.pi * r_inner**3
    V_material = V_shell_o - V_shell_i

    # ------------------------------------------------------------------
    #  Store results
    # ------------------------------------------------------------------
    fuel_tank.wall_thickness = t_wall

    fuel_tank.inner_structure.thickness              = t_wall
    fuel_tank.inner_structure.diameters.external     = 2 * R_o
    fuel_tank.inner_structure.diameters.internal     = 2 * r_inner
    fuel_tank.inner_structure.lengths.external       = L_o
    fuel_tank.inner_structure.lengths.internal       = L_o - 2 * t_wall

    fuel_tank.volume_properties.gross_volume = V_gross
    fuel_tank.volume_properties.net_volume   = V_net

    # Double for xz-symmetric tanks (left + right)
    if fuel_tank.xz_plane_symmetric:
        fuel_tank.volume_properties.gross_volume *= 2
        fuel_tank.volume_properties.net_volume   *= 2
        V_material                               *= 2

    fuel_tank.inner_structure.mass_properties.mass = (
        V_material * fuel_tank.inner_structure.material.density)
    fuel_tank.mass_properties.mass = (
        fuel_tank.tank_accesories_weight_factor
        * fuel_tank.inner_structure.mass_properties.mass)

    return


# ----------------------------------------------------------------------------------------------------------------------
#  Wall thickness: von Mises stress residual for a thick-walled cylinder (Lamé)
# ----------------------------------------------------------------------------------------------------------------------
def _tank_stress(ro_ri, P_internal, P_external, sigma_allow):
    A = (P_internal - P_external * ro_ri**2) / (ro_ri**2 - 1)
    B = (P_internal - P_external) * ro_ri**2 / (ro_ri**2 - 1)

    sigma_theta = A + B   # hoop (circumferential) at inner wall
    sigma_r     = A - B   # radial at inner wall
    sigma_z     = A       # axial (closed-end cylinder)

    sigma_vm = np.sqrt(((sigma_theta - sigma_r)**2 +
                        (sigma_r     - sigma_z)**2 +
                        (sigma_z     - sigma_theta)**2) / 2)
    return sigma_vm - sigma_allow


# ----------------------------------------------------------------------------------------------------------------------
#  Bounded 1D root finder (brentq with minimize_scalar fallback)
# ----------------------------------------------------------------------------------------------------------------------
def _find_root(func, a, b, args=(), xtol=1e-9):
    try:
        return brentq(func, a, b, xtol=xtol, args=args)
    except ValueError:
        res = minimize_scalar(lambda x: func(x, *args)**2,
                              bounds=(a, b), method="bounded",
                              options={"xatol": xtol})
        return float(res.x)
