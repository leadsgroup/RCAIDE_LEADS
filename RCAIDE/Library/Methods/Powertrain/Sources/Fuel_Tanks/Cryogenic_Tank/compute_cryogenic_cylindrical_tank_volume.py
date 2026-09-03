# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank/compute_cryogenic_cylindrical_tank_volume.py
#
# Created: Dec 2025, S. Shekar
# Modified: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank.compute_cryogenic_tank_heat_leak import compute_cryogenic_tank_heat_leak
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Common.find_root import _find_root
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Common.solve_insulation import _solve_insulation

import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Cryogenic Cylindrical Tank Volume
# ----------------------------------------------------------------------------------------------------------------------
def compute_cryogenic_cylindrical_tank_volume(fuel_tank, fuel_tanks=None):
    """
    Sizes a cryogenic rounded-end cylindrical tank (cylinder + hemispherical caps)
    to fit within the outer mold line set by the upstream non-integral tank function.

    The upstream function (e.g. compute_wing_non_integral_tank_volume) determines the
    external dimensions (diameters.external, lengths.external) from the wing geometry.
    This function works inward: it iterates on the usable fuel volume until the
    pressure vessel wall + insulation thickness exactly fills the available envelope.

    Sizing involves three nested solves:
        1. Outer loop   – adjusts fuel volume (V_guess) until the total outer radius
                          (structural wall + insulation) matches the external envelope.
        2. Middle solve – finds the insulation thickness that satisfies the allowable
                          heat leak constraint for a given inner geometry.
        3. Inner solve  – finds the equilibrium temperature at the insulation outer
                          surface by balancing external convection/radiation with
                          conduction through the wall and insulation.

    The wall thickness ratio (ro/ri) is solved once before the outer loop because it
    depends only on internal/external pressures and material yield strength (von Mises),
    which do not change during iteration.

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        Fuel tank object. Must have external dimensions already set by the upstream
        non-integral tank function.
    fuel_tanks : Container
        Parent container (tank is removed if sizing produces invalid results).

    Updates (in-place)
    ------------------
    fuel_tank.volume_properties.net_volume   : usable fuel volume [m³]
    fuel_tank.volume_properties.gross_volume : total internal volume incl. ullage [m³]
    fuel_tank.inner_structure.*              : pressure vessel geometry
    fuel_tank.insulation_thickness           : required insulation thickness [m]
    fuel_tank.structural.mass_properties.mass: structural shell mass [kg]
    fuel_tank.insulation.mass_properties.mass: insulation mass [kg]
    fuel_tank.mass_properties.mass           : total tank mass (structure + insulation) [kg]
    """
    fuel_tank.wall_thickness = None

    # ------------------------------------------------------------------
    #  Unpack constants that do not change during iteration
    # ------------------------------------------------------------------
    safety_factor = fuel_tank.safety_factor
    aspect_ratio  = fuel_tank.lengths.external / fuel_tank.diameters.external      
    ullage_frac   = fuel_tank.ullage_volume_fraction  # fraction of internal volume reserved for ullage
    T_inlet       = fuel_tank.design_inlet_temperature
    Qo            = fuel_tank.design_heat_flux    # max allowable heat leak [W/m²]
    k_mat         = fuel_tank.inner_structure.material.thermal_conductivity
    k_ins_mat     = fuel_tank.insulation.material.thermal_conductivity
    sigma_allow   = fuel_tank.inner_structure.material.yield_tensile_strength / safety_factor
    PI_Q          = 1.5  # heat-flow multiplier for thermal sizing margin

    # Internal and external design pressures. P_internal is the physical
    # rated pressure itself -- no separate burst/proof multiplier is applied
    # on top of it; safety_factor (via sigma_allow above) is the sole
    # structural margin.
    P_sat      = fuel_tank.fuel.cryogen_properties(T_inlet, "Pressure (MPa)", phase='liquid') * Units.MPa
    P_rated    = P_sat + fuel_tank.pressure_margin
    P_internal = P_rated
    P_external = fuel_tank.design_external_pressure

    # Operating (rated) pressure target for the in-flight boil-off model --
    # the same P_rated used as the structural design pressure above.
    fuel_tank.design_pressure = P_rated

    # Atmospheric conditions at design altitude (computed once, passed to inner solvers)
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(fuel_tank.design_altitude,
                                           fuel_tank.design_isa_deviation)
    Ta     = float(np.asarray(atmo_data.temperature).ravel()[0])
    rho    = float(np.asarray(atmo_data.density).ravel()[0])
    mu     = float(np.asarray(atmo_data.dynamic_viscosity).ravel()[0])
    k_air  = float(np.asarray(atmo_data.thermal_conductivity).ravel()[0])
    Cp_air = float(np.asarray(atmosphere.fluid_properties.compute_cp(Ta)).ravel()[0])

    # Derived air properties
    nu       = mu / rho                    # kinematic viscosity [m²/s]
    alpha_th = k_air / (rho * Cp_air)      # thermal diffusivity [m²/s]
    Pr       = nu / alpha_th               # Prandtl number

    # ------------------------------------------------------------------
    #  Step 1: Solve wall thickness ratio ro/ri (loop-invariant)
    #
    #  Uses thick-walled cylinder theory (Lamé) with von Mises failure
    #  criterion. The ratio ro/ri determines the structural wall thickness
    #  for any inner radius: t_wall = r_inner * (ro_ri - 1).
    # ------------------------------------------------------------------
    ro_ri = _find_root(_tank_stress, 1 + 1e-5, 1.1, args=(P_internal, P_external, sigma_allow), xtol=1e-6)

    # Outer envelope volume from external dimensions (set by upstream function)
    R_true = fuel_tank.diameters.external / 2
    L_true = fuel_tank.lengths.external
    V_outer_true = np.pi * R_true**2 * L_true + (4 / 3) * np.pi * R_true**3

    # Pack thermal constants into a tuple for the inner solvers
    therm = (Ta, T_inlet, Qo, PI_Q, k_mat, k_ins_mat, k_air, nu, alpha_th, Pr)

    # ------------------------------------------------------------------
    #  Step 2: Solve for fuel volume
    #
    #  Find V_guess such that the resulting tank (structure + insulation)
    #  exactly fills the outer envelope:
    #    V_guess → V_total (add ullage) → r_inner, L_inner (from aspect ratio)
    #    → r_outer (wall thickness) → t_ins (insulation) → outer volume
    #  Root is where outer volume = V_outer_true.
    # ------------------------------------------------------------------
    outer_args = (ullage_frac, aspect_ratio, ro_ri, L_true, V_outer_true, therm, fuel_tank)
    V_lo       = 1e-3
    V_hi       = fuel_tank.volume_properties.gross_volume
    V_guess    = _find_root(_volume_residual, V_lo, V_hi, args=outer_args, xtol=1e-5)

    # Recompute converged geometry (one final evaluation)
    V_total = V_guess / (1 - ullage_frac)
    r_inner = (V_total / (2 * np.pi * (aspect_ratio - 1 / 3)))**(1 / 3)
    L_inner = 2 * r_inner * (aspect_ratio - 1)
    r_outer = ro_ri * r_inner
    t_ins   = _solve_insulation(_insulation_residual, therm, fuel_tank, r_outer, r_inner, L_inner)

    # compute design total heat transfer if not already set (used for boil-off model)
    if fuel_tank.design_total_heat_transfer is None:
        _, Qc_total = compute_cryogenic_tank_heat_leak(t_ins, Ta, T_inlet, k_mat, k_ins_mat, k_air, nu, alpha_th, Pr, r_outer, r_inner, L_inner)
        fuel_tank.design_total_heat_transfer = Qc_total

    fuel_tank.volume_properties.net_volume   = V_guess
    fuel_tank.volume_properties.gross_volume = V_total

    # ------------------------------------------------------------------
    #  Store converged structural geometry
    # ------------------------------------------------------------------
    fuel_tank.inner_structure.thickness           = r_outer - r_inner
    fuel_tank.inner_structure.diameters.external  = 2 * r_outer
    fuel_tank.inner_structure.diameters.internal  = 2 * r_inner
    fuel_tank.inner_structure.lengths.internal    = L_inner
    fuel_tank.inner_structure.lengths.external    = 2 * r_outer * (aspect_ratio - 1)
    fuel_tank.insulation.thickness                = t_ins

    # ------------------------------------------------------------------
    #  Compute insulation mass
    #  v_ins = volume between outer envelope and structural shell
    #  a_ins = outer surface area (for surface-density-based insulation)
    # ------------------------------------------------------------------
    R_ext   = fuel_tank.diameters.external / 2
    L_cyl_e = fuel_tank.lengths.external - fuel_tank.diameters.external      # cylinder-only (tip-to-tip minus caps)
    R_so    = fuel_tank.inner_structure.diameters.external / 2
    L_so    = fuel_tank.inner_structure.lengths.external

    a_ins    = 2 * np.pi * R_ext * L_cyl_e + 4 * np.pi * R_ext**2
    v_ins    = (np.pi * R_ext**2 * L_cyl_e + (4 / 3) * np.pi * R_ext**3) - \
               (np.pi * R_so**2  * L_so    + (4 / 3) * np.pi * R_so**3)
    mass_ins = v_ins * fuel_tank.insulation.material.density + \
               a_ins * fuel_tank.insulation.material.specific_density

    # ------------------------------------------------------------------
    #  Compute structural shell mass
    #  V_material = volume of pressure vessel wall (outer shell - inner shell)
    # ------------------------------------------------------------------
    L_outer    = fuel_tank.inner_structure.lengths.external
    V_shell_o  = np.pi * r_outer**2 * L_outer + (4 / 3) * np.pi * r_outer**3
    V_shell_i  = np.pi * r_inner**2 * L_inner + (4 / 3) * np.pi * r_inner**3
    V_material = V_shell_o - V_shell_i

    # Double volumes and masses for symmetric tanks (left + right wing)
    if fuel_tank.xz_plane_symmetric:
        fuel_tank.volume_properties.gross_volume *= 2
        fuel_tank.volume_properties.net_volume   *= 2
        V_material *= 2
        mass_ins   *= 2

    if np.isnan(mass_ins):
        print(f"[WARNING] Tank '{fuel_tank.tag}' is too small. Removing from list.")
        fuel_tanks.pop(fuel_tank.tag)

    fuel_tank.insulation.mass_properties.mass = mass_ins
    fuel_tank.inner_structure.mass_properties.mass = V_material * fuel_tank.inner_structure.material.density
    fuel_tank.mass_properties.mass = fuel_tank.tank_accesories_weight_factor * (
        fuel_tank.insulation.mass_properties.mass + fuel_tank.inner_structure.mass_properties.mass)

    return


# ----------------------------------------------------------------------------------------------------------------------
#  Wall thickness: von Mises stress residual for a thick-walled cylinder
#
#  Uses Lamé's equations for radial, hoop, and axial stresses at the inner wall.
#  Returns (von Mises stress) - (allowable stress); root is at the correct ro/ri.
# ----------------------------------------------------------------------------------------------------------------------
def _tank_stress(ro_ri, P_internal, P_external, sigma_allow):
    # Lamé constants
    A = (P_internal - P_external * ro_ri**2) / (ro_ri**2 - 1)
    B = (P_internal - P_external) * ro_ri**2  / (ro_ri**2 - 1)

    # Principal stresses at the inner wall (worst case)
    sigma_theta = A + B    # hoop (circumferential)
    sigma_r     = A - B    # radial
    sigma_z     = A        # axial (closed-end cylinder)

    # Von Mises equivalent stress
    sigma_vm = np.sqrt(((sigma_theta - sigma_r)**2 +
                        (sigma_r     - sigma_z)**2 +
                        (sigma_z     - sigma_theta)**2) / 2)
    return sigma_vm - sigma_allow


# ----------------------------------------------------------------------------------------------------------------------
#  Outer volume residual: V_outer_true - V_outer(V_guess)
#
#  Maps a fuel volume guess through the full sizing chain (ullage → geometry →
#  structural wall → insulation) and returns the difference between the resulting
#  outer envelope volume and the target. Root is the correct fuel volume.
# ----------------------------------------------------------------------------------------------------------------------
def _volume_residual(V_guess, ullage_frac, aspect_ratio, ro_ri, L_true, V_outer_true, therm, fuel_tank):
    V_total = V_guess / (1 - ullage_frac)
    r_inner = (V_total / (2 * np.pi * (aspect_ratio - 1 / 3)))**(1 / 3)
    L_inner = 2 * r_inner * (aspect_ratio - 1)
    r_outer = ro_ri * r_inner
    t_ins   = _solve_insulation(_insulation_residual, therm, fuel_tank, r_outer, r_inner, L_inner)
    R_calc  = r_outer + t_ins
    V_outer_calc = np.pi * R_calc**2 * L_true + (4 / 3) * np.pi * R_calc**3
    return V_outer_true - V_outer_calc


# ----------------------------------------------------------------------------------------------------------------------
#  Insulation residual: (heat flux through insulation) - (allowable heat leak)
#
#  For a given insulation thickness, solves for the equilibrium surface temperature and
#  resulting heat leak via the shared Churchill/conduction-network model (also used at
#  runtime by compute_cryogenic_tank_performance.py), then compares the conductive heat
#  flux through the wall to the maximum allowable heat leak Qo.
# ----------------------------------------------------------------------------------------------------------------------
def _insulation_residual(t_ins, therm, fuel_tank, r_o, r_i, l_i):
    Ta, Ti, Qo, PI_Q, k_mat, k_ins_mat, k_air, nu, alpha_th, Pr = therm

    _, Qc = compute_cryogenic_tank_heat_leak(t_ins, Ta, Ti, k_mat, k_ins_mat, k_air, nu, alpha_th, Pr, r_o, r_i, l_i)

    # Compare conductive heat flux (per unit inner surface area) to allowable
    A_inner = 2 * np.pi * r_i * l_i + 4 * np.pi * r_i**2
    return PI_Q * Qc / A_inner - Qo
