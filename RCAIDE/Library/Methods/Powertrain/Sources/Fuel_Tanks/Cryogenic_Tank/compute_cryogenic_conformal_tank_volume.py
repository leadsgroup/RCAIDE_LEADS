# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Cryogenic_Tank/compute_cryogenic_tank_conformal_tank_volume.py
#
# Created: Feb 2026, S. Shekar
# Modified: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units

import numpy as np
from .compute_cryogenic_cylindrical_tank_volume import _find_root

# ----------------------------------------------------------------------------------------------------------------------
#  Cryogenic Conformal (Prismatic) Tank Volume
# ----------------------------------------------------------------------------------------------------------------------
def compute_cryogenic_conformal_tank_volume(fuel_tank, _):
    """
    Sizes a cryogenic conformal (prismatic/cuboid) tank to fit within the equivalent
    rectangle envelope set by the upstream integral tank function.

    The upstream function (e.g. compute_wing_integral_tank_volume) determines the
    outer envelope via widths.external, lengths.external, heights.external.
    This function works inward: it finds the fuel volume where the pressure vessel
    wall + insulation exactly fills that envelope.

    Sizing chain (all direct — no nested solvers):
        V_guess → V_total (add ullage) → h_i, w_i, l_i (from aspect ratio)
        → th (membrane stress, closed-form) → outer structure dimensions
        → t_ins (1D conduction) → total outer dimensions → V_calculated
    Root is where V_calculated = V_cuboid (the envelope volume).

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        Fuel tank with external dimensions already set by the upstream function.

    Updates (in-place)
    ------------------
    fuel_tank.volume_properties.net_volume / gross_volume
    fuel_tank.inner_structure.*
    fuel_tank.insulation_thickness, total_thickness
    fuel_tank.structural / insulation / mass_properties.mass
    """
    fuel_tank.wall_thickness = None

    # ------------------------------------------------------------------
    #  Unpack constants
    # ------------------------------------------------------------------
    safety_factor = fuel_tank.safety_factor
    aspect_ratio  = fuel_tank.lengths.external / fuel_tank.heights.external
    ullage_frac   = fuel_tank.ullage_volume_fraction
    T_inlet       = fuel_tank.design_inlet_temperature
    Qo_total      = fuel_tank.design_total_heat_transfer
    k_ins_mat     = fuel_tank.insulation.material.thermal_conductivity
    sigma_allow   = fuel_tank.inner_structure.material.yield_tensile_strength / safety_factor
    hw_ratio      = fuel_tank.heights.external / fuel_tank.widths.external

    # Net pressure differential for structural sizing
    P_sat      = fuel_tank.fuel.cryogen_properties(T_inlet, "Pressure (MPa)", phase='liquid') * Units.MPa
    P_internal = fuel_tank.pressure_factor * P_sat
    P_external = fuel_tank.design_external_pressure
    P_net      = P_internal - P_external

    # Operating (rated) pressure target for the in-flight boil-off model --
    # distinct from P_internal above, which is the structural proof/burst
    # pressure used only to size wall thickness with margin.
    fuel_tank.design_pressure = P_sat + fuel_tank.pressure_margin

    # Atmospheric temperature at design altitude
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo_data  = atmosphere.compute_values(fuel_tank.design_altitude,
                                           fuel_tank.design_isa_deviation)
    Ta = float(np.asarray(atmo_data.temperature).ravel()[0])

    # Outer envelope volume (constant target)
    V_cuboid = fuel_tank.widths.external * fuel_tank.heights.external * fuel_tank.lengths.external

    # ------------------------------------------------------------------
    #  Solve for fuel volume
    # ------------------------------------------------------------------
    outer_args = (ullage_frac, aspect_ratio, hw_ratio, P_net, sigma_allow,
                  Ta, T_inlet, Qo_total, k_ins_mat, fuel_tank, V_cuboid)
    V_lo    = 1e-3
    V_hi    = V_cuboid
    V_guess = _find_root(_volume_residual, V_lo, V_hi, args=outer_args, xtol=1e-2)

    # ------------------------------------------------------------------
    #  Recompute converged geometry
    # ------------------------------------------------------------------
    V_total, h_i, w_i, l_i, th, h_o, w_o, l_o, t_ins, mass_ins, h_o_o, w_o_o, l_o_o = \
        _sizing_chain(V_guess, ullage_frac, aspect_ratio, hw_ratio, P_net, sigma_allow,
                      Ta, T_inlet, Qo_total, k_ins_mat, fuel_tank)

    mass_struct = ((l_o * w_o * h_o) - (h_i * l_i * w_i)) * fuel_tank.inner_structure.material.density

    # Double for symmetric tanks
    if fuel_tank.xz_plane_symmetric:
        V_guess     *= 2
        V_total     *= 2
        mass_struct *= 2
        mass_ins    *= 2

    # ------------------------------------------------------------------
    #  Store results
    # ------------------------------------------------------------------
    fuel_tank.volume_properties.net_volume   = V_guess
    fuel_tank.volume_properties.gross_volume = V_total

    # Structural pressure vessel geometry
    fuel_tank.inner_structure.thickness          = th
    fuel_tank.inner_structure.lengths.external   = l_o
    fuel_tank.inner_structure.lengths.internal   = l_i
    fuel_tank.inner_structure.widths.external    = w_o
    fuel_tank.inner_structure.widths.internal    = w_i
    fuel_tank.inner_structure.heights.external   = h_o
    fuel_tank.inner_structure.heights.internal   = h_i

    # Overall outer dimensions (structure + insulation)
    fuel_tank.lengths.external = l_o_o
    fuel_tank.widths.external  = w_o_o
    fuel_tank.heights.external = h_o_o

    # Insulation and wall thicknesses
    fuel_tank.insulation.thickness = t_ins
    fuel_tank.wall_thickness       = t_ins + th

    # Mass buildup
    fuel_tank.insulation.mass_properties.mass = mass_ins
    fuel_tank.inner_structure.mass_properties.mass = mass_struct
    fuel_tank.mass_properties.mass = fuel_tank.tank_accesories_weight_factor * (
        fuel_tank.insulation.mass_properties.mass + fuel_tank.inner_structure.mass_properties.mass)

    return


# ----------------------------------------------------------------------------------------------------------------------
#  Sizing chain: V_guess → all intermediate geometry → total outer volume
# ----------------------------------------------------------------------------------------------------------------------
def _sizing_chain(V_guess, ullage_frac, aspect_ratio, hw_ratio, P_net, sigma_allow,
                  Ta, Ti, Qo_total, k_ins_mat, fuel_tank):
    # Internal volume = fuel + ullage
    V_total = V_guess / (1 - ullage_frac)

    # Inner dimensions from volume, aspect ratio, and height/width ratio
    h_i = ((V_total / aspect_ratio) * hw_ratio)**(1 / 3)
    w_i = h_i / hw_ratio
    l_i = aspect_ratio * h_i

    # Structural wall thickness (membrane stress on a rectangular pressure vessel)
    # σ = P * L_max / (2t)  →  t = P * L_max / (2σ)
    th = P_net * max(l_i, w_i) / (2 * sigma_allow)

    # Outer structural dimensions
    h_o = h_i + 2 * th
    l_o = l_i + 2 * th
    w_o = w_i + 2 * th

    # Insulation thickness (1D conduction: t = k·ΔT / q_flux)
    area_ref = 2 * (l_o * w_o + l_o * h_o + w_o * h_o)
    q_flux   = Qo_total / max(area_ref, 1e-12)
    t_ins    = k_ins_mat * (Ta - Ti) / max(q_flux, 1e-12)

    # Total outer dimensions (structure + insulation)
    h_o_o = h_o + 2 * t_ins
    l_o_o = l_o + 2 * t_ins
    w_o_o = w_o + 2 * t_ins

    # Insulation mass
    a_ins    = 2 * (l_o_o * w_o_o + l_o_o * h_o_o + w_o_o * h_o_o)
    v_ins    = (l_o_o * w_o_o * h_o_o) - (l_o * w_o * h_o)
    mass_ins = v_ins * fuel_tank.insulation.material.density + \
               a_ins * fuel_tank.insulation.material.specific_density

    return V_total, h_i, w_i, l_i, th, h_o, w_o, l_o, t_ins, mass_ins, h_o_o, w_o_o, l_o_o


# ----------------------------------------------------------------------------------------------------------------------
#  Outer volume residual: V_cuboid - V_calculated(V_guess)
# ----------------------------------------------------------------------------------------------------------------------
def _volume_residual(V_guess, ullage_frac, aspect_ratio, hw_ratio, P_net, sigma_allow,
                     Ta, Ti, Qo_total, k_ins_mat, fuel_tank, V_cuboid):
    results = _sizing_chain(V_guess, ullage_frac, aspect_ratio, hw_ratio, P_net, sigma_allow,
                            Ta, Ti, Qo_total, k_ins_mat, fuel_tank)
    h_o_o, w_o_o, l_o_o = results[10], results[11], results[12]
    return V_cuboid - (h_o_o * l_o_o * w_o_o)
