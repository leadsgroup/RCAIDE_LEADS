# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/evaluate_FEA.py
#
# Created: Mar 2026, M. Clarke, S. Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core                                                          import Data
from RCAIDE.Library.Methods.Aerostructures.Finite_Element_Analysis.FEA              import FEA
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM                  import VLM
from RCAIDE.Library.Methods.Utilities                                                import Cubic_Spline_Blender

import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  evaluate_surrogate
# ----------------------------------------------------------------------------------------------------------------------
def evaluate_surrogate(state, settings, vehicle):
    """Query FEA surrogates to obtain wing deflections and aerodynamic coefficients.

    Args:
        state    : mission segment state (holds conditions and analyses)
        settings : analysis settings
        vehicle  : vehicle configuration

    Returns:
        None — results written directly to state.conditions
    """
    conditions     = state.conditions
    aerostructures = state.analyses.aerostructures

    sub_sur   = aerostructures.surrogates.subsonic
    sup_sur   = aerostructures.surrogates.supersonic
    trans_sur = aerostructures.surrogates.transonic

    AoA    = np.atleast_2d(conditions.aerodynamics.angles.alpha)
    Mach   = np.atleast_2d(conditions.freestream.mach_number)
    n_cpts = len(AoA)

    hsub_min = aerostructures.hsub_min
    hsub_max = aerostructures.hsub_max
    hsup_min = aerostructures.hsup_min
    hsup_max = aerostructures.hsup_max

    sub_trans_spline = Cubic_Spline_Blender(hsub_min, hsub_max)
    sup_trans_spline = Cubic_Spline_Blender(hsup_max, hsup_min)
    h_sub = lambda M: sub_trans_spline.compute(M)
    h_sup = lambda M: sup_trans_spline.compute(M)

    # Per-wing structural deflection and twist surrogates (3D: AoA × Mach × node_idx)
    for wing in vehicle.wings:
        n_nodes = sub_sur.deflection_w[wing.tag].grid[2].shape[0]

        conditions.aerostructures[wing.tag]               = Data()
        conditions.aerostructures[wing.tag].deflection    = np.zeros((n_cpts, n_nodes, 3))
        conditions.aerostructures[wing.tag].elastic_twist = np.zeros((n_cpts, n_nodes, 1))

        for ti in range(n_cpts):
            aoa_ti  = float(AoA[ti])
            mach_ti = float(Mach[ti])
            # Build query array: one row per node, columns = (AoA, Mach, node_idx)
            node_pts = np.column_stack([
                np.full(n_nodes, aoa_ti),
                np.full(n_nodes, mach_ti),
                np.arange(n_nodes, dtype=float)
            ])

            conditions.aerostructures[wing.tag].deflection[ti, :, 0] = blend_structural(
                sub_sur.deflection_u[wing.tag], trans_sur.deflection_u[wing.tag],
                sup_sur.deflection_u[wing.tag], h_sub, h_sup, mach_ti, node_pts)
            conditions.aerostructures[wing.tag].deflection[ti, :, 1] = blend_structural(
                sub_sur.deflection_v[wing.tag], trans_sur.deflection_v[wing.tag],
                sup_sur.deflection_v[wing.tag], h_sub, h_sup, mach_ti, node_pts)
            conditions.aerostructures[wing.tag].deflection[ti, :, 2] = blend_structural(
                sub_sur.deflection_w[wing.tag], trans_sur.deflection_w[wing.tag],
                sup_sur.deflection_w[wing.tag], h_sub, h_sup, mach_ti, node_pts)
            conditions.aerostructures[wing.tag].elastic_twist[ti, :, 0] = blend_structural(
                sub_sur.elastic_twist[wing.tag], trans_sur.elastic_twist[wing.tag],
                sup_sur.elastic_twist[wing.tag], h_sub, h_sup, mach_ti, node_pts)
    return


# ----------------------------------------------------------------------------------------------------------------------
#  evaluate_no_surrogate
# ----------------------------------------------------------------------------------------------------------------------
def evaluate_no_surrogate(state, settings, vehicle):
    """Evaluate aerostructural response directly by calling VLM then FEA.

    VD (vortex distribution) must already be stored in
    state.analyses.aerodynamics.vortex_distribution after the VLM analysis setup.

    Args:
        state    : mission segment state
        settings : analysis settings
        vehicle  : vehicle configuration

    Returns:
        None — results written directly to state.conditions
    """
    conditions = state.conditions

    VLM_results = VLM(conditions, settings, vehicle)
    VD          = state.analyses.aerodynamics.vortex_distribution

    conditions.aerostructures = FEA(conditions, VLM_results, VD, settings, vehicle)
    return


# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def blend(sub_sur, trans_sur, sup_sur, h_sub, h_sup, Mach, pts):
    """Cubic-spline Mach blending for a scalar 2D surrogate (AoA × Mach)."""
    sub_val = np.atleast_1d(sub_sur(pts))
    if trans_sur is None and sup_sur is None:
        return h_sub(Mach) * sub_val
    trans_val = np.atleast_1d(trans_sur(pts))
    sup_val   = np.atleast_1d(sup_sur(pts))
    return (h_sub(Mach) * sub_val
            + (1 - h_sub(Mach) - h_sup(Mach)) * trans_val
            + h_sup(Mach) * sup_val)


def blend_structural(sub_sur, trans_sur, sup_sur, h_sub, h_sup, mach_scalar, pts):
    """Cubic-spline Mach blending for a 3D structural surrogate (AoA × Mach × node)."""
    sub_val = sub_sur(pts)
    if trans_sur is None and sup_sur is None:
        return h_sub(mach_scalar) * sub_val
    trans_val = trans_sur(pts)
    sup_val   = sup_sur(pts)
    return (h_sub(mach_scalar) * sub_val
            + (1 - h_sub(mach_scalar) - h_sup(mach_scalar)) * trans_val
            + h_sup(mach_scalar) * sup_val)
