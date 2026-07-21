# RCAIDE/Library/Methods/Aerostructures/Finite_Element_Analysis/build_FEA_surrogate.py
#
# Created: Mar 2026, M. Clarke, S. Sharma

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Data

from scipy.interpolate import RegularGridInterpolator
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  build_FEA_surrogates  (entry point called from train_FEA_surrogate.py)
# ----------------------------------------------------------------------------------------------------------------------
def build_FEA_surrogates(aerostructures, vehicle):
    """Wrap training data into interpolator objects for all Mach regimes.

    Expected training data structure (populated by train_FEA_surrogate.py):
        aerostructures.training.Mach                          shape (n_Mach,)
        aerostructures.training.angle_of_attack               shape (n_AoA,)
        aerostructures.training.subsonic / .supersonic / .transonic
            .Clift_alpha                                      shape (n_AoA, n_Mach)
            .Cdrag_induced_alpha                              shape (n_AoA, n_Mach)
            .Clift_wing_alpha[wing.tag]                       shape (n_AoA, n_Mach)
            .Cdrag_induced_wing_alpha[wing.tag]               shape (n_AoA, n_Mach)
            .deflection_u[wing.tag]                           shape (n_AoA, n_Mach, n_nodes)
            .deflection_v[wing.tag]                           shape (n_AoA, n_Mach, n_nodes)
            .deflection_w[wing.tag]                           shape (n_AoA, n_Mach, n_nodes)
            .elastic_twist[wing.tag]                          shape (n_AoA, n_Mach, n_nodes)
    """
    surrogates = aerostructures.surrogates
    training   = aerostructures.training
    Mach       = aerostructures.training.Mach
    sub_len    = int(np.sum(Mach < 1.0))
    sup_Mach   = Mach[sub_len:]

    surrogates.subsonic = build_surrogate(aerostructures, training.subsonic, vehicle)

    if len(sup_Mach) > 2:
        surrogates.supersonic = build_surrogate(aerostructures, training.supersonic, vehicle)
        surrogates.transonic  = build_surrogate(aerostructures, training.transonic,  vehicle)
    else:
        surrogates.supersonic = no_surrogate(vehicle)
        surrogates.transonic  = no_surrogate(vehicle)
    return


# ----------------------------------------------------------------------------------------------------------------------
#  build_surrogate  (one Mach regime)
# ----------------------------------------------------------------------------------------------------------------------
def build_surrogate(aerostructures, training, vehicle):
    """Build RegularGridInterpolator objects for one Mach regime."""
    surrogates = Data()
    mach_data  = training.Mach
    AoA_data   = aerostructures.training.angle_of_attack

    # Per-wing structural deflection/twist surrogates (3D: AoA × Mach × node_idx)
    surrogates.deflection_u  = Data()
    surrogates.deflection_v  = Data()
    surrogates.deflection_w  = Data()
    surrogates.elastic_twist = Data()

    for wing in vehicle.wings:
        n_nodes  = training.deflection_w[wing.tag].shape[2]
        node_idx = np.arange(n_nodes, dtype=float)

        surrogates.deflection_u[wing.tag] = RegularGridInterpolator(
            (AoA_data, mach_data, node_idx), training.deflection_u[wing.tag],
            method='linear', bounds_error=False, fill_value=None)
        surrogates.deflection_v[wing.tag] = RegularGridInterpolator(
            (AoA_data, mach_data, node_idx), training.deflection_v[wing.tag],
            method='linear', bounds_error=False, fill_value=None)
        surrogates.deflection_w[wing.tag] = RegularGridInterpolator(
            (AoA_data, mach_data, node_idx), training.deflection_w[wing.tag],
            method='linear', bounds_error=False, fill_value=None)
        surrogates.elastic_twist[wing.tag] = RegularGridInterpolator(
            (AoA_data, mach_data, node_idx), training.elastic_twist[wing.tag],
            method='linear', bounds_error=False, fill_value=None)

    return surrogates


# ----------------------------------------------------------------------------------------------------------------------
#  no_surrogate  (placeholder when insufficient Mach points exist)
# ----------------------------------------------------------------------------------------------------------------------
def no_surrogate(vehicle):
    """Return a surrogate container of None values for an unavailable Mach regime."""
    surrogates = Data()
    surrogates.deflection_u  = Data()
    surrogates.deflection_v  = Data()
    surrogates.deflection_w  = Data()
    surrogates.elastic_twist = Data()

    for wing in vehicle.wings:
        surrogates.deflection_u[wing.tag]  = None
        surrogates.deflection_v[wing.tag]  = None
        surrogates.deflection_w[wing.tag]  = None
        surrogates.elastic_twist[wing.tag] = None

    return surrogates
