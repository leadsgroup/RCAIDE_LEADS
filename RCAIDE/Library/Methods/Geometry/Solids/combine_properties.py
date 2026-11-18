# RCAIDE/Library/Methods/Geometry/Solids/combine_properties.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Data, array_namespace

# ----------------------------------------------------------------------------------------------------------------------
#  combine_properties
# ----------------------------------------------------------------------------------------------------------------------
def combine_properties(parts):
    """
    Volume, center of gravity and inertia per unit mass of several solids of the same density taken as one.

    Parameters
    ----------
    parts : list of Data
        Each with volume, center_of_gravity (3,) and inertia_per_mass (3, 3) about its own center.

    Returns
    -------
    properties : Data
        The same fields for the whole; each part's inertia is moved to the common center (parallel axis theorem).
    """
    xp      = array_namespace(parts[0].volume, parts[0].center_of_gravity)
    volume  = sum(part.volume for part in parts)
    center  = sum(part.volume * part.center_of_gravity for part in parts) / volume
    inertia = 0. * parts[0].inertia_per_mass
    for part in parts:
        d       = part.center_of_gravity - center
        inertia = inertia + part.volume * (part.inertia_per_mass + xp.dot(d, d) * xp.eye(3) - xp.outer(d, d))
    return Data(volume=volume, center_of_gravity=center, inertia_per_mass=inertia / volume)


def mirror_properties(part, axis=1):
    """The solid reflected across the plane normal to axis (1: the xz plane of symmetry)."""
    xp     = array_namespace(part.volume, part.center_of_gravity)
    signs  = xp.asarray([-1. if i == axis else 1. for i in range(3)])
    return Data(volume=part.volume, center_of_gravity=part.center_of_gravity * signs,
                inertia_per_mass=part.inertia_per_mass * xp.outer(signs, signs))


def scale_properties(part, volume_fraction):
    """The solid scaled about its center of gravity so its volume is volume_fraction of the original."""
    return Data(volume=part.volume * volume_fraction, center_of_gravity=part.center_of_gravity,
                inertia_per_mass=part.inertia_per_mass * volume_fraction ** (2. / 3.))
