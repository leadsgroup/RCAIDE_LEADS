# RCAIDE/Library/Methods/Powertrain/Propulsors/Common/compute_propulsor_moment.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import array_namespace

# ----------------------------------------------------------------------------------------------------------------------
#  compute_propulsor_moment
# ----------------------------------------------------------------------------------------------------------------------
def compute_propulsor_moment(origin, center_of_gravity, thrust):
    """
    Moment of a propulsor's thrust about the center of gravity at each control point, (origin - cg) x thrust, on NumPy
    or JAX; the origin and center of gravity may themselves be inputs (a design-dependent propulsor position).

    Inputs:
       origin              propulsor position, [[x, y, z]]                     [m]
       center_of_gravity   vehicle center of gravity, [[x, y, z]]              [m]
       thrust              thrust vector at each control point, (n, 3)         [N]

    Outputs:
       moment              (n, 3)                                              [N m]
    """
    xp  = array_namespace(thrust, *origin[0], *center_of_gravity[0])
    arm = xp.stack([xp.asarray(origin[0][i] - center_of_gravity[0][i], dtype=float) for i in range(3)])
    return xp.cross(arm[None, :] * xp.ones_like(thrust[:, :1]), thrust)
