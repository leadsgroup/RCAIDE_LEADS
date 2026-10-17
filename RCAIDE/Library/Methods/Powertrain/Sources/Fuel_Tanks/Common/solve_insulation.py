# RCAIDE/Library/Methods/Powertrain/Sources/Fuel_Tanks/Common/solve_insulation.py
#
# Created: Sep 2026, RCAIDE Team

from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Common.find_root import find_root, bracket_root

# ----------------------------------------------------------------------------------------------------------------------
#  Insulation thickness solver
#
#  Brackets the root first (geometric expansion), then solves with find_root.
#  residual_func is the geometry-specific insulation residual (cylindrical vs.
#  cuboid); geometry_args are threaded through to it unchanged.
# ----------------------------------------------------------------------------------------------------------------------
def solve_insulation(residual_func, therm, fuel_tank, *geometry_args):
    ins_args = (therm, fuel_tank) + geometry_args
    bracket  = bracket_root(residual_func, start=1e-6, factor=5, limit=1e2, args=ins_args)
    if bracket:
        return find_root(residual_func, bracket[0], bracket[1], args=ins_args, xtol=1e-9)
    return find_root(residual_func, 1e-6, 1e2, args=ins_args, xtol=1e-9)
