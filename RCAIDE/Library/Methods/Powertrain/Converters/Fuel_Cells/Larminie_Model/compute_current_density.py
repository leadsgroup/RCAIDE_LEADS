# RCAIDE/Library/Methods/Powertrain/Converters/Fuel_Cells/Larminie_Model/compute_current_density.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units, array_namespace
from RCAIDE.Library.Methods.Powertrain.Converters.Fuel_Cells.Larminie_Model.compute_voltage import compute_voltage

import numpy as np

# current density bounds of a cell [A/m^2]
LOWER_CURRENT_DENSITY = 0.0001 / (Units.cm ** 2.)
UPPER_CURRENT_DENSITY = 1.2 / (Units.cm ** 2.)

# bisection halvings: the bracket ratio is 1.2e4, so 80 halvings reach the last bit of a current density
BISECTIONS = 80

# ----------------------------------------------------------------------------------------------------------------------
#  Cell power and its slope
# ----------------------------------------------------------------------------------------------------------------------
def cell_power(current_density, fuel_cell):
    """Power of one cell at a current density [W] (Larminie and Dicks voltage, see compute_voltage)."""
    return compute_voltage(fuel_cell, current_density) * current_density * fuel_cell.interface_area

def cell_power_slope(current_density, fuel_cell):
    """d(cell power)/d(current density) [W m^2/A], from the derivative of compute_voltage."""
    xp    = array_namespace(current_density)
    scale = 0.001 / (Units.cm ** 2.)                          # current density of 1 mA/cm^2 [A/m^2]
    r     = fuel_cell.r / (1000 * (Units.cm ** 2))
    i1    = current_density / scale
    dv_di = (-r - fuel_cell.A1 / i1 - fuel_cell.m * fuel_cell.n * xp.exp(fuel_cell.n * i1)) / scale
    return fuel_cell.interface_area * (compute_voltage(fuel_cell, current_density) + current_density * dv_di)

def bisect(function, lower, upper, xp):
    """Point where an increasing-to-decreasing sign change of function lies, by bisection on [lower, upper]."""
    def halve(iteration, bracket):
        lower, upper = bracket
        middle   = 0.5 * (lower + upper)
        positive = function(middle) > 0.0
        return xp.where(positive, middle, lower), xp.where(positive, upper, middle)
    bracket = (lower, upper)
    if xp is np:
        for iteration in range(BISECTIONS):
            bracket = halve(iteration, bracket)
    else:
        import jax
        bracket = jax.lax.fori_loop(0, BISECTIONS, halve, bracket)
    return 0.5 * (bracket[0] + bracket[1])

# ----------------------------------------------------------------------------------------------------------------------
#  compute_maximum_power_current_density
# ----------------------------------------------------------------------------------------------------------------------
def compute_maximum_power_current_density(fuel_cell, like=0.0):
    """
    Current density of the cell's maximum power within its bounds.

    The cell power v(i) i A with v = Eoc - r i - A1 ln(i) - m exp(n i) is strictly concave in i
    (P'' = A (-2 r - A1/i - m n exp(n i) (2 + n i)) < 0), so its slope falls through zero once; the maximum is found
    by bisection on the slope, or is a bound where the slope does not change sign.

    Inputs:
       fuel_cell   Larminie fuel cell (r, Eoc, A1, m, n, interface_area)
       like        array whose shape and backend the result takes (default: a float)

    Outputs:
       current density of maximum power [A/m^2]
    """
    xp    = array_namespace(like)
    lower = LOWER_CURRENT_DENSITY + 0 * xp.asarray(like, dtype=float)
    upper = UPPER_CURRENT_DENSITY + 0 * xp.asarray(like, dtype=float)
    peak  = bisect(lambda i: cell_power_slope(i, fuel_cell), lower, upper, xp)
    peak  = xp.where(cell_power_slope(upper, fuel_cell) >= 0.0, upper, peak)
    return xp.where(cell_power_slope(lower, fuel_cell) <= 0.0, lower, peak)

# ----------------------------------------------------------------------------------------------------------------------
#  compute_current_density
# ----------------------------------------------------------------------------------------------------------------------
def compute_current_density(fuel_cell, cell_power_demand):
    """
    Current density at which each cell delivers its power demand, on the low-current branch of the power curve.

    Below the maximum power the power rises monotonically with current density from the lower bound, so the demand is
    met at one current density, found by bisection; a demand above the maximum power is held at the maximum-power
    current density, and one below the lower bound's power at the lower bound. On JAX the result is differentiable
    with respect to the demand and the cell parameters: the implicit function theorem is applied to the power match,
    to the zero slope at maximum power, or to the bound (jax.lax.custom_root, one independent equation per point).

    Inputs:
       fuel_cell           Larminie fuel cell
       cell_power_demand   power demanded of one cell at each control point, 1-D [W]

    Outputs:
       current density at each control point [A/m^2]
    """
    xp        = array_namespace(cell_power_demand)
    peak      = compute_maximum_power_current_density(fuel_cell, cell_power_demand)
    lower     = LOWER_CURRENT_DENSITY + 0 * peak
    saturated = cell_power_demand >= cell_power(peak, fuel_cell)
    at_lower  = cell_power_demand <= cell_power(lower, fuel_cell)

    def solve(residual, guess):
        matched = bisect(lambda i: cell_power_demand - cell_power(i, fuel_cell), lower, peak, xp)
        return xp.where(saturated, peak, xp.where(at_lower, lower, matched))

    if xp is np:
        return solve(None, peak)

    import jax
    saturated, at_lower = jax.lax.stop_gradient(saturated), jax.lax.stop_gradient(at_lower)

    def residual(i):
        return xp.where(saturated, cell_power_slope(i, fuel_cell),
                        xp.where(at_lower, i - LOWER_CURRENT_DENSITY, cell_power(i, fuel_cell) - cell_power_demand))

    def tangent_solve(linear_map, y):
        return y / linear_map(xp.ones_like(y))

    return jax.lax.custom_root(residual, jax.lax.stop_gradient(peak), solve, tangent_solve)
