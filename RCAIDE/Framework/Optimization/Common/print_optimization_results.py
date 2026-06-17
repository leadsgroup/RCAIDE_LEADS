# RCAIDE/Framework/Optimization/Common/print_optimization_results.py

# -----------------------------------------------------------------------------------------------------------------
#  IMPORT
# -----------------------------------------------------------------------------------------------------------------
from .helper_functions import get_values

# ----------------------------------------------------------------------------------------------------------------------
#  print_optimization_results
# ----------------------------------------------------------------------------------------------------------------------
def print_optimization_results(nexus):
    """Prints a formatted summary of the optimization problem state to the console.

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    nexus   [Nexus]  optimization problem instance

    Outputs:
    N/A  (prints to stdout)

    Properties Used:
    N/A
    """

    opt_prob    = nexus.optimization_problem
    inputs      = opt_prob.inputs
    aliases     = opt_prob.aliases
    objective   = opt_prob.objective
    constraints = opt_prob.constraints

    obj_value  = get_values(nexus, objective,   aliases)[0]
    con_values = get_values(nexus, constraints, aliases)

    W = 64

    feasible = all(
        _is_feasible(con_values[i], constraints[i, 1], float(constraints[i, 2]))
        for i in range(len(constraints))
    )
    status = 'SUCCESSFUL' if feasible else 'CONSTRAINTS VIOLATED'

    print()
    print('=' * W)
    print(f"{'  OPTIMIZATION RESULTS':<{W}}")
    print('=' * W)
    print(f"  Status     : {status}")
    print(f"  Iterations : {nexus.total_number_of_iterations}")

    # ------------------------------------------------------------------
    # Design Inputs
    # ------------------------------------------------------------------
    print(f"\n  {'DESIGN INPUTS'}")
    print('  ' + '-' * (W - 2))
    print(f"  {'Variable':<24}  {'Value':>12}   {'Lower':>10} - {'Upper':<10}")
    print('  ' + '-' * (W - 2))
    for row in inputs:
        tag = _fmt(row[0])
        val, lb, ub = float(row[1]), float(row[2]), float(row[3])
        print(f"  {tag:<24}  {val:>12.4f}   {lb:>10.4f} - {ub:<10.4f}")

    # ------------------------------------------------------------------
    # Objective
    # ------------------------------------------------------------------
    print(f"\n  {'OBJECTIVE'}")
    print('  ' + '-' * (W - 2))
    print(f"  {_fmt(objective[0][0]):<24}  {obj_value:>12.4f}")

    # ------------------------------------------------------------------
    # Constraints
    # ------------------------------------------------------------------
    print(f"\n  {'CONSTRAINTS'}")
    print('  ' + '-' * (W - 2))
    print(f"  {'Constraint':<24}  {'Value':>10}  {'':>2}  {'Limit':>10}  {'Status'}")
    print('  ' + '-' * (W - 2))
    for i, row in enumerate(constraints):
        tag, sense, edge = row[0], row[1], float(row[2])
        val    = con_values[i]
        status = 'SATISFIED' if _is_feasible(val, sense, edge) else 'VIOLATED '
        print(f"  {_fmt(tag):<24}  {val:>10.4f}  {sense:>2}  {edge:>10.4f}  {status}")

    print('=' * W)
    print()

    return


def _fmt(tag):
    return tag.replace('_', ' ').title()


def _is_feasible(value, sense, edge):
    if sense == '>':
        return value >= edge
    if sense == '<':
        return value <= edge
    return abs(value - edge) < 1e-6
