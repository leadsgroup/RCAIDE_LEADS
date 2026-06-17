# RCAIDE/Framework/Optimization/Common/generate_line_plot.py

# -----------------------------------------------------------------------------------------------------------------
#  IMPORT
# -----------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core                                      import Data
from RCAIDE.Framework.Optimization.Common.generate_carpet_plot import _fmt, _axis_label
import numpy              as np
import matplotlib.pyplot  as plt

# ----------------------------------------------------------------------------------------------------------------------
#  generate_line_plot
# ----------------------------------------------------------------------------------------------------------------------
def generate_line_plot(problem,
                       design_input_1_index = 0,
                       number_of_points     = 5,
                       plot_objective       = True,
                       plot_constraint      = True,
                       unit_labels          = None):
    """Sweeps one design variable across its bounds and produces line plots
    of the objective and constraints.

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    problem               [Nexus]  optimization problem
    design_input_1_index  [int]    column index of the design variable to sweep
    number_of_points      [int]    number of evaluation points along the sweep
    plot_objective        [bool]   if True, plot the objective vs the swept variable
    plot_constraint       [bool]   if True, plot each constraint vs the swept variable
    unit_labels           [list]   optional unit strings aligned with problem.inputs rows,
                                  e.g. ['m²', 'km', 'nmi']. If None, tag name only is shown.

    Outputs:
    outputs.inputs          [array]  (2, number_of_points) swept variable values
    outputs.objective       [array]  (number_of_points,) objective values
    outputs.constraint_val  [array]  (n_constraints, number_of_points) constraint values

    Properties Used:
    N/A
    """

    idx0      = design_input_1_index
    opt_prob  = problem.optimization_problem
    inp       = opt_prob.inputs
    names     = inp[:, 0]
    con_names = opt_prob.constraints[:, 0]
    obj_name  = opt_prob.objective[0][0]
    n_con     = len(con_names)
    units     = unit_labels if unit_labels is not None else [None] * len(names)

    x   = np.linspace(float(inp[idx0, 2]), float(inp[idx0, 3]), number_of_points)
    obj = np.zeros(number_of_points)
    con = np.zeros((n_con, number_of_points))

    for i in range(number_of_points):
        inp[idx0, 1] = x[i]
        obj[i]       = problem.objective()[0]
        con[:, i]    = problem.all_constraints()

    x_lbl    = _axis_label(names[idx0], units[idx0])
    con_lbls = [_fmt(n) for n in con_names]

    if plot_objective:
        fig, ax = plt.subplots(num=0)
        ax.plot(x, obj, lw=2)
        ax.set_xlabel(x_lbl)
        ax.set_ylabel(_fmt(obj_name))
        fig.tight_layout()

    if plot_constraint:
        for i in range(n_con):
            fig, ax = plt.subplots(num=i + 1)
            ax.plot(x, con[i], lw=2)
            ax.axhline(0, color='gray', linewidth=1, linestyle='--')
            ax.set_xlabel(x_lbl)
            ax.set_ylabel(con_lbls[i])
            fig.tight_layout()

    plt.show(block=True)

    outputs                = Data()
    outputs.inputs         = np.vstack([x, np.zeros_like(x)])
    outputs.objective      = obj
    outputs.constraint_val = con
    return outputs
