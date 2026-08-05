# RCAIDE/Framework/Optimization/Common/generate_carpet_plot.py

# -----------------------------------------------------------------------------------------------------------------
#  IMPORT
# -----------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Data
from matplotlib.lines      import Line2D
import numpy               as np
import matplotlib.pyplot   as plt

# ----------------------------------------------------------------------------------------------------------------------
#  generate_carpet_plot
# ----------------------------------------------------------------------------------------------------------------------
def generate_carpet_plot(problem,
                         design_input_1_index            = 0,
                         design_input_2_index            = 1,
                         number_of_points                = 5,
                         generate_objective_plot         = True,
                         objective_plot_constraint_index = 0,
                         generate_constraint_plots       = True,
                         unit_labels                     = None):
    """Sweeps two design variables across their bounds and produces contour plots
    of the objective and constraints (carpet plots).

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    problem                          [Nexus]  optimization problem
    design_input_1_index             [int]    column index of the first design variable to sweep
    design_input_2_index             [int]    column index of the second design variable to sweep
    number_of_points                 [int]    number of evaluation points along each axis
    generate_objective_plot          [bool]   if True, plot the objective contour with constraint overlay
    objective_plot_constraint_index  [int]    index of the constraint to overlay on the objective plot
    generate_constraint_plots        [bool]   if True, plot a separate filled contour for every constraint
    unit_labels                      [list]   optional unit strings aligned with problem.inputs rows,
                                             e.g. ['m²', 'km', 'nmi']. If None, tag name only is shown.

    Outputs:
    outputs.inputs          [array]  (2, number_of_points) swept variable values
    outputs.objective       [array]  (number_of_points, number_of_points) objective values
    outputs.constraint_val  [array]  (n_constraints, number_of_points, number_of_points) constraint values

    Properties Used:
    N/A
    """

    idx0, idx1   = design_input_1_index, design_input_2_index
    opt_prob     = problem.optimization_problem
    inp          = opt_prob.inputs
    names        = inp[:, 0]
    con_names    = opt_prob.constraints[:, 0]
    obj_name     = opt_prob.objective[0][0]
    n_con        = len(con_names)
    units        = unit_labels if unit_labels is not None else [None] * len(names)

    # sweep grids
    x   = np.linspace(float(inp[idx0, 2]), float(inp[idx0, 3]), number_of_points)
    y   = np.linspace(float(inp[idx1, 2]), float(inp[idx1, 3]), number_of_points)
    obj = np.zeros((number_of_points, number_of_points))
    con = np.zeros((n_con, number_of_points, number_of_points))

    for i in range(number_of_points):
        for j in range(number_of_points):
            inp[idx0, 1] = x[i]
            inp[idx1, 1] = y[j]
            obj[j, i]    = problem.objective()[0]
            con[:, j, i] = problem.all_constraints()

    x_lbl    = _axis_label(names[idx0], units[idx0])
    y_lbl    = _axis_label(names[idx1], units[idx1])
    con_lbls = [_fmt(n) for n in con_names]

    if generate_objective_plot:
        fig, ax = plt.subplots()
        CS = ax.contourf(x, y, obj)
        fig.colorbar(CS, ax=ax).ax.set_ylabel(_fmt(obj_name))
        CS_con = ax.contour(x, y, con[objective_plot_constraint_index],
                            colors='white', linewidths=1.5, linestyles='--')
        ax.clabel(CS_con, inline=True, fontsize=8, fmt='%.2f')
        ax.legend(handles=[Line2D([0], [0], color='white', lw=1.5, ls='--',
                                  label=con_lbls[objective_plot_constraint_index])],
                  loc='best', framealpha=0.3, facecolor='dimgray',
                  edgecolor='none', labelcolor='white')
        ax.set_xlabel(x_lbl)
        ax.set_ylabel(y_lbl)
        fig.tight_layout()

    if generate_constraint_plots:
        for i in range(n_con):
            fig, ax = plt.subplots()
            CS_c = ax.contourf(x, y, con[i])
            fig.colorbar(CS_c, ax=ax).ax.set_ylabel(con_lbls[i])
            if con[i].min() <= 0 <= con[i].max():
                ax.contour(x, y, con[i], levels=[0],
                           colors='white', linewidths=2, linestyles='--')
            ax.set_xlabel(x_lbl)
            ax.set_ylabel(y_lbl)
            fig.tight_layout()

    plt.show(block=True)

    outputs                = Data()
    outputs.inputs         = np.vstack([x, y])
    outputs.objective      = obj
    outputs.constraint_val = con
    return outputs


def _fmt(tag):
    return tag.replace('_', ' ').title()


def _axis_label(tag, unit=None):
    return f'{_fmt(tag)} ({unit})' if unit else _fmt(tag)
