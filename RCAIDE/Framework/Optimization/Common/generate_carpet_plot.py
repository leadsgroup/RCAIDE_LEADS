# RCAIDE/Framework/Optmizaition/Common/generate_carpet_plot.py 

# ----------------------------------------------------------------------------------------------------------------- 
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Framework.Core import Data
import numpy as np
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------------------------------------------------
#  generate_carpet_plot
# ---------------------------------------------------------------------------------------------------------------------- 
def generate_carpet_plot(problem,
                         design_input_1_index            = 0,
                         design_input_2_index            = 1,
                         number_of_points                = 5,
                         generate_objective_plot         = True,
                         objective_plot_constraint_index = 0,
                         generate_constraint_plots       = True):
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
    generate_constraint_plots        [bool]   if True, plot a separate contour for every constraint

    Outputs:
    outputs.inputs          [array]  (2, number_of_points) swept variable values
    outputs.objective       [array]  (number_of_points, number_of_points) objective values
    outputs.constraint_val  [array]  (n_constraints, number_of_points, number_of_points) constraint values

    Properties Used:
    N/A
    """

    # unpack
    idx0             = design_input_1_index
    idx1             = design_input_2_index
    opt_prob         = problem.optimization_problem
    base_inputs      = opt_prob.inputs
    names            = base_inputs[:, 0]
    bndl             = base_inputs[:, 2]
    bndu             = base_inputs[:, 3]
    base_objective   = opt_prob.objective
    obj_name         = base_objective[0][0]
    base_constraints = opt_prob.constraints
    constraint_names = base_constraints[:, 0]

    # define sweep arrays
    inputs         = np.zeros([2, number_of_points])
    obj            = np.zeros([number_of_points, number_of_points])
    constraint_num = np.shape(base_constraints)[0]
    constraint_val = np.zeros([constraint_num, number_of_points, number_of_points])

    inputs[0, :] = np.linspace(bndl[idx0], bndu[idx0], number_of_points)
    inputs[1, :] = np.linspace(bndl[idx1], bndu[idx1], number_of_points)

    # evaluate problem across sweep grid
    for i in range(number_of_points):
        for j in range(number_of_points):
            opt_prob.inputs[:, 1][idx0] = inputs[0, i]
            opt_prob.inputs[:, 1][idx1] = inputs[1, j]
            obj[j, i]                   = problem.objective()[0]
            constraint_val[:, j, i]     = problem.all_constraints().tolist()

    if generate_objective_plot:
        plt.figure(0)
        CS = plt.contourf(inputs[0, :], inputs[1, :], obj, linewidths=2)
        cbar = plt.colorbar(CS)
        cbar.ax.set_ylabel(obj_name)
        CS_con = plt.contour(inputs[0, :], inputs[1, :],
                             constraint_val[objective_plot_constraint_index, :, :],
                             colors='white', linewidths=1.5)
        plt.clabel(CS_con, inline=True, fontsize=8,
                   fmt={lv: constraint_names[objective_plot_constraint_index]
                        for lv in CS_con.levels})
        plt.xlabel(names[idx0])
        plt.ylabel(names[idx1])

    if generate_constraint_plots:
        for i in range(constraint_num):
            plt.figure(i + 1)
            CS_const = plt.contour(inputs[0, :], inputs[1, :], constraint_val[i, :, :])
            cbar = plt.colorbar(CS_const)
            cbar.ax.set_ylabel(constraint_names[i])
            plt.xlabel(names[idx0])
            plt.ylabel(names[idx1])

    plt.show(block=True)

    # pack outputs
    outputs                = Data()
    outputs.inputs         = inputs
    outputs.objective      = obj
    outputs.constraint_val = constraint_val

    return outputs
