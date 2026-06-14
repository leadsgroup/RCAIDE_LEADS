# RCAIDE/Framework/Optmizaition/Common/generate_line_plot.py 

# ----------------------------------------------------------------------------------------------------------------- 
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Framework.Core import Data
import numpy as np
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------------------------------------------------------
#  generate_line_plot
# ---------------------------------------------------------------------------------------------------------------------- 
def generate_line_plot(problem,
                       design_input_1_index = 0,
                       number_of_points     = 5,
                       plot_objective       = True,
                       plot_constraint      = True):
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

    Outputs:
    outputs.inputs          [array]  (2, number_of_points) swept variable values
    outputs.objective       [array]  (number_of_points,) objective values
    outputs.constraint_val  [array]  (n_constraints, number_of_points) constraint values

    Properties Used:
    N/A
    """

    # unpack
    idx0             = design_input_1_index
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
    obj            = np.zeros([number_of_points])
    constraint_num = np.shape(base_constraints)[0]
    constraint_val = np.zeros([constraint_num, number_of_points])

    inputs[0, :] = np.linspace(bndl[idx0], bndu[idx0], number_of_points)

    # evaluate problem across sweep
    for i in range(number_of_points):
        opt_prob.inputs[:, 1][idx0] = inputs[0, i]
        obj[i]                      = problem.objective()[0]
        constraint_val[:, i]        = problem.all_constraints().tolist()

    if plot_objective:
        plt.figure(0)
        plt.plot(inputs[0, :], obj, lw=2)
        plt.xlabel(names[idx0])
        plt.ylabel(obj_name)

    if plot_constraint:
        for i in range(constraint_num):
            plt.figure(i + 1)
            plt.plot(inputs[0, :], constraint_val[i, :], lw=2)
            plt.xlabel(names[idx0])
            plt.ylabel(constraint_names[i])

    plt.show(block=True)

    # pack outputs
    outputs                = Data()
    outputs.inputs         = inputs
    outputs.objective      = obj
    outputs.constraint_val = constraint_val

    return outputs
