# RCAIDE/Framework/Optimization/Common/line_plot.py 

# ----------------------------------------------------------------------------------------------------------------- 
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Framework.Core import Data
import numpy as np
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------
#  line_plot
# ---------------------------------------------------------------------- 
def line_plot(problem,
              design_input_1_index = 0, 
              number_of_points     = 10, 
              plot_objective       = True,
              plot_constraint      = True): 
    """
    Takes in an optimization problem and runs a line plot of the first  variable of sweep index
    sweep_index. i.e. sweep_index=0 means you want to sweep the first variable, sweep_index = 4 is the 5th variable)
    
        Assumptions:
        N/A
    
        Source:
        N/A
    
        Inputs:
        problem            [Nexus Class]
        number_of_points   [int]
        plot_objective           [int]
        plot_constraint         [int]
        sweep_index        [int]

        
        Outputs:
        Beautiful plots!
            Outputs:
                inputs     [array]
                objective  [array]
                constraint [array]
    
        Properties Used:
        N/A
    """          
    opt_prob         = problem.optimization_problem
    base_inputs      = opt_prob.inputs
    names            = base_inputs[:,0] # Names
    bndl             = base_inputs[:,2] # Bounds
    bndu             = base_inputs[:,3] # Bounds
    units           = base_inputs[:,5] # Scaling
    base_objective   = opt_prob.objective
    obj_name         = base_objective[0][0] #objective function name (used for scaling)
    obj_scaling      = base_objective[0][1]
    base_constraints = opt_prob.constraints
    constraint_names = base_constraints[:,0]
   
    #define inputs, output, and constraints for sweep
    inputs          = np.zeros([1,number_of_points])
    inputs_scaled   = np.zeros([1,number_of_points])
    obj             = np.zeros([number_of_points])
    constraint_num  = np.shape(base_constraints)[0] # of constraints
    constraint_val  = np.zeros([constraint_num,number_of_points])
    
    
    #create inputs matrix
    inputs[0,:] = np.linspace(bndl[design_input_1_index], bndu[design_input_1_index], number_of_points)
    inputs_scaled[0,:] = inputs[0,:]*units[design_input_1_index] 

    #inputs defined; now run sweep
    for i in range(0, number_of_points):
        opt_prob.inputs[:,1][design_input_1_index]= inputs[0,i]
        objective          = problem.objective()*obj_scaling 
        obj[i]             = objective[0]*obj_scaling
        constraint_val[:,i]= problem.all_constraints().tolist()
  
    if plot_objective==True:
        plt.figure(0)
        plt.plot(inputs[0,:], obj, lw = 2)
        plt.xlabel(names[design_input_1_index])
        plt.ylabel(obj_name) 

    if plot_constraint==True:
        for i in range(0, constraint_num):
            plt.figure(i+1)
            plt.plot(inputs[0,:], constraint_val[i,:], lw = 2)
            plt.xlabel(names[design_input_1_index])
            plt.ylabel(constraint_names[i]) 
        
        
    # pack outputs
    outputs= Data()
    outputs.inputs         = inputs_scaled
    outputs.objective      = obj
    outputs.constraints    = constraint_val
    
    return outputs
    
    