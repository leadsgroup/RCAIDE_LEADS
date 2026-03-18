# RCAIDE/Framework/Optimization/Common/carpet_plot.py 

# ----------------------------------------------------------------------------------------------------------------- 
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Framework.Core import Data
import numpy as np
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------
#  carpet_plot
# ---------------------------------------------------------------------- 
def carpet_plot(problem,
                design_input_1_index            = 0, 
                design_input_2_index            = 1,                
                number_of_points                = 5,
                generate_objective_plot         = True, 
                objective_plot_constraint_index = None, 
                generate_constraint_plots       = True,): 
    """ Takes in an optimization problem and runs a carpet plot of the first 2 variables
        sweep_index_0, sweep_index_1 is index of variables you want to run carpet plot (i.e. sweep_index_0=0 means you want to sweep first variable, sweep_index_0 = 4 is the 5th variable)
    
        Assumptions:
        N/A
    
        Source:
        N/A
    
        Inputs:
        problem            [Nexus Class]
        number_of_points   [int]
        plot_objective           [int]
        plot_constraint         [int]
        sweep_index_0      [int]
        sweep_index_1      [int]
        
        Outputs:
        Beautiful Beautiful Plots!
            Outputs:
                inputs     [array]
                objective  [array]
                constraint [array]
    
        Properties Used:
        N/A
    """         

    # unpack 
    opt_prob        = problem.optimization_problem
    base_inputs     = opt_prob.inputs
    names           = base_inputs[:,0] # Names
    bndl            = base_inputs[:,2] # Bounds
    bndu            = base_inputs[:,3] # Bounds 
    units           = base_inputs[:,5] # Scaling
    base_objective  = opt_prob.objective
    obj_name        = base_objective[0][0] #objective function name (used for scaling)
    obj_scaling     = base_objective[0][1]
    base_constraints= opt_prob.constraints
    constraint_names= base_constraints[:,0]
    constraint_scale= base_constraints[:,3]
   
    #define inputs, output, and constraints for sweep
    inputs          = np.zeros([2,number_of_points])
    inputs_scaled   = np.zeros([2,number_of_points])
    obj             = np.zeros([number_of_points,number_of_points])
    constraint_num  = np.shape(base_constraints)[0] # of constraints
    constraint_val  = np.zeros([constraint_num,number_of_points,number_of_points]) 
    
    #create inputs matrix
    inputs[0,:] = np.linspace(bndl[design_input_1_index], bndu[design_input_1_index], number_of_points)
    inputs[1,:] = np.linspace(bndl[design_input_2_index], bndu[design_input_2_index], number_of_points)
    
    inputs_scaled[0,:] = inputs[0,:]*units[design_input_1_index]
    inputs_scaled[1,:] = inputs[1,:]*units[design_input_2_index]
    
    #inputs defined; now run sweep
    for i in range(0, number_of_points):
        for j in range(0,number_of_points): 
            opt_prob.inputs[:,1][design_input_1_index]= inputs[0,i]
            opt_prob.inputs[:,1][design_input_2_index]= inputs[1,j]
            objective            = problem.objective()*obj_scaling
            obj[i,j]             = objective[0]*obj_scaling 
            constraint           =  problem.all_constraints() 
            constraint_val[:,i,j]= constraint * constraint_scale
  
    if generate_objective_plot:
        plt.figure('Objective Plot') 
        CS   = plt.contourf(inputs_scaled[0,:],inputs_scaled[1,:], obj, linewidths=2)
        cbar = plt.colorbar(CS) 
        cbar.ax.set_ylabel(obj_name)
        
        if objective_plot_constraint_index !=  None:
            CS2 = plt.contour(inputs_scaled[0,:],inputs_scaled[1,:], constraint_val[objective_plot_constraint_index,:,:], linewidths=2)
            cbar2 = plt.colorbar(CS2) 
            cbar2.ax.set_ylabel(constraint_names[objective_plot_constraint_index]) 
            
        plt.xlabel(names[design_input_1_index])
        plt.ylabel(names[design_input_2_index])
       
    if generate_constraint_plots: 
        for i in range(0, constraint_num): 
            plt.figure(constraint_names[i]) 
            CS_const = plt.contour(inputs_scaled[0,:],inputs_scaled[1,:], constraint_val[i,:,:])
            cbar     = plt.colorbar(CS_const)
            cbar.ax.set_ylabel(constraint_names[i])
            plt.xlabel(names[design_input_1_index])
            plt.ylabel(names[design_input_2_index]) 
        
    plt.tight_layout()
    
    # pack outputs
    outputs= Data()
    outputs.inputs         = inputs_scaled
    outputs.objective      = obj
    outputs.constraints    = constraint_val
                 
    return outputs
    
    