# RCAIDE/Framework/Optimization/Common/print_optimization_results.py 

# ----------------------------------------------------------------------------------------------------------------- 
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------- 
from RCAIDE.Framework.Core import Data
from .helper_functions import get_values, scale_obj_values, scale_const_values

# ----------------------------------------------------------------------
#  print_optimization_results
# ---------------------------------------------------------------------- 
def print_optimization_results(nexus,
                               generate_report = False,
                               filename = 'Optimization_Results'):
    """ Writes the optimization outputs to a file

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    nexus            [nexus()]
    filename         [str]

    Outputs:
    N/A

    Properties Used:
    N/A
    """       
 
    #unpack optimization problem values
    objective          = nexus.optimization_problem.objective
    aliases            = nexus.optimization_problem.aliases
    constraints        = nexus.optimization_problem.constraints
    
    #inputs
    inputs_names       = nexus.optimization_problem.inputs[:,0] 
    unscaled_inputs    = nexus.optimization_problem.inputs[:,1] 
    input_scaling      = nexus.optimization_problem.inputs[:,3]
    scaled_inputs      = unscaled_inputs/input_scaling
    
    #objective
    objective_value    = get_values(nexus,objective,aliases)
    scaled_objective   = scale_obj_values(objective , objective_value)
    
    #constraints
    constraint_values  = get_values(nexus,constraints,aliases) 
    scaled_constraints = scale_const_values(constraints,constraint_values)
    
    problem_inputs  = []
    problem_constraints = []
    for value in scaled_inputs:
        problem_inputs.append(value) 
    for value in scaled_constraints:
        problem_constraints.append(value)
         

    print("\t \n\n======== OPTIMIZATION REPORT ====================" )    
    print("\t------------------------------------------------------- "  )     
    print("\t Number of Iterations:", nexus.total_number_of_iterations )           
    print("\t------------------------------------------------------- "  )     
    print("\t Objective           :", objective[0],' : ', scaled_objective[0] )
    print("\t------------------------------------------------------- "  )     
    for i in range(len(inputs_names)): 
        print("\t Optimized Inputs    :", inputs_names[i], ' : ', problem_inputs[i])   
    print("\t------------------------------------------------------- "  )              
    print("\t Constaints          :", problem_constraints)   
    print("\t------------------------------------------------------- "  )       
   
    if generate_report:
        file = open( filename+ ".dat","w")     
        file.write('iteration = ')
        file.write(str(nexus.total_number_of_iterations))
        file.write(' , ')
        file.write('objective = ')
        file.write(str(scaled_objective[0]))
        file.write(', inputs = ')
        file.write(str(problem_inputs))
        file.write(', constraints = ')
        file.write(str(problem_constraints)) 
        file.write('\n') 
        file.close()
    
    return