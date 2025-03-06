# RCAIDE/Library/Missions/Segments/converge_root.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# Package imports
import RCAIDE
from RCAIDE.Framework.Core import  Units, Data
from RCAIDE.Framework.Optimization.Packages.scipy import scipy_setup
 
import numpy as np 
# ----------------------------------------------------------------------------------------------------------------------
# converge root
# ---------------------------------------------------------------------------------------------------------------------- 
def converge_root(segment):
    """Interfaces the mission to a numerical solver. The solver may be changed by using root_finder.

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    segment                            [Data]
    segment.settings.root_finder       [Data]
    state.numerics.tolerance_solution  [Unitless]

    Outputs:
    state.unknowns                     [Any]
    segment.state.numerics.converged   [Unitless]

    Properties Used:
    N/A
    """       
    
    
    optimization_problem = add_mission_variables(optimization_problem,segment) 
    output   = scipy_setup.SciPy_Solve(optimization_problem,solver=solver_name, iter = iterations , sense_step = solver_sense_step,tolerance = solver_tolerance)      
    
    if ier!=1:
        print("Segment did not converge. Segment Tag: " + segment.tag)
        print("Error Message:\n" + msg)
        segment.state.numerics.converged = False
        segment.converged = False
    else:
        segment.state.numerics.converged = True
        segment.converged = True
                            
    return

def add_mission_variables(optimization_problem,segment):
    """Make a pretty table view of the problem with objective and constraints at the current inputs for the dummy solver
    

        Assumptions:
        N/A

        Source:
        N/A

        Inputs:
        x                  [vector]

        Outputs:
        input              [array]
        const_table        [array]

        Properties Used:
        None
    """             
    
    # unpack 
    inp = optimization_problem.inputs
    con = optimization_problem.constraints
    ali = optimization_problem.aliases
    
    # add the inputs
    input_count = 0
    vec = np.array([]) 
    
    # Change the mission solve to dummy solver
    print('Overwriting the solver in ' + segment + ' segment.')
    segment.settings.root_finder = RCAIDE.Library.Mission.Solver.dummy_mission_solver
    
    # Start putting together the inputs
    print ('Adding in the new inputs for ' + segment + '.')
    n_points     = segment.state.numerics.number_of_control_points
    unknown_keys = list(segment.state.unknowns.keys())
    unknown_keys.remove('tag')  
    len_inputs     = n_points*len(unknown_keys)
    unknown_value  = Data()
    full_unkn_vals = Data()
    for unkn in unknown_keys:
        unknown_value[unkn]  = segment.state.unknowns[unkn]
        full_unkn_vals[unkn] = unknown_value[unkn]*np.ones(n_points)

    # Basic construction
    # [Input_###, initial, (-np.inf, np.inf), initial, Units.less]
    initial_values    = full_unkn_vals.pack_array()
    input_len_strings = np.tile('Mission_Input_', len_inputs)
    input_numbers     = np.linspace(1,len_inputs,len_inputs,dtype=np.int16)
    input_names       = np.core.defchararray.add(input_len_strings,np.array(input_numbers+input_count).astype(str))
    bounds            = np.broadcast_to((-np.inf,np.inf),(len_inputs,2))
    units             = np.broadcast_to(Units.less,(len_inputs,))
    new_inputs        = np.reshape(np.tile(np.atleast_2d(np.array([None,None,(None,None),None,None])),len_inputs), (-1, 5))
    
    # Add in the inputs
    new_inputs[:,0]   = input_names 
    new_inputs[:,1]   = initial_values
    new_inputs[:,2]   = bounds.tolist()
    new_inputs[:,3]   = initial_values
    new_inputs[:,4]   = units
    inp               = np.concatenate((new_inputs,inp),axis=0)
    optimization_problem.inputs = inp
    
    # Create the equality constraints to the beginning of the constraints
    # all equality constraints are 0, scale 1, and unitless
    new_con = np.reshape(np.tile(np.atleast_2d(np.array([None,None,None,None,None])),len_inputs), (-1, 5))

    con_len_strings = np.tile('Residual_', len_inputs)
    con_names       = np.core.defchararray.add(con_len_strings,np.array(input_numbers+input_count).astype(str))
    equals          = np.broadcast_to('=',(len_inputs,))
    zeros           = np.zeros(len_inputs)
    ones            = np.ones(len_inputs)
    
    # Add in the new constraints
    new_con[:,0]    = con_names
    new_con[:,1]    = equals
    new_con[:,2]    = zeros
    new_con[:,3]    = ones
    new_con[:,4]    = units
    con             = np.concatenate((new_con,con),axis=0)
    optimization_problem.constraints = con
    
    # add the corresponding aliases
    # setup the aliases for the inputs
    output_numbers = np.linspace(0,n_points-1,n_points,dtype=np.int16)
    basic_string_con = Data()
    input_string = []
    for unkn in unknown_keys:
        basic_string_con[unkn] = np.tile('missions.' + mission_key + '.segments.' + segment + '.state.unknowns.'+unkn+'[', n_points)
        input_string.append(np.core.defchararray.add(basic_string_con[unkn],np.array(output_numbers).astype(str)))
    input_string  = np.ravel(input_string)
    input_string  = np.core.defchararray.add(input_string, np.tile(']',len_inputs))
    input_aliases = np.reshape(np.tile(np.atleast_2d(np.array((None,None))),len_inputs), (-1, 2))
                                  
    input_aliases[:,0] = input_names
    input_aliases[:,1] = input_string
    
    
    # setup the aliases for the residuals
    basic_string_res = np.tile('missions.' + mission_key + '.segments.' + segment + '.state.residuals.pack_array()[', len_inputs)
    residual_string  = np.core.defchararray.add(basic_string_res,np.array(input_numbers-1).astype(str))
    residual_string  = np.core.defchararray.add(residual_string, np.tile(']',len_inputs))
    residual_aliases = np.reshape(np.tile(np.atleast_2d(np.array((None,None))),len_inputs), (-1, 2))
    
    residual_aliases[:,0] = con_names
    residual_aliases[:,1] = residual_string
    
    # Put all the aliases in!
    for ii in range(len_inputs):
        ali.append(residual_aliases[ii].tolist())
        ali.append(input_aliases[ii].tolist())
        
    # The mission needs the state expanded now
    segment.process.initialize.expand_state(segment)
    
    # Update the count of inputs
    input_count = input_count+input_numbers[-1]            
        
    return                                
     

def dummy_mission_solver( iterate, unknowns, args=(), xtol = 0., full_output=1):
    """Rather than run a mission solver this solves a mission at a particular instance.

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    iterate      [suave function]
    unknowns     [inputs to the function]
    args         [[segment,state]]
    xtol         [irrelevant]
    full_output  [irrelevant]

    Outputs:
    unknowns     [inputs to the function]
    infodict     [None]
    ier          [1]
    msg          [string]

    Properties Used:
    N/A
    """       
    
    
    iterate(unknowns,args)
    
    infodict = None
    ier      = 1
    msg      = 'Used dummy mission solver'
    
    return unknowns, infodict, ier, msg
