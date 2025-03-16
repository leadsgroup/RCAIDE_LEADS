# RCAIDE/Library/Missions/Segments/converge.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import  Units, Data
from RCAIDE.Framework.Optimization.Packages.scipy import scipy_setup
from RCAIDE.Framework.Optimization.Common         import Nexus
from RCAIDE.Framework.Analyses.Process            import Process

import scipy 
import scipy.optimize
import numpy as np 
import sys 
import os 


# ----------------------------------------------------------------------------------------------------------------------
# converge root
# ---------------------------------------------------------------------------------------------------------------------- 
def converge(segment):
    """Interfaces the mission a root finder algorithm.

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
    
    if segment.state.numerics.solver.type  == "optimize": 
        problem  = add_mission_variables(segment) 
       
        # Commense suppression of console window output  
        devnull = open(os.devnull,'w')
        sys.stdout = devnull
         
        outputs  = scipy_setup.SciPy_Solve(problem,
                                           solver     = segment.state.numerics.solver.method,
                                           sense_step = segment.state.numerics.solver.step_size,
                                           iter       = segment.state.numerics.solver.max_evaluations,
                                           tolerance  = segment.state.numerics.solver.tolerance_solution)
    
        # Terminate suppression of console window output   
        sys.stdout = sys.__stdout__  
         
        if outputs[3] != 0:
            mission_converge = False        
            error_message =  outputs[4] 
        else:
            mission_converge = True
     
    elif segment.state.numerics.solver.type  == "root_finder": 
        unknowns = segment.state.unknowns.pack_array() 
         
        unknowns,infodict,ier,error_message = scipy.optimize.fsolve(iterate_root_finder,
                                             unknowns,
                                             args   = segment,
                                             xtol   = segment.state.numerics.solver.tolerance_solution,
                                             maxfev = segment.state.numerics.solver.max_evaluations,
                                             epsfcn = segment.state.numerics.solver.step_size,
                                             full_output = 1)
        
        if ier !=1:
            mission_converge = False
        else:
            mission_converge = True
            
    else: 
        raise Exception('undefined mission solver type')        
        
    if mission_converge == False:
        print("Segment did not converge. Segment Tag: " + segment.tag)
        print("Error Message:\n" + error_message)
        segment.state.numerics.solver.converged = False
        segment.converged = False
    else:
        segment.state.numerics.solver.converged = True
        segment.converged = True
                                
    return
    
# ---------------------------------------------------------------------------------------------------------------------- 
#  Helper Functions
# ---------------------------------------------------------------------------------------------------------------------- 
def iterate_root_finder(unknowns, segment):
    
    """Runs one iteration of of all analyses for the mission.

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    state.unknowns                [Data]
    segment.process.iterate       [Data]

    Outputs:
    residuals                     [Unitless]

    Properties Used:
    N/A
    """       
    if isinstance(unknowns,np.ndarray):
        segment.state.unknowns.unpack_array(unknowns)
    else:
        segment.state.unknowns = unknowns
        
    segment.process.iterate(segment)
    
    residuals = segment.state.residuals.pack_array()
        
    return residuals



def add_mission_variables(segment):
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
    
    # Step 1: Define Nexus
    nexus                        = Nexus()
    optimization_problem         = Data() 
    
    # Step 2 : Get segment type 
    ground_seg_flag =  (type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Landing) or\
                       (type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Takeoff) or \
                       (type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Ground) 
    constant_throttle_seg = type(segment) == RCAIDE.Framework.Mission.Segments.Cruise.Constant_Throttle_Constant_Altitude 
    single_pt_seg = (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude) or\
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_AVL_Trimmed) or \
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_No_Propulsion) or \
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Throttle) 
 
    
    # Step 2: Optimizer Inputs 
    # Step 2.1: Extract inputs
    input_count  = 0
    unknown_keys = list(segment.state.unknowns.keys())
    unknown_keys.remove('tag') 
    if ground_seg_flag: 
        n_points     = segment.state.numerics.number_of_control_points
        if len(unknown_keys) == 2: 
            len_inputs   = n_points
        else:
            len_inputs   = n_points +  (len(unknown_keys) - 2) * n_points
    elif constant_throttle_seg:
        n_points     = segment.state.numerics.number_of_control_points  
        len_inputs   = n_points*(len(unknown_keys)-1) + 1
    elif single_pt_seg:
        n_points   = 1
        len_inputs = len(unknown_keys)
    else:
        n_points     = segment.state.numerics.number_of_control_points  
        len_inputs = n_points*len(unknown_keys)
        
    unknown_value  = Data()
    full_unkn_vals = Data()
    for unkn in unknown_keys:
        unknown_value[unkn]  = segment.state.unknowns[unkn]
        full_unkn_vals[unkn] = unknown_value[unkn] 

    # Step 2.2: Construct nexus format  : [Variable_###, initial, -np.inf, np.inf , scaling, Units.less]
    initial_values    = full_unkn_vals.pack_array()
    input_len_strings = np.tile('Variable_', len_inputs)
    input_numbers     = np.linspace(1,len_inputs,len_inputs,dtype=np.int16)
    input_names       = np.core.defchararray.add(input_len_strings,np.array(input_numbers+input_count).astype(str))
    bounds            = np.broadcast_to((-np.inf,np.inf),(len_inputs,2))
    units             = np.broadcast_to(Units.less,(len_inputs,))
    new_inputs        = np.reshape(np.tile(np.atleast_2d(np.array([None,None,None,None,None,None])),len_inputs), (-1, 6))
    
    # Step 2.4 Add in the inputs 
    new_inputs[:,0]     = input_names   
    new_inputs[:,1]     = initial_values 
    new_inputs[:,2:4]   = bounds   
    new_inputs[:,4]     = 1
    new_inputs[:,5]     = units 
    optimization_problem.inputs = np.array(new_inputs,dtype=object)
    
    # Step 3: Constraints 
    # Step 3.1 : Create the equality constraints to the beginning of the constraints all equality constraints are 0, scale 1, and unitless
    new_con = np.reshape(np.tile(np.atleast_2d(np.array([None,None,None,None,None])),len_inputs), (-1, 5))

    con_len_strings = np.tile('Residual_', len_inputs)
    con_names       = np.core.defchararray.add(con_len_strings,np.array(input_numbers+input_count).astype(str))
    equals          = np.broadcast_to('=',(len_inputs,))
    zeros           = np.zeros(len_inputs)
    ones            = np.ones(len_inputs)
    
    # Step 3.2 Add in the new constraints
    new_con[:,0]    = con_names
    new_con[:,1]    = equals
    new_con[:,2]    = zeros  
    new_con[:,3]    = ones
    new_con[:,4]    = 1*Units.less
    optimization_problem.constraints =  np.array(new_con,dtype=object)            
    
    # Step 4. Aliases 
    # Step 4.1: Setup the aliases for the inputs
    basic_string_con = Data()
    input_string = []     

    if ground_seg_flag: 
        input_aliases      = np.reshape(np.tile(np.atleast_2d(np.array((None,None))),len_inputs), (-1, 2))         
        count = 0
        for unkn in unknown_keys: 
            basic_string_con = Data()
            input_string = []            
            if unkn == 'elapsed_time':
                output_numbers = np.array([0])
                basic_string_con[unkn] = np.tile('segment.state.unknowns.'+unkn , 1)
                input_string.append(np.core.defchararray.add(basic_string_con[unkn],np.array(output_numbers).astype(str))) 
                input_string       = np.core.defchararray.add(input_string, np.tile(']',1)) 
                input_aliases[count:count+1,1] = 'segment.state.unknowns.'+unkn
                count += 1
                
            elif unkn == 'ground_velocity': 
                output_numbers = np.linspace(0,n_points-2,n_points-1,dtype=np.int16)                    
                basic_string_con[unkn] = np.tile('segment.state.unknowns.'+unkn+'[', n_points-1)
                input_string.append(np.core.defchararray.add(basic_string_con[unkn],np.array(output_numbers).astype(str)))
                input_string       = np.core.defchararray.add(input_string, np.tile(']',n_points-1))  
                input_aliases[count:count+n_points-1,1] = input_string
                count += n_points-1 
            
            else: 
                output_numbers = np.linspace(0,n_points-1,n_points,dtype=np.int16)                     
                basic_string_con[unkn] = np.tile('segment.state.unknowns.'+unkn+'[', n_points)
                input_string.append(np.core.defchararray.add(basic_string_con[unkn],np.array(output_numbers).astype(str))) 
                input_string       = np.core.defchararray.add(input_string, np.tile(']',n_points))     
                input_aliases[count:count+n_points,1] = input_string
                count += n_points 
             
        input_aliases[:,0] = input_names
                
    elif constant_throttle_seg: 
        output_numbers = np.linspace(0,n_points-1,n_points,dtype=np.int16)        
        for unkn in unknown_keys[:-1]:
            basic_string_con[unkn] = np.tile('segment.state.unknowns.'+unkn+'[', n_points)
            input_string.append(np.core.defchararray.add(basic_string_con[unkn],np.array(output_numbers).astype(str)))
        input_string         = np.ravel(input_string)
        input_string         = np.core.defchararray.add(input_string, np.tile(']',len_inputs-1)) 
        input_aliases        = np.reshape(np.tile(np.atleast_2d(np.array((None,None))),len_inputs), (-1, 2)) 
        input_aliases[:,0]   = input_names   
        input_aliases[:-1,1] = input_string 
        input_aliases[-1,1]  = 'segment.state.unknowns.'+unknown_keys[-1]
        
    elif single_pt_seg:  
        for unkn in unknown_keys:
            basic_string_con[unkn] = np.tile('segment.state.unknowns.'+unkn+'[', n_points)
            input_string.append(np.core.defchararray.add(basic_string_con[unkn],np.array([0]).astype(str)))
        input_string       = np.ravel(input_string)
        input_string       = np.core.defchararray.add(input_string, np.tile(']',len_inputs))
        input_aliases      = np.reshape(np.tile(np.atleast_2d(np.array((None,None))),len_inputs), (-1, 2)) 
        input_aliases[:,0] = input_names
        input_aliases[:,1] = input_string    
    else:  
        output_numbers = np.linspace(0,n_points-1,n_points,dtype=np.int16) 
        for unkn in unknown_keys:
            basic_string_con[unkn] = np.tile('segment.state.unknowns.'+unkn+'[', n_points)
            input_string.append(np.core.defchararray.add(basic_string_con[unkn],np.array(output_numbers).astype(str)))
        input_string       = np.ravel(input_string)
        input_string       = np.core.defchararray.add(input_string, np.tile(']',len_inputs))
        input_aliases      = np.reshape(np.tile(np.atleast_2d(np.array((None,None))),len_inputs), (-1, 2)) 
        input_aliases[:,0] = input_names
        input_aliases[:,1] = input_string
    
    # Step 4.2: Setup the aliases for the residuals
    basic_string_res      = np.tile('segment.state.residuals.pack_array()[', len_inputs)
    residual_string       = np.core.defchararray.add(basic_string_res,np.array(input_numbers-1).astype(str))
    residual_string       = np.core.defchararray.add(residual_string, np.tile(']',len_inputs))
    residual_aliases      = np.reshape(np.tile(np.atleast_2d(np.array((None,None))),len_inputs), (-1, 2)) 
    residual_aliases[:,0] = con_names
    residual_aliases[:,1] = residual_string
        
    # Step 4.3: Append Aliases
    aliases = []
    for ii in range(len_inputs):
        aliases.append(input_aliases[ii].tolist()) 
        aliases.append(residual_aliases[ii].tolist())
    
    # Step 5: Objective function
    if segment.state.numerics.solver.objective == None:     
        aliases.append([ 'nothing'                   , 'postprocess.nothing']) 
        optimization_problem.objective = np.array([ [  'nothing'  ,  1   ,    1*Units.less]  ],dtype=object)            
    elif segment.state.numerics.solver.objective == "energy":
        aliases.append([ 'energy_consumed'          , 'postprocess.energy_consumed']) 
        optimization_problem.objective = np.array([ [  'energy_consumed'  ,  1   ,    1*Units.less]  ],dtype=object)            
    elif segment.state.numerics.solver.objective == "power":
        aliases.append([ 'maximum_power'          , 'postprocess.maximum_power'])
        optimization_problem.objective = np.array([ [  'maximum_power'  ,  1   ,    1*Units.less]  ],dtype=object)   
    else:
        raise Exception('undefined objective function')
    
    # append aliases 
    optimization_problem.aliases = aliases        
    
    # Step 6: Expand Rows  
    segment.process.initialize.expand_state(segment)
    
    # Step 7: Update iteration
    input_count = input_count+input_numbers[-1]      
     
    # Step 8: Append segment
    nexus.segment = segment
     
    # Step 9: Append procedure
    nexus.procedure = iterate_segment()
     
    # Step 9: Append post-process
    nexus.postprocess = Data()
    
    # Step 10: Append optimization problem 
    nexus.optimization_problem   = optimization_problem
    
    return nexus

def iterate_segment(): 
    procedure                           = Process()  
    procedure.segment                   = Process()
    procedure.segment.design_mission    = iterate_optimizer     
    procedure.post_process              = segment_post_process   
        
    return procedure
    
def iterate_optimizer(nexus):
    segment = nexus.segment
     
    unknowns = segment.state.unknowns.pack_array()
    if isinstance(unknowns,np.ndarray):
        segment.state.unknowns.unpack_array(unknowns)
    else:
        segment.state.unknowns = unknowns
        
    segment.process.iterate(segment)
    
    residuals = segment.state.residuals.pack_array()    
    nexus.residuals =  residuals
    return nexus


  
def segment_post_process(nexus):
    # unpack
    power      = nexus.segment.state.conditions.energy.power
    I          = nexus.segment.state.numerics.time.integrate
    
    # compute max power of segment 
    max_power  = np.max(nexus.segment.state.conditions.energy.power)
    
    # compute total energy consumed 
    if (type(nexus.segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude) or\
                    (type(nexus.segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_AVL_Trimmed) or \
                    (type(nexus.segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_No_Propulsion) or \
                    (type(nexus.segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Throttle): 
        energy_consumed =  0
    else:
        energy_consumed = np.dot(I,power)[-1][0]
    
    postprocess                 = nexus.postprocess
    postprocess.maximum_power   = max_power
    postprocess.energy_consumed = energy_consumed 
    postprocess.nothing         = 0
    
    return nexus  