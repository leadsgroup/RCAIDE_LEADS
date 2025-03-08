# RCAIDE/Library/Missions/Segments/converge_root.py
# 
# 
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import  Units, Data
from RCAIDE.Framework.Optimization.Packages.scipy import scipy_setup
from RCAIDE.Framework.Optimization.Common             import Nexus
from RCAIDE.Framework.Analyses.Process                     import Process

import scipy 
import scipy.optimize
import numpy as np 
import sys 
import subprocess
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
    
    if segment.state.numerics.solver  == "Optimize":
        unknowns = segment.state.unknowns.pack_array()
    
    
        problem  = add_mission_variables(segment)
    
    
        print_output = False
    
        # Initialize suppression of console window output
        if print_output == False:
            devnull = open(os.devnull,'w')
            sys.stdout = devnull
    
        outputs  = scipy_setup.SciPy_Solve(problem)
    
        # Terminate suppression of console window output  
        if print_output == False:
            sys.stdout = sys.__stdout__ 
    
        unknowns = outputs[0]
    
        if outputs[3] != 0:
            print("Segment did not converge. Segment Tag: " + segment.tag)
            print("Error Message:\n" + outputs[4])
            segment.state.numerics.converged = False
            segment.converged = False
        else:
            segment.state.numerics.converged = True
            segment.converged = True        
    
        
    if segment.state.numerics.solver  == "Root_Finder": 
        unknowns = segment.state.unknowns.pack_array()
        
        try:
            root_finder = segment.settings.root_finder
        except AttributeError:
            root_finder = scipy.optimize.fsolve 
         
        unknowns,infodict,ier,msg = root_finder(iterate_root_finder,
                                             unknowns,
                                             args = segment,
                                             xtol = segment.state.numerics.tolerance_solution,
                                             maxfev = segment.state.numerics.max_evaluations,
                                             epsfcn = segment.state.numerics.step_size,
                                             full_output = 1)
        
        if ier!=1:
            print("Segment did not converge. Segment Tag: " + segment.tag)
            print("Error Message:\n" + msg)
            segment.state.numerics.converged = False
            segment.converged = False
        else:
            segment.state.numerics.converged = True
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
    
    # unpack  
    nexus                        = Nexus()
    optimization_problem         = Data()
    
    # add the inputs
    input_count = 0
    
    # Start putting together the inputs 
    n_points     = segment.state.numerics.number_of_control_points
    unknown_keys = list(segment.state.unknowns.keys())
    unknown_keys.remove('tag')  
    len_inputs     = n_points*len(unknown_keys)
    unknown_value  = Data()
    full_unkn_vals = Data()
    for unkn in unknown_keys:
        unknown_value[unkn]  = segment.state.unknowns[unkn]
        full_unkn_vals[unkn] = unknown_value[unkn]#*np.ones(n_points)

    # Basic construction 
    # [Variable_###, initial, -np.inf, np.inf , scaling, Units.less]
    initial_values    = full_unkn_vals.pack_array()
    input_len_strings = np.tile('Variable_', len_inputs)
    input_numbers     = np.linspace(1,len_inputs,len_inputs,dtype=np.int16)
    input_names       = np.core.defchararray.add(input_len_strings,np.array(input_numbers+input_count).astype(str))
    bounds            = np.broadcast_to((-np.inf,np.inf),(len_inputs,2))
    units             = np.broadcast_to(Units.less,(len_inputs,))
    new_inputs        = np.reshape(np.tile(np.atleast_2d(np.array([None,None,None,None,None,None])),len_inputs), (-1, 6))
    
    # Add in the inputs 
    new_inputs[:,0]     = input_names   
    new_inputs[:,1]     = initial_values 
    new_inputs[:,2:4]   = bounds   
    new_inputs[:,4]     = 1 #initial_values
    new_inputs[:,5]     = units 
    optimization_problem.inputs = np.array(new_inputs,dtype=object)   
    
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
    new_con[:,2]    = zeros # segment.state.numerics.tolerance_solution
    new_con[:,3]    = ones
    new_con[:,4]    = 1*Units.less
    optimization_problem.constraints =  np.array(new_con,dtype=object)            
    
    # add the corresponding aliases
    # setup the aliases for the inputs
    output_numbers = np.linspace(0,n_points-1,n_points,dtype=np.int16)
    basic_string_con = Data()
    input_string = []
    for unkn in unknown_keys:
        basic_string_con[unkn] = np.tile('segment.state.unknowns.'+unkn+'[', n_points)
        input_string.append(np.core.defchararray.add(basic_string_con[unkn],np.array(output_numbers).astype(str)))
    input_string  = np.ravel(input_string)
    input_string  = np.core.defchararray.add(input_string, np.tile(']',len_inputs))
    input_aliases = np.reshape(np.tile(np.atleast_2d(np.array((None,None))),len_inputs), (-1, 2)) 
    input_aliases[:,0] = input_names
    input_aliases[:,1] = input_string
    
    # setup the aliases for the residuals
    basic_string_res = np.tile('segment.state.residuals.pack_array()[', len_inputs)
    residual_string  = np.core.defchararray.add(basic_string_res,np.array(input_numbers-1).astype(str))
    residual_string  = np.core.defchararray.add(residual_string, np.tile(']',len_inputs))
    residual_aliases = np.reshape(np.tile(np.atleast_2d(np.array((None,None))),len_inputs), (-1, 2))
    
    residual_aliases[:,0] = con_names
    residual_aliases[:,1] = residual_string
    
    # Put all the aliases in!
    aliases = []
    for ii in range(len_inputs):
        aliases.append(input_aliases[ii].tolist()) 
        aliases.append(residual_aliases[ii].tolist())
    
    aliases.append([ 'Nothing'                   , 'summary.Nothing']) # to change 
    
    optimization_problem.aliases = aliases        
        
    # The mission needs the state expanded now
    segment.process.initialize.expand_state(segment)
    
    # Update the count of inputs
    input_count = input_count+input_numbers[-1]      
    
    # TO CHANGE 
    optimization_problem.objective = np.array([ [  'Nothing'  ,  1   ,    1*Units.less]  ],dtype=object)    
     
    
    # -------------------------------------------------------------------
    #  Missions
    # -------------------------------------------------------------------
    nexus.segment = segment
    
    # -------------------------------------------------------------------
    #  Procedure
    # -------------------------------------------------------------------    
    nexus.procedure = iterate_segment()
    
    # -------------------------------------------------------------------
    #  Summary
    # ------------------------------------------------------------------- 
    nexus.summary = Data()
    
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
    summary =  nexus.summary 
    summary.Nothing =  0
    return nexus  