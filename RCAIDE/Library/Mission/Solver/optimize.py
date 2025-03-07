# optimize.py
# 
# Created:  Dec 2016, E. Botero
# Modified: Jun 2017, E. Botero
#           Mar 2020, M. Clarke
#           Apr 2020, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import scipy.optimize as opt
import numpy as np  

# ----------------------------------------------------------------------
#  Converge Root
# ----------------------------------------------------------------------
def converge_opt(segment):
    """Interfaces the mission to an optimization algorithm

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    state.unknowns                     [Data]
    segment                            [Data]
    segment.algorithm                  [string]

    Outputs:
    state.unknowns                     [Any]

    Properties Used:
    N/A
    """     
    
    # pack up the array
    unknowns = segment.state.unknowns.pack_array()
    
    # Have the optimizer call the wrapper
    obj       = lambda unknowns:get_objective(unknowns,segment)   
    econ      = lambda unknowns:get_econstraints(unknowns,segment)
    
    # Setup the bnds of the problem
    bnds = make_bnds(unknowns, segment)
    
    # Solve the problem, based on chosen algorithm
    #if segment.algorithm == 'SLSQP':
    unknowns = opt.fmin_slsqp(obj,unknowns,f_eqcons=econ,bounds=bnds,iter=2000)
          
    return
    
# ----------------------------------------------------------------------
#  Helper Functions
# ----------------------------------------------------------------------
    
def get_objective(unknowns, segment):
    """ Runs the mission if the objective value is needed
    
        Assumptions:
        N/A
        
        Inputs:
        state.unknowns      [Data]
    
        Outputs:
        objective           [float]

        Properties Used:
        N/A
                                
    """      
    
    if isinstance(unknowns,np.ndarray):
        segment.state.unknowns.unpack_array(unknowns)
    else:
        segment.state.unknowns = unknowns
        
    if not np.all(segment.state.inputs_last == segment.state.unknowns):       
        segment.process.iterate(segment)
        
    objective = segment.state.objective_value
    
    return objective

def get_econstraints(unknowns, segment):
    """ Runs the mission if the equality constraint values are needed
    
        Assumptions:
        N/A
        
        Inputs:
        state.unknowns      [Data]
            
        Outputs:
        constraints          [array]

        Properties Used:
        N/A
                                
    """       
    
    if isinstance(unknowns,np.ndarray):
        segment.state.unknowns.unpack_array(unknowns)
    else:
        segment.state.unknowns = unknowns
        
    if not np.all(segment.state.inputs_last == segment.state.unknowns):       
        segment.process.iterate(segment)

    constraints = segment.state.constraint_values
    
    return constraints

def make_bnds(unknowns, segment):
    """ Automatically sets the bounds of the optimization.
    
        Assumptions:
        Restricts throttle to between 0 and 100%
        Restricts body angle from 0 to pi/2 radians
        Restricts flight path angle from 0 to pi/2 radians
        
        Inputs:
        none
            
        Outputs:
        bnds

        Properties Used:
        N/A
                                
    """      

    ones    = segment.state.ones_row(1)
    ones_m1 = segment.state.ones_row_m1(1).resize(segment.state._size)
    ones_m2 = segment.state.ones_row_m2(1).resize(segment.state._size)
    
    throttle_bnds = ones*(0.,1.)
    body_angle    = ones*(0., np.pi/2.)
    gamma         = ones*(0., np.pi/2.) 
    
    bnds = np.vstack([throttle_bnds,body_angle])
    
    bnds = list(map(tuple, bnds))
    

    #n_points     = segment.state.numerics.number_of_control_points
    #unknown_keys = list(segment.state.unknowns.keys())
    #unknown_keys.remove('tag')  
    #len_inputs     = n_points*len(unknown_keys)    
    #bnds            = np.broadcast_to((-np.inf,np.inf),(len_inputs,2))
    #bnds = list(map(tuple, bnds))
    return bnds