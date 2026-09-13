# RCAIDE/Library/Mission/Solver/solver.py
#
#
# Created:  Jul 2023, M. Clarke  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import  Units, Data
from RCAIDE.Framework.Optimization.Packages.scipy import scipy_setup
from RCAIDE.Framework.Optimization.Packages.pyopt import pyopt_setup
from RCAIDE.Framework.Optimization.Common  import Nexus
from RCAIDE.Framework.Analyses.Process            import Process

import scipy
import scipy.optimize
import numpy as np
import sys
import os

# ----------------------------------------------------------------------------------------------------------------------
# converge 
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
    state.unknowns.mission                     [Any]
    segment.state.numerics.converged   [Unitless]

    Properties Used:
    N/A
    """ 
    numerics = segment.state.numerics 
    total_unknowns = segment.state.number_of_mission_unknowns
    if segment.state.numerics.network_solver.type is None:
        total_unknowns += segment.state.number_of_network_unknowns
    if total_unknowns == 0:
        segment.process.iterate(segment)
        numerics.mission_solver.converged = True
        segment.converged = True
        return

    if numerics.mission_solver.type  == "optimize":
        problem  = add_mission_variables(segment)
        package  = getattr(numerics.mission_solver, 'package', 'scipy')

        # Comment suppression of console window output.  
        if numerics.mission_solver.verbose == False:
            devnull = open(os.devnull,'w')
            sys.stdout = devnull

        try:
            if package == "scipy":
                outputs  = scipy_setup.SciPy_Solve(problem,
                                                   solver     = numerics.mission_solver.method,
                                                   sense_step = numerics.mission_solver.step_size,
                                                   iter       = numerics.mission_solver.max_evaluations,
                                                   tolerance  = numerics.mission_solver.tolerance)

                if outputs[3] != 0:
                    mission_converge = False
                    error_message =  outputs[4]
                else:
                    mission_converge = True
                    error_message    = ""

            elif package == "pyopt": 
                outputs = pyopt_setup.Pyoptsparse_Solve(problem,
                                                        solver     = numerics.mission_solver.method,
                                                        sense_step = numerics.mission_solver.step_size)

                input_names = problem.optimization_problem.inputs[:,0]
                x_star      = np.array([np.atleast_1d(outputs.xStar[name])[0] for name in input_names], dtype=float)
                problem.evaluate(x_star) 
                residual  = np.atleast_1d(problem.equality_constraint(x_star))
                converged_residual = (residual.size == 0) or np.all(np.abs(residual) <= numerics.mission_solver.tolerance)

                if outputs.optInform is not None and outputs.optInform['value'] != 0:
                    mission_converge = False
                    error_message    = outputs.optInform['text']
                elif not converged_residual:
                    mission_converge = False
                    error_message    = (
                        f"pyopt ({numerics.mission_solver.method}) reported success but the "
                        f"equality-constraint residual is not within tolerance: max |residual| = "
                        f"{np.max(np.abs(residual)) if residual.size else 0.0:.3e} > "
                        f"{numerics.mission_solver.tolerance:.1e}"
                    )
                else:
                    mission_converge = True
                    error_message    = ""
            else:
                raise ValueError(
                    f"Unsupported mission_solver.package '{package}'. Supported values "
                    f"are 'scipy', 'pyopt'."
                )
        finally:
            # Terminate suppression of console window output
            if numerics.mission_solver.verbose == False:
                sys.stdout = sys.__stdout__

    elif numerics.mission_solver.type  == "root_finder":
        unknowns = segment.state.unknowns.mission.pack_array() 
        if segment.state.numerics.network_solver.type is None:
            unknowns = np.concatenate([unknowns, segment.state.unknowns.network.pack_array()])


        if segment.state.number_of_mission_unknowns != segment.state.number_of_mission_residuals:
            raise AttributeError('\n The system of equations representing the mission is not square. The number of unknowns (' + str(segment.state.number_of_mission_unknowns) + \
                                 ') is not equal to the number of residuals (equations) (' + str(segment.state.number_of_mission_residuals) + '). Either enforce of unknowns '+\
                                 ' to be equal to the number of residuals (equations) to use fsolve or switch RCAIDE solver type to "optimize" when defining the segment.'+ \
                                 '\n i.e. numerics.mission_solver.type  = "optimize" ') 
        else:
            # scale unknowns/residuals to O(1) before fsolve -- see _magnitude_scale
            unknown_scale  = _magnitude_scale(unknowns)
            residual_scale = _magnitude_scale(iterate_root_finder(unknowns, segment))

            def scaled_iterate(x_scaled):
                return iterate_root_finder(x_scaled * unknown_scale, segment) / residual_scale

            x_scaled,infodict,ier,error_message = scipy.optimize.fsolve(scaled_iterate,
                                                 unknowns / unknown_scale,
                                                 xtol   = numerics.mission_solver.tolerance,
                                                 maxfev = numerics.mission_solver.max_evaluations,
                                                 epsfcn = numerics.mission_solver.step_size,
                                                 full_output = 1) 
            scaled_iterate(x_scaled)

        if ier !=1:
            mission_converge = False
        else:
            mission_converge = True
            
    else: 
        raise Exception('undefined mission solver type')        
        
    if mission_converge == False or segment.state.numerics.network_solver.converged is False:
        print("Segment did not converge. Segment Tag: " + segment.tag)
        print("Error Message:\n" + error_message)
        numerics.mission_solver.converged = False 
        segment.converged = False
    else:
        numerics.mission_solver.converged = True
        segment.converged = True
                                
    return

# ----------------------------------------------------------------------------------------------------------------------
# scaling helper
# ----------------------------------------------------------------------------------------------------------------------
def _magnitude_scale(values, floor_exponent=-6):
    """Power-of-10 scale so values/scale lands near O(1) (e.g. 1e5 -> scale=1e5).
    Unscaled, unknowns/residuals of very different physical magnitude sharing
    one solver tolerance/step size leave the small ones effectively degenerate,
    and a Newton step can overshoot by orders of magnitude. Zero entries fall
    back to scale=1. Mirrors the unknown scaling in add_mission_variables.

    floor_exponent bounds how small a scale can get (default 1e-6, matching the
    typical solver tolerance): a value that's already near machine noise (e.g.
    ~1e-12, from a residual whose initial guess happens to be almost exact)
    would otherwise get an equally tiny scale, making that noise look like an
    O(1) constraint the solver must chase to the same relative precision as
    everything else -- preventing real convergence indefinitely.
    """
    factor = np.ceil(np.log10(np.abs(values)))
    factor[~np.isfinite(factor)] = 0
    factor = np.maximum(factor, floor_exponent)
    return 10.0 ** factor

# ----------------------------------------------------------------------------------------------------------------------
#  Helper Functions
# ----------------------------------------------------------------------------------------------------------------------
def _segment_soft_max_power(segment, sharpness=30.0):
    """Smooth (log-sum-exp) stand-in for max(power). A hard max is non-differentiable
    at whichever control point currently holds the peak, so its finite-difference
    gradient can jump discontinuously when that point changes between SLSQP's
    perturbed evaluations; this blends nearby points in instead, and is always >=
    the true max (standard log-sum-exp upper bound), overshooting by at most
    log(n_points)/p -- i.e. it never understates the peak.

    `sharpness` is in e-foldings across the segment's own power spread (max-min):
    e.g. sharpness=30 means a point at the very bottom of the segment's power
    range contributes negligible weight (~1e-13) while points within a few percent
    of peak blend in smoothly. Falls back to scaling by |peak| when the segment's
    power is ~constant (spread ~ 0), so it doesn't blow up to infinite sharpness
    there. Exponents are always <= 0 (peak subtracted first), so this can only
    underflow toward 0, never overflow -- regardless of the sign or magnitude of
    power.
    """
    power  = np.ravel(segment.state.conditions.energy.outputs.power.propulsive)
    peak   = np.max(power)
    spread = peak - np.min(power)
    scale  = spread if spread > 0 else max(abs(peak), 1.0)
    p      = sharpness / scale
    return float(peak + np.log(np.sum(np.exp(p * (power - peak)))) / p)


def iterate_root_finder(unknowns, segment):
    
    """Runs one iteration of of all analyses for the mission.

    Assumptions:
    fsolve has no native bounds support, unlike the "optimize"/SLSQP
    path -- without this, a proposed unknown (e.g. a [0,1] power split
    ratio or bounded control variable) can wander outside its declared
    bounds mid-iteration. Clip to the same bounds the SLSQP path
    enforces natively before evaluating the residual.

    Source:
    N/A

    Inputs:
    state.unknowns.mission                [Data]
    segment.process.iterate       [Data]

    Outputs:
    residuals                     [Unitless]

    Properties Used:
    N/A
    """
    if isinstance(unknowns, np.ndarray):
        lower = segment.state.unknowns_lower_bounds.mission.pack_array()
        upper = segment.state.unknowns_upper_bounds.mission.pack_array()
        if segment.state.numerics.network_solver.type is None:
            lower = np.concatenate([lower, segment.state.unknowns_lower_bounds.network.pack_array()])
            upper = np.concatenate([upper, segment.state.unknowns_upper_bounds.network.pack_array()])
        unknowns = np.clip(unknowns, lower, upper)

        mission_vec = segment.state.unknowns.mission.pack_array()
        mission_len = mission_vec.size
        segment.state.unknowns.mission.unpack_array(unknowns[:mission_len])
        if segment.state.numerics.network_solver.type is None:
            network_vec = segment.state.unknowns.network.pack_array()
            network_len = network_vec.size
            segment.state.unknowns.network.unpack_array(
                unknowns[mission_len:mission_len + network_len]
            )
    else:
        segment.state.unknowns.mission = unknowns
        if segment.state.numerics.network_solver.type is None:
            segment.state.unknowns.network = unknowns

    segment.process.iterate(segment)

    residuals = segment.state.residuals.mission.pack_array()
    if segment.state.numerics.network_solver.type is None:
        residuals = np.concatenate([
            residuals,
            segment.state.residuals.network.pack_array(),
        ])

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
    # -------------------------------------------------------------------------------------------
    # Step 1: Optimization framework
    # -------------------------------------------------------------------------------------------
    nexus                        = Nexus()
    optimization_problem         = Data() 
    segment.process.initialize.expand_state(segment)

    ground_seg_flag =  (type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Landing) or\
                       (type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Takeoff) or \
                       (type(segment) == RCAIDE.Framework.Mission.Segments.Ground.Ground)  
    single_pt_seg = (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude) or\
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_AVL_Trimmed) or \
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_No_Propulsion) or \
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Throttle)  
    
    input_count   = 0

    # -------------------------------------------------------------------------------------------      
    # get keys from mission and network unknowns 
    # -------------------------------------------------------------------------------------------
    # unknown kets 
    unknown_keys       = list(segment.state.unknowns.mission.keys())  
    net_unknown_keys   = list(segment.state.unknowns.network.keys())  
    residual_keys      = list(segment.state.residuals.mission.keys())  
    net_residual_keys  = list(segment.state.residuals.network.keys())
    
    # remove tag
    unknown_keys.remove('tag')
    net_unknown_keys.remove('tag')
    residual_keys.remove('tag')
    net_residual_keys.remove('tag')

    # -------------------------------------------------------------------------------------------   
    # determine dimension of optimization problem
    # -------------------------------------------------------------------------------------------
    if ground_seg_flag: 
        n_points      = segment.state.numerics.number_of_control_points
        len_inputs    = n_points
        len_residuals = n_points
        if segment.state.numerics.network_solver.type is None:
            len_inputs     += n_points * segment.state.number_of_network_unknowns
            len_residuals  += n_points * segment.state.number_of_network_residuals            
    elif single_pt_seg:
        n_points      = 1
        len_inputs    = segment.state.number_of_mission_unknowns 
        len_residuals = segment.state.number_of_mission_residuals 
    else:
        n_points      = segment.state.numerics.number_of_control_points  
        len_inputs    = n_points * segment.state.number_of_mission_unknowns   
        len_residuals = n_points * segment.state.number_of_mission_residuals
        if segment.state.numerics.network_solver.type is None:
            len_inputs     += n_points * segment.state.number_of_network_unknowns
            len_residuals  += n_points * segment.state.number_of_network_residuals   
        
    # -------------------------------------------------------------------------------------------
    # get unknowns assicated with the mission solver
    # -------------------------------------------------------------------------------------------         
    full_unkn_vals        = Data()
    full_upper_bound_vals = Data()
    full_lower_bound_vals = Data()
    for unkn in unknown_keys: 
        full_unkn_vals[unkn]        = segment.state.unknowns.mission[unkn]
        full_lower_bound_vals[unkn] = np.atleast_2d(segment.state.unknowns_lower_bounds.mission[unkn])
        full_upper_bound_vals[unkn] = np.atleast_2d(segment.state.unknowns_upper_bounds.mission[unkn])    
    
    # -------------------------------------------------------------------------------------------    
    # get unknowns from network if solver is coupled 
    # -------------------------------------------------------------------------------------------
    if segment.state.numerics.network_solver.type is None and (single_pt_seg != True):
        for unkn in net_unknown_keys: 
            full_unkn_vals[unkn]        = segment.state.unknowns.network[unkn]
            full_lower_bound_vals[unkn] = np.atleast_2d(segment.state.unknowns_lower_bounds.network[unkn])
            full_upper_bound_vals[unkn] = np.atleast_2d(segment.state.unknowns_upper_bounds.network[unkn]) 
        
    # -------------------------------------------------------------------------------------------            
    # Construct inputs nexus format  : [Variable_###, initial, -np.inf, np.inf , scaling, Units.less] 
    # -------------------------------------------------------------------------------------------
    initial_values    = full_unkn_vals.pack_array()
    input_len_strings = np.tile('Variable_', len_inputs)
    input_numbers     = np.linspace(1,len_inputs,len_inputs,dtype=np.int16)
    input_names       = np.char.add(input_len_strings,np.array(input_numbers+input_count).astype(str))
    lower_bounds      = full_lower_bound_vals.pack_array()
    upper_bounds      = full_upper_bound_vals.pack_array()
    units             = np.broadcast_to(Units.less,(len_inputs,))
    new_inputs        = np.reshape(np.tile(np.atleast_2d(np.array([None,None,None,None,None,None])),len_inputs), (-1, 6))
    
    # scaling factor for optimizer 
    factor = np.ceil(np.log10(abs(initial_values)))
    factor[np.isinf(factor)] = 0
    scale  = 10 ** (factor)
    
    # Step 2.4 Add in the inputs 
    new_inputs[:,0]     = input_names   
    new_inputs[:,1]     = initial_values 
    new_inputs[:,2]     = lower_bounds   
    new_inputs[:,3]     = upper_bounds  
    new_inputs[:,4]     = scale
    new_inputs[:,5]     = units
    optimization_problem.inputs = np.array(new_inputs,dtype=object)

    # -------------------------------------------------------------------------------------------
    # Construct constraints nexus format: equality constraints, scaled per-residual (see _magnitude_scale)
    # -------------------------------------------------------------------------------------------
    segment.process.iterate(segment)
    initial_residuals = segment.state.residuals.mission.pack_array()
    if segment.state.numerics.network_solver.type is None and not single_pt_seg:
        initial_residuals = np.concatenate([initial_residuals, segment.state.residuals.network.pack_array()])
    residual_scale = _magnitude_scale(initial_residuals)

    new_con = np.reshape(np.tile(np.atleast_2d(np.array([None,None,None,None,None])),len_residuals), (-1, 5))
    con_len_strings = np.tile('Residual_', len_residuals)
    con_numbers     = np.linspace(1,len_residuals,len_residuals,dtype=np.int16)
    con_names       = np.char.add(con_len_strings,np.array(con_numbers).astype(str))
    equals          = np.broadcast_to('=',(len_residuals,))
    zeros           = np.zeros(len_residuals)

    # Step 3.2 Add in the new constraints
    new_con[:,0]    = con_names
    new_con[:,1]    = equals
    new_con[:,2]    = zeros
    new_con[:,3]    = residual_scale
    new_con[:,4]    = 1*Units.less
    optimization_problem.constraints =  np.array(new_con,dtype=object)

    # -------------------------------------------------------------------------------------------      
    # Construct Aliases nexus format 
    # -------------------------------------------------------------------------------------------  
    # Step 4.1: Setup the aliases for the inputs
    basic_string_con = Data()
    input_string = []
    input_string_network = []

    if ground_seg_flag:       
        output_numbers = np.linspace(0,n_points-2,n_points-1,dtype=np.int16)
        basic_string_con[unknown_keys[1]] = np.tile('segment.state.unknowns.mission.'+unknown_keys[1]+'[', n_points-1)
        input_string.append(np.char.add(basic_string_con[unknown_keys[1]],np.array(output_numbers).astype(str)))  
        output_numbers = np.linspace(0,n_points-1,n_points,dtype=np.int16) 
        if segment.state.numerics.network_solver.type is None:
            for unkn in net_unknown_keys:  
                basic_string_con[unkn] = np.tile('segment.state.unknowns.network.'+unkn+'[', n_points)
                input_string_network.append(np.char.add(basic_string_con[unkn],np.array(output_numbers).astype(str)))   
            input_string = np.hstack((input_string[0],np.ravel(input_string_network))) 
        input_string        = np.char.add(input_string, np.tile(']',len_inputs-1))
        input_aliases       = np.reshape(np.tile(np.atleast_2d(np.array((None,None))),len_inputs), (-1, 2)) 
        input_aliases[:,0]  = input_names
        input_aliases[0,1]  = 'segment.state.unknowns.mission.'+unknown_keys[0] 
        input_aliases[1:,1] = input_string 
        
    elif single_pt_seg:  
        for unkn in unknown_keys:
            basic_string_con[unkn] = np.tile('segment.state.unknowns.mission.'+unkn+'[', n_points)
            input_string.append(np.char.add(basic_string_con[unkn],np.array([0]).astype(str)))
        input_string       = np.ravel(input_string).astype(str)
        input_string       = np.char.add(input_string, np.tile(']',len_inputs))
        input_aliases      = np.reshape(np.tile(np.atleast_2d(np.array((None,None))),len_inputs), (-1, 2))
        input_aliases[:,0] = input_names
        input_aliases[:,1] = input_string
    else:  
        output_numbers = np.linspace(0,n_points-1,n_points,dtype=np.int16) 
        for unkn in unknown_keys:
            basic_string_con[unkn] = np.tile('segment.state.unknowns.mission.'+unkn+'[', n_points)
            input_string.append(np.char.add(basic_string_con[unkn],np.array(output_numbers).astype(str)))
        
        if segment.state.numerics.network_solver.type is None:
            for unkn in net_unknown_keys:
                basic_string_con[unkn] = np.tile('segment.state.unknowns.network.'+unkn+'[', n_points)
                input_string_network.append(np.char.add(basic_string_con[unkn],np.array(output_numbers).astype(str)))
            input_string = np.hstack((np.ravel(input_string),np.ravel(input_string_network))).astype(str)
        else:
            input_string = np.ravel(input_string).astype(str)
        input_string       = np.char.add(input_string, np.tile(']',len_inputs))
        input_aliases      = np.reshape(np.tile(np.atleast_2d(np.array((None,None))),len_inputs), (-1, 2)) 
        input_aliases[:,0] = input_names
        input_aliases[:,1] = input_string
    
    # Step 4.2: Setup the aliases for the residuals. 
    def _pack_aliases(container_path, count):
        """Build 'container_path[i]' strings for i in 0..count-1."""
        base = np.tile(container_path + '[', count)
        return np.char.add(np.char.add(base, np.arange(count, dtype=np.int16).astype(str)), ']')

    if ground_seg_flag:
        # Ground segments have two mission residual fields in pack_array order: 
        all_res = _pack_aliases('segment.state.residuals.mission.pack_array()', n_points)

        if segment.state.numerics.network_solver.type is None:
            n_n_res = n_points * segment.state.number_of_network_residuals
            res_n   = _pack_aliases('segment.state.residuals.network.pack_array()', n_n_res)
            all_res = np.concatenate([all_res, res_n])

    elif single_pt_seg:
        # Single-point: one control point per residual; no network coupling.
        all_res = _pack_aliases('segment.state.residuals.mission.pack_array()', len_residuals)

    else:
        # General multi-point segments (climb, cruise, descent, …).
        n_m_res = n_points * segment.state.number_of_mission_residuals
        res_m   = _pack_aliases('segment.state.residuals.mission.pack_array()', n_m_res)

        if segment.state.numerics.network_solver.type is None:
            n_n_res = n_points * segment.state.number_of_network_residuals
            res_n   = _pack_aliases('segment.state.residuals.network.pack_array()', n_n_res)
            all_res = np.concatenate([res_m, res_n])
        else:
            all_res = res_m

    residual_aliases      = np.reshape(np.tile(np.atleast_2d(np.array((None, None))), len_residuals), (-1, 2))
    residual_aliases[:,0] = con_names
    residual_aliases[:,1] = all_res
        
    # Step 4.3: Append Aliases
    aliases = []
    for ii in range(len_inputs):
        aliases.append(input_aliases[ii].tolist())
    for jj in range(len_residuals):   
        aliases.append(residual_aliases[jj].tolist())
    
    # Step 5: Objective function 
    if segment.state.numerics.mission_solver.objective == None:
        aliases.append([ 'nothing'                   , 'postprocess.nothing'])
        optimization_problem.objective = np.array([ [  'nothing'  ,  1   ,    1*Units.less]  ],dtype=object)
    elif segment.state.numerics.mission_solver.objective == "energy":
        obj_scale = _magnitude_scale(np.atleast_1d(segment_energy_consumed(segment)))[0]
        aliases.append([ 'energy_consumed'          , 'postprocess.energy_consumed'])
        optimization_problem.objective = np.array([ [  'energy_consumed'  ,  obj_scale   ,    1*Units.less]  ],dtype=object)
    elif segment.state.numerics.mission_solver.objective == "power":
        obj_scale = _magnitude_scale(np.atleast_1d(_segment_soft_max_power(segment)))[0]
        aliases.append([ 'maximum_power'          , 'postprocess.maximum_power'])
        optimization_problem.objective = np.array([ [  'maximum_power'  ,  obj_scale   ,    1*Units.less]  ],dtype=object)
    else:
        raise Exception('undefined objective function')
    
    # append aliases
    optimization_problem.aliases = aliases

    # Step 7: Append segment
    nexus.segment = segment

    # Step 8: Append procedure
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
    if isinstance(unknowns, np.ndarray):
        mission_vec = segment.state.unknowns.mission.pack_array()
        mission_len = mission_vec.size
        segment.state.unknowns.mission.unpack_array(unknowns[:mission_len])
        if segment.state.numerics.network_solver.type is None:
            network_vec = segment.state.unknowns.network.pack_array()
            network_len = network_vec.size
            segment.state.unknowns.network.unpack_array(
                unknowns[mission_len:mission_len + network_len]
            )
    else:
        segment.state.unknowns.mission = unknowns
        if segment.state.numerics.network_solver.type is None:
            segment.state.unknowns.network = unknowns
    segment.process.iterate(segment)

    residuals = segment.state.residuals.pack_array()    
    nexus.residuals =  residuals
    return nexus


  
def segment_post_process(nexus):
    segment = nexus.segment

    postprocess                 = nexus.postprocess
    postprocess.maximum_power   = _segment_soft_max_power(segment)
    postprocess.energy_consumed = segment_energy_consumed(segment)
    postprocess.nothing         = 0

    return nexus


def segment_energy_consumed(segment):
    """Total propulsive energy consumed over the segment [J], or 0 for a single-point
    segment (one control point, no time interval to integrate over).
    """
    SPS = RCAIDE.Framework.Mission.Segments.Single_Point
    single_point_types = (SPS.Set_Speed_Set_Altitude, SPS.Set_Speed_Set_Altitude_AVL_Trimmed,
                           SPS.Set_Speed_Set_Altitude_No_Propulsion, SPS.Set_Speed_Set_Throttle)
    if type(segment) in single_point_types:
        return 0.0

    power = segment.state.conditions.energy.outputs.power.propulsive
    I     = segment.state.numerics.time.integrate
    return float(np.dot(I, power)[-1][0])  