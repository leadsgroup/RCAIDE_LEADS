# RCAIDE/Library/Mission/Solver/solver.py
#
#
# Created:  Jul 2023, M. Clarke  
# Modified: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import  Units, Data
from RCAIDE.Framework.Optimization.Packages.scipy import scipy_setup
from RCAIDE.Framework.Optimization.Packages.pyopt import pyopt_setup
from RCAIDE.Framework.Optimization.Common  import Nexus
from RCAIDE.Framework.Analyses.Process            import Process
from RCAIDE.Framework.Mission.Common.Segment_Layout import Segment_Layout

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
        segment.process.initialize.expand_state(segment)
        problem  = add_mission_variables(segment, Segment_Layout(segment.state))
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
                # leave the segment at the returned solution, not the optimizer's last evaluated point (as the pyopt branch does)
                problem.evaluate(outputs[0])

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
        layout   = Segment_Layout(segment.state)
        unknowns = layout.buffer[solver_slices(layout, segment.state.numerics.network_solver.type is None)[0]].copy()


        if segment.state.number_of_mission_unknowns != segment.state.number_of_mission_residuals:
            raise AttributeError('\n The system of equations representing the mission is not square. The number of unknowns (' + str(segment.state.number_of_mission_unknowns) + \
                                 ') is not equal to the number of residuals (equations) (' + str(segment.state.number_of_mission_residuals) + '). Either enforce of unknowns '+\
                                 ' to be equal to the number of residuals (equations) to use fsolve or switch RCAIDE solver type to "optimize" when defining the segment.'+ \
                                 '\n i.e. numerics.mission_solver.type  = "optimize" ') 
        else:
            # scale unknowns/residuals to O(1) before fsolve -- see magnitude_scale
            unknown_scale  = magnitude_scale(unknowns)
            residual_scale = magnitude_scale(iterate_root_finder(unknowns, segment, layout))

            def scaled_iterate(x_scaled):
                return iterate_root_finder(x_scaled * unknown_scale, segment, layout) / residual_scale

            x_scaled,infodict,ier,error_message = scipy.optimize.fsolve(scaled_iterate,
                                                 unknowns / unknown_scale,
                                                 xtol   = numerics.mission_solver.tolerance,
                                                 maxfev = numerics.mission_solver.max_evaluations,
                                                 epsfcn = numerics.mission_solver.step_size,
                                                 full_output = 1) 
            scaled_iterate(x_scaled)   # leaves the segment at the solution, mirrored into the bounds

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
def magnitude_scale(values, floor_exponent=-6):
    """Power-of-10 scale that brings values near O(1), so unknowns and residuals of very different magnitude
    share one solver tolerance. Zero entries use a scale of 1. floor_exponent bounds the smallest scale (1e-6,
    the typical solver tolerance), so a near-zero residual is not scaled up into noise the solver has to chase.
    """
    factor = np.ceil(np.log10(np.abs(values)))
    factor[~np.isfinite(factor)] = 0
    factor = np.maximum(factor, floor_exponent)
    return 10.0 ** factor

# ----------------------------------------------------------------------------------------------------------------------
#  Helper Functions
# ----------------------------------------------------------------------------------------------------------------------
def segment_soft_max_power(segment, sharpness=30.0):
    """Smooth upper bound on the segment's peak propulsive power by log-sum-exp. A hard max makes the
    finite-difference gradient jump when the peak moves between control points. The bound never understates
    the peak and exceeds it by at most log(n_points)/p. sharpness is in e-foldings across the segment's power
    spread; a segment at constant power is scaled by its peak instead.
    """
    power  = np.ravel(segment.state.conditions.energy.outputs.power.propulsive)
    peak   = np.max(power)
    spread = peak - np.min(power)
    scale  = spread if spread > 0 else max(abs(peak), 1.0)
    p      = sharpness / scale
    return float(peak + np.log(np.sum(np.exp(p * (power - peak)))) / p)


def iterate_root_finder(unknowns, segment, layout):
    """Residuals of the segment at the given unknowns, written to and read from the segment's layout.

    Assumptions:
    fsolve has no bounds, so a proposed unknown (e.g. a [0,1] power split ratio or a bounded control variable)
    can leave its declared bounds mid-iteration. It is mirrored back inside (reflect_into_bounds) before the
    residual is evaluated. Clipping it to the bound instead would make the residual flat beyond the bound, so the
    finite-difference Jacobian would lose that unknown and the solve would stall at the bound.

    Source:
    N/A

    Inputs:
    unknowns                      [array]
    segment.process.iterate       [Data]
    layout                        [Segment_Layout]

    Outputs:
    residuals                     [array]

    Properties Used:
    N/A
    """
    network = segment.state.numerics.network_solver.type is None
    lower   = segment.state.unknowns_lower_bounds.mission.pack_array()
    upper   = segment.state.unknowns_upper_bounds.mission.pack_array()
    if network:
        lower = np.concatenate([lower, segment.state.unknowns_lower_bounds.network.pack_array()])
        upper = np.concatenate([upper, segment.state.unknowns_upper_bounds.network.pack_array()])

    unknown_slice, residual_slice = solver_slices(layout, network)
    return segment_residual(reflect_into_bounds(unknowns, lower, upper), segment, layout, unknown_slice, residual_slice)


def reflect_into_bounds(x, lower, upper):
    """x with every entry outside [lower, upper] mirrored back inside at the bound it crossed, e.g. 0.1 below a
    lower bound becomes 0.1 above it. Entries within their bounds are returned unchanged. With both bounds finite
    the mirroring repeats, so any value maps into the interval."""
    both     = np.isfinite(lower) & np.isfinite(upper)
    width    = np.where(both, upper - lower, 1.)
    folded   = np.mod(x - np.where(both, lower, 0.), 2.*width)          # position along one mirrored period
    in_both  = np.where(both, lower, 0.) + np.where(folded <= width, folded, 2.*width - folded)
    in_lower = 2.*lower - x                                              # mirrored at a lower bound only
    in_upper = 2.*upper - x                                              # mirrored at an upper bound only
    mirrored = np.where(both, in_both, np.where(x < lower, in_lower, in_upper))
    outside  = (x < lower) | (x > upper)
    return np.where(outside, mirrored, x)


def segment_residual(unknowns, segment, layout, unknown_slice, residual_slice):
    """Residuals of the segment at the given unknowns, the one residual evaluation both solver paths use."""
    layout.buffer[unknown_slice] = unknowns
    iterate_in_layout(segment, layout)
    return layout.buffer[residual_slice].copy()


def iterate_in_layout(segment, layout):
    """One process.iterate; the solver writes unknowns to and reads residuals from the layout, so either one
    rebound to a new array instead of written in place is an error rather than a silently stale value."""
    segment.process.iterate(segment)
    rebound = layout.verify(segment.state, group="unknowns") + layout.verify(segment.state, group="residuals")
    if rebound:
        raise RuntimeError(f"{', '.join(rebound)} rebound to a new array during iterate; unknowns and residuals "
                           f"must be written in place, e.g. residual[...] = value")


def solver_slices(layout, network):
    """Slices of layout.buffer holding the solver's unknowns and residuals: the mission group, then the network
    group when the network is solved with the mission."""
    groups = ("mission", "network") if network else ("mission",)
    out    = []
    for kind in ("unknowns", "residuals"):
        parts = [layout.slices[kind + "." + g] for g in groups if kind + "." + g in layout.slices]
        if len(parts) == 2 and parts[0].stop != parts[1].start:
            raise ValueError(f"{kind}.mission and {kind}.network are not adjacent in the segment layout")
        out.append(slice(parts[0].start, parts[-1].stop) if parts else slice(0, 0))
    return out


def add_mission_variables(segment, layout):
    """Sets up the segment's converge as an optimization problem whose inputs are its unknowns and whose
    equality constraints are its residuals, both held in the segment's layout.

        Assumptions:
        N/A

        Source:
        N/A

        Inputs:
        segment            [Data]
        layout             [Segment_Layout]

        Outputs:
        nexus              [Segment_Problem]

        Properties Used:
        None
    """
    nexus                        = Segment_Problem()
    optimization_problem         = Data()

    single_pt_seg = (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude) or\
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_AVL_Trimmed) or \
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Altitude_No_Propulsion) or \
                    (type(segment) == RCAIDE.Framework.Mission.Segments.Single_Point.Set_Speed_Set_Throttle)
    network = segment.state.numerics.network_solver.type is None and not single_pt_seg
    unknown_slice, residual_slice = solver_slices(layout, network)
    len_inputs    = unknown_slice.stop - unknown_slice.start
    len_residuals = residual_slice.stop - residual_slice.start

    # -------------------------------------------------------------------------------------------
    # bounds of the mission unknowns, then of the network unknowns if solved together
    # -------------------------------------------------------------------------------------------
    full_upper_bound_vals = Data()
    full_lower_bound_vals = Data()
    groups = ("mission", "network") if network else ("mission",)
    for group in groups:
        for unkn in segment.state.unknowns[group].keys():
            if unkn == 'tag':
                continue
            full_lower_bound_vals[unkn] = np.atleast_2d(segment.state.unknowns_lower_bounds[group][unkn])
            full_upper_bound_vals[unkn] = np.atleast_2d(segment.state.unknowns_upper_bounds[group][unkn])

    # -------------------------------------------------------------------------------------------
    # Construct inputs nexus format  : [Variable_###, initial, -np.inf, np.inf , scaling, Units.less]
    # -------------------------------------------------------------------------------------------
    initial_values    = layout.buffer[unknown_slice].copy()
    input_len_strings = np.tile('Variable_', len_inputs)
    input_numbers     = np.linspace(1,len_inputs,len_inputs,dtype=np.int16)
    input_names       = np.char.add(input_len_strings,np.array(input_numbers).astype(str))
    lower_bounds      = full_lower_bound_vals.pack_array()
    upper_bounds      = full_upper_bound_vals.pack_array()
    units             = np.broadcast_to(Units.less,(len_inputs,))
    new_inputs        = np.reshape(np.tile(np.atleast_2d(np.array([None,None,None,None,None,None])),len_inputs), (-1, 6))

    # scaling factor for optimizer
    factor = np.ceil(np.log10(abs(initial_values)))
    factor[np.isinf(factor)] = 0
    scale  = 10 ** (factor)

    new_inputs[:,0]     = input_names
    new_inputs[:,1]     = initial_values
    new_inputs[:,2]     = lower_bounds
    new_inputs[:,3]     = upper_bounds
    new_inputs[:,4]     = scale
    new_inputs[:,5]     = units
    optimization_problem.inputs = np.array(new_inputs,dtype=object)

    # -------------------------------------------------------------------------------------------
    # Construct constraints nexus format: equality constraints, scaled per-residual (see magnitude_scale)
    # -------------------------------------------------------------------------------------------
    iterate_in_layout(segment, layout)
    residual_scale = magnitude_scale(layout.buffer[residual_slice].copy())

    new_con = np.reshape(np.tile(np.atleast_2d(np.array([None,None,None,None,None])),len_residuals), (-1, 5))
    con_len_strings = np.tile('Residual_', len_residuals)
    con_numbers     = np.linspace(1,len_residuals,len_residuals,dtype=np.int16)
    con_names       = np.char.add(con_len_strings,np.array(con_numbers).astype(str))
    equals          = np.broadcast_to('=',(len_residuals,))
    zeros           = np.zeros(len_residuals)

    new_con[:,0]    = con_names
    new_con[:,1]    = equals
    new_con[:,2]    = zeros
    new_con[:,3]    = residual_scale
    new_con[:,4]    = 1*Units.less
    optimization_problem.constraints =  np.array(new_con,dtype=object)

    # -------------------------------------------------------------------------------------------
    # Objective function
    # -------------------------------------------------------------------------------------------
    if segment.state.numerics.mission_solver.objective == None:
        optimization_problem.objective = np.array([ [  'nothing'  ,  1   ,    1*Units.less]  ],dtype=object)
    elif segment.state.numerics.mission_solver.objective == "energy":
        obj_scale = magnitude_scale(np.atleast_1d(segment_energy_consumed(segment)))[0]
        optimization_problem.objective = np.array([ [  'energy_consumed'  ,  obj_scale   ,    1*Units.less]  ],dtype=object)
    elif segment.state.numerics.mission_solver.objective == "power":
        obj_scale = magnitude_scale(np.atleast_1d(segment_soft_max_power(segment)))[0]
        optimization_problem.objective = np.array([ [  'maximum_power'  ,  obj_scale   ,    1*Units.less]  ],dtype=object)
    else:
        raise Exception('undefined objective function')

    nexus.segment              = segment
    nexus.layout               = layout
    nexus.unknown_slice        = unknown_slice
    nexus.residual_slice       = residual_slice
    nexus.residual_index       = {name: i for i, name in enumerate(con_names)}
    nexus.procedure            = iterate_segment()
    nexus.postprocess          = Data()
    nexus.optimization_problem = optimization_problem

    return nexus


class Segment_Problem(Nexus):
    """Nexus for one segment's converge: inputs are evaluated by segment_residual and the residual constraints are
    read from its result, in place of string aliases."""

    def __defaults__(self):
        self.segment        = None
        self.layout         = None
        self.unknown_slice  = None
        self.residual_slice = None
        self.residual_index = {}
        self.postprocess    = Data()
        self.unknowns       = None
        self.residuals      = None

    def set_inputs(self, inputs, converted_values):
        self.unknowns = converted_values

    def output_values(self, outputs):
        names = np.array(outputs)[:, 0]
        return np.array([self.residuals[self.residual_index[n]] if n in self.residual_index else self.postprocess[n]
                         for n in names], dtype=float)


def iterate_segment(): 
    procedure                           = Process()  
    procedure.segment                   = Process()
    procedure.segment.design_mission    = iterate_optimizer     
    procedure.post_process              = segment_post_process   
        
    return procedure
    
def iterate_optimizer(nexus):
    nexus.residuals = segment_residual(nexus.unknowns, nexus.segment, nexus.layout, nexus.unknown_slice, nexus.residual_slice)
    return nexus


  
def segment_post_process(nexus):
    segment = nexus.segment

    postprocess                 = nexus.postprocess
    postprocess.maximum_power   = segment_soft_max_power(segment)
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