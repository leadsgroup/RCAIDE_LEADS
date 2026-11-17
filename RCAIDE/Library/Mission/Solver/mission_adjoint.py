# RCAIDE/Library/Mission/Solver/mission_adjoint.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Mission.Solver.compiled_residual import (compile_segment, unknowns_vector, set_unknowns, copy_state,
                                                             state_arrays, update_position)

# relative step of the finite differences of a segment setup with respect to the end state of the segment before it
SETUP_STEP = 1e-6

# relative change of an input that marks an array as read by a setup: the arrays are moved by 1e-3 relative, and
# repeated setups differ by round-off (1e-13)
DETECTION_TOLERANCE = 1e-9

# the state paths of the previous segment that a segment setup reads, by compiled residual and segment tag: found once
# per segment structure, since a state holds hundreds of arrays (spanwise loads, vortex strengths) a setup never reads
READ_BY_SETUP = {}

# ----------------------------------------------------------------------------------------------------------------------
#  mission_adjoint
# ----------------------------------------------------------------------------------------------------------------------
def mission_adjoint(mission, seeds, setup_mission):
    """
    Derivative of one output of a solved mission with respect to the inputs of every segment, in one backward sweep
    over the segments (reverse mode), without solving the mission again.

    The converged unknowns x of a segment satisfy R(x, inputs) = 0 and its state arrays are S(x, inputs). For the
    weights s of the output on the state, the derivative with respect to the inputs is
       S_inputs^T s - R_inputs^T mu,  with  R_x^T mu = S_x^T s,
    all from the segment's compiled residual. A segment's inputs depend on the end state of the segment before it in
    two ways. Its initials are that state, and the residual reads some of them: their weights pass back exactly. The
    segment setup also writes the last row of a few arrays (position, velocity, time, mass, energy stored) into the
    segment's own inputs: that link is a finite difference of the setup, one per number of the last row, each a setup
    and one evaluation of the residual with no solve.

    Inputs:
       mission         solved mission (mission.evaluate() has run)
       seeds           {segment tag: {state path: array}}, the derivative of the output with respect to the state
                       arrays of the segments it reads (for example output_fuel_burned)
       setup_mission   the same mission built again, not solved (the setups run on its segments)

    Outputs:
       derivatives   one Data per segment, in order:
                        tag        segment tag
                        paths      paths of the segment's inputs (those of its compiled residual)
                        inputs     derivative of the output with respect to each input, one array per path
                        setups     number of setups run for the link to the segment before

    Assumptions:
       setup_mission has the same structure as mission. The end state a setup reads is the last row of each array.
    """
    segments = list(mission.segments.values())
    setups   = list(setup_mission.segments.values())
    setup_mission.process.initialize(setup_mission)
    # the state of each segment as set up, restored before every setup of the links
    starts   = [copy_state(segment.state, np.array) for segment in setups]

    carried     = {}
    derivatives = [None] * len(segments)
    for k in reversed(range(len(segments))):
        segment          = segments[k]
        x                = unknowns_vector(segment.state.unknowns)
        compiled, inputs = compile_segment(segment)

        # weights on the state of this segment: the output's own and those passed back by the segment after it
        weights_by_path = dict(carried)
        for path, weight in seeds.get(segment.tag, {}).items():
            weights_by_path[path] = weights_by_path.get(path, 0.) + np.asarray(weight, dtype=float)
        shapes  = state_shapes(compiled, x, inputs)
        weights = [np.broadcast_to(np.asarray(weights_by_path.get(path, 0.), dtype=float), shape) for path, shape in shapes]

        # weights on arrays the residual does not compute are those of inputs of this segment; any other is an error
        computed = {path for path, shape in shapes}
        direct   = {}
        for path, weight in weights_by_path.items():
            if path in computed:
                continue
            if path not in compiled.paths:
                raise ValueError(f"{segment.tag}: the output depends on {path}, neither computed by nor an input of the residual")
            direct[compiled.paths.index(path)] = weight

        # reverse products of the state and the residual at the converged unknowns
        x_bar, inputs_bar = compiled.state_transpose(x, inputs, weights)
        inputs_bar        = tuple(np.asarray(bar) + direct.get(i, 0.) for i, bar in enumerate(inputs_bar))
        jacobian          = np.asarray(compiled.jacobian(x, inputs))
        for name, array in (('Jacobian', jacobian), ('reverse product of the state', np.asarray(x_bar))):
            if not np.all(np.isfinite(array)):
                raise ValueError(f"{segment.tag}: the {name} of the residual is not finite")
        mu                = np.linalg.lstsq(jacobian.T, np.asarray(x_bar), rcond=None)[0]
        residual_bar      = compiled.residual_transpose(x, inputs, mu)[1]
        inputs_bar        = tuple(np.asarray(a) - np.asarray(b) for a, b in zip(inputs_bar, residual_bar))
        derivatives[k]    = Data(tag=segment.tag, paths=list(compiled.paths), inputs=inputs_bar, setups=0)

        # the weights passed to the segment before, through the setup of this one
        carried = {}
        if k > 0:
            carried, count    = setup_transpose(segments[k-1], setups[k], starts[k], compiled, x, inputs, inputs_bar)
            derivatives[k].setups = count
    return derivatives


def state_shapes(compiled, x, inputs):
    """(path, shape) of each state array of a compiled residual, in the order of its reverse product of the state.
    Values the stored state holds as plain floats are arrays of the compiled state."""
    tangents = compiled.state_derivative(x, inputs, np.zeros_like(x), tuple(np.zeros_like(i) for i in inputs))
    return [(path, np.shape(tangent)) for path, tangent in zip(compiled.state_paths, tangents)]


def setup_transpose(previous, setup_segment, start, compiled, x, inputs, inputs_bar):
    """
    The weights on the end state of the previous segment, (d inputs / d end state)^T inputs_bar, with the derivative
    of this segment's inputs with respect to the last row of each float array of the previous segment's state taken
    by central differences of the segment setup.

    Outputs:
       carried   {state path of the previous segment: array}, nonzero in the last row only
       count     number of setups run
    """
    base_state = previous.state
    carried    = {}
    count      = 0

    # the initials are inputs of the residual themselves: their weights pass back exactly
    own = [i for i, path in enumerate(compiled.paths) if not path.startswith('initials.')]
    for i, path in enumerate(compiled.paths):
        if path.startswith('initials.'):
            state_path = 'state' + path[len('initials'):]
            carried[state_path] = carried.get(state_path, 0.) + np.asarray(inputs_bar[i], dtype=float)

    # what the setup writes into the segment's own inputs, by central differences over the last row
    read, found = arrays_read_by_setup(base_state, setup_segment, start, compiled, x, own)
    count      += found
    for container, key, path in state_arrays(base_state):
        if path not in read:
            continue
        array = container[key]
        for column in range(array.shape[1]):
            value = array[-1, column]
            step  = SETUP_STEP * max(1., abs(value))
            sides = []
            for move in (+step, -step):
                moved = copy_state(base_state, np.array)
                for c, k, p in state_arrays(moved):
                    if p == path:
                        c[k] = np.array(c[k], dtype=float)
                        c[k][-1, column] = value + move
                sides.append(inputs_after_setup(setup_segment, moved, x, compiled, start))
                count += 1
            if sides[0] is None or sides[1] is None:
                continue
            derivative = sum(float(np.sum(np.asarray(inputs_bar[i]) * (sides[0][i] - sides[1][i]) / (2 * step))) for i in own)
            if derivative != 0.:
                weight = carried.setdefault(path, np.zeros_like(np.asarray(array, dtype=float)))
                weight[-1, column] += derivative
    return carried, count


def arrays_read_by_setup(state, setup_segment, start, compiled, x, own):
    """
    The paths of the arrays of a previous segment's state whose last row moves the inputs of this segment other than
    its initials (those at the indices own): each array's last row is moved as a whole and the setup run once. Kept by
    compiled residual and segment tag.

    Outputs:
       read    set of state paths
       count   number of setups run (zero when the paths are already known)
    """
    name = (id(compiled), setup_segment.tag)
    if name in READ_BY_SETUP:
        return READ_BY_SETUP[name], 0
    # the base from an unchanged copy, as the moved states are copies: copying moves the inputs by round-off
    base  = inputs_after_setup(setup_segment, copy_state(state, np.array), x, compiled, start)
    read  = set()
    count = 1
    for container, key, path in state_arrays(state):
        array = container[key]
        if array.ndim != 2 or array.shape[0] == 0:
            continue
        moved = copy_state(state, np.array)
        for c, k, p in state_arrays(moved):
            if p == path:
                c[k] = np.array(c[k], dtype=float)
                c[k][-1] = c[k][-1] + 1e-3 * np.maximum(1., np.abs(c[k][-1]))
        values = inputs_after_setup(setup_segment, moved, x, compiled, start)
        count += 1
        if values is None or any(np.any(np.abs(values[i] - base[i]) > DETECTION_TOLERANCE * (1. + np.abs(base[i]))) for i in own):
            read.add(path)
    READ_BY_SETUP[name] = read
    return read, count


def inputs_after_setup(segment, initials, x, compiled, start=None):
    """The inputs of a segment set up from the given end state of the segment before it, at the unknowns x; None
    when the setup gives a segment of another structure. A setup reads values of the segment's state that the setup
    before it left there, so with start (a copy_state of the segment's state) every float array is first reset to it."""
    if start is not None:
        start_arrays = {path: container[key] for container, key, path in state_arrays(start)}
        for container, key, path in state_arrays(segment.state):
            if path in start_arrays:
                container[key] = np.array(start_arrays[path])
    segment.state.initials = initials
    expand_state = segment.process.initialize.expand_state
    expand_state(segment)
    segment.process.initialize.expand_state = RCAIDE.Library.Methods.skip
    try:
        segment.process.initialize(segment)
    finally:
        segment.process.initialize.expand_state = expand_state
    set_unknowns(segment.state.unknowns, segment.state.unknowns, x)
    segment.process.iterate(segment)
    update_position(segment)
    other, values = compile_segment(segment)
    if other is not compiled:
        return None
    return [np.asarray(value, dtype=float) for value in values]


def output_fuel_burned(mission):
    """Seeds of the fuel burned over a mission: the vehicle mass at the start of the first segment less that at the
    end of the last."""
    mass     = 'state.conditions.weights.vehicle.mass'
    segments = list(mission.segments.values())
    seeds    = {}
    first    = np.zeros_like(np.asarray(segments[0].state.conditions.weights.vehicle.mass, dtype=float))
    first[0, 0] = 1.
    seeds.setdefault(segments[0].tag, {})[mass] = first
    last     = np.zeros_like(np.asarray(segments[-1].state.conditions.weights.vehicle.mass, dtype=float))
    last[-1, 0] = -1.
    seeds.setdefault(segments[-1].tag, {})
    seeds[segments[-1].tag][mass] = seeds[segments[-1].tag].get(mass, 0.) + last
    return seeds
