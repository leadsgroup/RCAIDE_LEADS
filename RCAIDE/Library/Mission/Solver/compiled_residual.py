# RCAIDE/Library/Mission/Solver/compiled_residual.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import copy
import types
import numpy as np

from RCAIDE.Framework.Core import Data, array_namespace
from RCAIDE.Library.Methods.Utilities.Linear_Interpolator import Linear_Interpolator

# compiled residuals by segment signature, shared by every segment with the same structure
COMPILED_SEGMENTS = {}

# ----------------------------------------------------------------------------------------------------------------------
#  compile_segment
# ----------------------------------------------------------------------------------------------------------------------
def compile_segment(segment):
    """
    The compiled residual of a segment and the segment's inputs for it.

    The residual of the segment's iterate steps is compiled with jax.jit as a function of the unknowns and of every
    float the steps read from the analyses (vehicle, surrogate tables, engine design), the state and the initials.
    Segments with the same signature (see segment_inputs) share one compiled function, so a new design with the same
    structure does not compile again.

    Inputs:
       segment     an initialized segment (expand_state and initialize have run)

    Outputs:
       compiled    Compiled_Segment (residual, jacobian, input_derivative, state_derivative)
       inputs      tuple of arrays, the float inputs of this segment that the residual reads, one per compiled.paths
    """
    references, signature = segment_inputs(segment)
    if signature not in COMPILED_SEGMENTS:
        COMPILED_SEGMENTS[signature] = Compiled_Segment(segment, references)
    compiled = COMPILED_SEGMENTS[signature]
    compiled.segment = segment
    by_path  = {reference[3]: reference for reference in references}
    inputs   = tuple(np.asarray(read_input(by_path[path]), dtype=float) for path in compiled.paths)
    return compiled, inputs

# ----------------------------------------------------------------------------------------------------------------------
#  Compiled_Segment
# ----------------------------------------------------------------------------------------------------------------------
class Compiled_Segment:
    """
    The compiled residual of one segment structure, of the unknowns x and the inputs (one array per path):
       residual(x, inputs)                      residual
       jacobian(x, inputs)                      derivative of the residual with respect to x
       input_derivative(x, inputs, direction)   derivative of the residual along a direction of the inputs
       state_derivative(x, inputs, x_direction, direction)
                                                derivative of the state arrays (state_paths) along a direction of
                                                the unknowns and the inputs
    """

    def __init__(self, segment, references):
        import jax
        import jax.numpy as jnp

        # the segment a trace runs on: any segment with this signature, set by compile_segment before each use
        self.segment = segment
        self.traces  = 0

        # trace with every float input as an argument; the inputs the residual does not read drop out of the trace
        x      = unknowns_vector(segment.state.unknowns)
        paths  = [reference[3] for reference in references]
        values = [jnp.asarray(read_input(reference), dtype=float) for reference in references]
        traced = jax.make_jaxpr(lambda x, values: residual_of_inputs(segment, x, paths, values))(jnp.asarray(x), values)
        read   = set()
        for equation in traced.jaxpr.eqns:
            read.update(variable for variable in equation.invars if type(variable).__name__ != 'Literal')
        read.update(variable for variable in traced.jaxpr.outvars if type(variable).__name__ != 'Literal')
        used   = [i for i, variable in enumerate(traced.jaxpr.invars[1:]) if variable in read]

        self.paths = [paths[i] for i in used]

        def residual(x, inputs):
            self.traces += 1
            return residual_of_inputs(self.segment, x, self.paths, list(inputs))
        def input_derivative(x, inputs, direction):
            return jax.jvp(lambda inputs: residual(x, inputs), (inputs,), (direction,))[1]
        def state(x, inputs):
            self.state_paths, arrays = state_of_inputs(self.segment, x, self.paths, list(inputs))
            return [jnp.asarray(array, dtype=float) for array in arrays]
        def state_derivative(x, inputs, x_direction, direction):
            return jax.jvp(state, (x, inputs), (x_direction, direction))[1]
        self.residual         = jax.jit(residual)
        self.jacobian         = jax.jit(jax.jacfwd(residual))
        self.input_derivative = jax.jit(input_derivative)
        self.state_derivative = jax.jit(state_derivative)

# ----------------------------------------------------------------------------------------------------------------------
#  The residual as a function of the unknowns and the inputs
# ----------------------------------------------------------------------------------------------------------------------
def residual_of_inputs(segment, x, paths, values):
    """
    Residual of the segment's iterate steps with unknowns x and the float inputs at paths replaced by values, on NumPy
    or JAX. The steps run on a copy of the state, and the analyses are restored afterwards.
    """
    return evaluate_segment(segment, x, paths, values)[0]


def state_of_inputs(segment, x, paths, values):
    """
    The float arrays of the segment's state after its iterate steps and its position update, as (state paths, arrays):
    what the next segment and the mission outputs read. Same arguments as residual_of_inputs.
    """
    return evaluate_segment(segment, x, paths, values)[1:]


def evaluate_segment(segment, x, paths, values):
    """Runs the iterate steps and the position update on a copy of the state; residual, state paths and state arrays."""
    state = segment.state
    fresh = copy_state(state, np.array)
    if state.initials:
        dict.__setitem__(fresh, 'initials', copy_state(state.initials, np.array))

    # segment.conditions is an alias of segment.state.conditions, so both are swapped
    segment.state, segment.conditions = fresh, fresh.conditions
    references = segment_inputs(segment)[0]
    by_path    = {reference[3]: reference for reference in references}
    replaced   = [by_path[path] for path in paths]
    saved      = [read_input(reference) for reference in replaced]
    try:
        for reference, value, original in zip(replaced, values, saved):
            # a NumPy scalar goes back as the float it replaces (numba kernels are typed on floats)
            if isinstance(value, np.ndarray) and value.ndim == 0 and not isinstance(original, np.ndarray):
                value = float(value)
            write_input(reference, value)
        # after the inputs: the state's own unknowns are among them
        set_unknowns(fresh.unknowns, state.unknowns, x)
        for step in iterate_steps(segment.process.iterate):
            step(segment)
        xp       = array_namespace(x, *values)
        residual = xp.concatenate([xp.ravel(value, order='F') for value in float_leaves(fresh.residuals)])
        update_position(segment)
        located  = state_arrays(fresh)
        return residual, [path for container, key, path in located], [container[key] for container, key, path in located]
    finally:
        segment.state, segment.conditions = state, state.conditions
        for reference, value in zip(replaced, saved):
            write_input(reference, value)


def update_position(segment):
    """The post-process step that integrates the horizontal position, which the next segment starts from."""
    if 'inertial_position' in segment.process.post_process.keys():
        segment.process.post_process.inertial_position(segment)
    return


def state_arrays(state, path='state'):
    """One (container, key, path) per float array of a state, NumPy or JAX, without the initials."""
    located = []
    for key, value in dict.items(state):
        if key == 'initials':
            continue
        if isinstance(value, dict):
            located += state_arrays(value, f"{path}.{key}")
        elif array_namespace(value) is not np or (isinstance(value, np.ndarray) and value.dtype.kind == 'f'):
            located.append((state, key, f"{path}.{key}"))
    return located

# ----------------------------------------------------------------------------------------------------------------------
#  Inputs and signature of a segment
# ----------------------------------------------------------------------------------------------------------------------
def segment_inputs(segment):
    """
    Walks the segment's settings, its analyses, its state and its initials (one level, not the chain of earlier
    segments).

    Outputs:
       references   one (container, key, kind, path) per float input: finite floats and finite float arrays
       signature    everything else, by value (types, tags, flags, integers, non-finite floats, the shapes of the float
                    inputs and the iterate steps): segments with equal signatures run the same compiled residual
    """
    references = []
    static     = [('segment', type(segment).__qualname__)]
    static    += [('step', describe(step)) for step in iterate_steps(segment.process.iterate)]
    static    += [('unknown', path, np.shape(container[key])) for container, key, path in unknown_entries(segment.state.unknowns)]
    # the segment's own settings (speeds, altitudes, distances); its tags name it and its converged flag is a result
    collect_inputs(segment, 'segment', references, static, set(),
                   skip=('analyses', 'state', 'conditions', 'process', 'tag', 'mission_tag', 'converged'))
    collect_inputs(segment.analyses, 'analyses', references, static, set())
    collect_inputs(segment.state, 'state', references, static, set(), skip=('initials',))
    if segment.state.initials:
        collect_inputs(segment.state.initials, 'initials', references, static, set(), skip=('initials',))
    return references, tuple(static)


def signature_difference(segment, other, limit=5):
    """The first entries in which the signatures of two segments differ, as text."""
    first  = {entry[0]: entry[1:] for entry in segment_inputs(segment)[1]}
    second = {entry[0]: entry[1:] for entry in segment_inputs(other)[1]}
    lines  = [f"{path}: {str(first.get(path))[:60]} against {str(second.get(path))[:60]}"
              for path in list(first) + [path for path in second if path not in first] if first.get(path) != second.get(path)]
    return "; ".join(lines[:limit])


# data only the NumPy path reads, so neither an input nor part of the structure of the compiled residual: the idle
# fallback deck of a gas turbine (the JAX matching returns NaN at a point NumPy would route to it)
NUMPY_ONLY = ('idle_fallback',)

def collect_inputs(item, path, references, static, seen, skip=()):
    """Appends the float inputs under item to references and everything else to static; skip names keys of item itself."""
    if id(item) in seen:
        static.append((path, 'seen'))
        return
    seen.add(id(item))
    static.append((path, type(item).__qualname__))

    # the unknowns are arguments of the residual; what the solve writes about itself (convergence, counters) is a result
    if path in ('state.unknowns', 'state.numerics.mission_solver'):
        return

    if isinstance(item, Linear_Interpolator):
        for i in range(len(item.axes)):
            collect_value(item.axes, i, item.axes[i], 'item', f"{path}.axes[{i}]", references, static, seen)
        collect_value(item, 'values', item.values, 'attribute', f"{path}.values", references, static, seen)
    elif isinstance(item, dict):
        for key, value in dict.items(item):
            if key not in skip and key not in NUMPY_ONLY and not (path == 'state' and str(key).startswith('number_of_')):
                collect_value(item, key, value, 'item', f"{path}.{key}", references, static, seen)
        # object attributes of a Data (the conditions size, a vehicle's maps to its own containers), walked after its
        # items so a container held both ways is walked once; not the binding to the preallocated layout
        for key, value in getattr(item, '__dict__', {}).items():
            if key != 'segment_layout':
                collect_value(item, key, value, 'attribute', f"{path}:{key}", references, static, seen)
    else:
        for i, value in enumerate(item):
            collect_value(item, i, value, 'item', f"{path}[{i}]", references, static, seen)
    return


def collect_value(container, key, value, kind, path, references, static, seen):
    """One entry of a container: a float input, a container to walk, or a static value."""
    if is_float_input(value):
        references.append((container, key, kind, path))
        static.append((path, 'float', np.shape(value)))
    elif isinstance(value, (dict, list, Linear_Interpolator)):
        collect_inputs(value, path, references, static, seen)
    else:
        static.append((path, describe(value)))
    return


def is_float_input(value):
    """A finite float or float array. Integers and non-finite floats (inf as "no limit") are static."""
    if isinstance(value, np.ndarray):
        return value.dtype.kind == 'f' and bool(np.all(np.isfinite(value)))
    if isinstance(value, (float, np.floating)):
        return bool(np.isfinite(value))
    return False


def describe(value, seen=None):
    """A static value as something hashable that is equal for equal values; objects are described by their contents."""
    if value is None or isinstance(value, (bool, int, float, str, np.generic)):
        return repr(value)
    if isinstance(value, np.ma.MaskedArray):
        return (str(value.dtype), value.shape, np.ma.getdata(value).tobytes(), np.ma.getmaskarray(value).tobytes())
    if isinstance(value, np.ndarray):
        return (str(value.dtype), value.shape, value.tobytes())
    if isinstance(value, types.ModuleType):
        return value.__name__
    if hasattr(value, '__qualname__'):
        return f"{getattr(value, '__module__', '')}.{value.__qualname__}"

    # containers and objects, each described once
    seen = set() if seen is None else seen
    if id(value) in seen:
        return 'seen'
    seen.add(id(value))
    if isinstance(value, dict):
        return (type(value).__qualname__,) + tuple((repr(key), describe(entry, seen)) for key, entry in dict.items(value))
    if isinstance(value, (list, tuple)):
        return (type(value).__qualname__,) + tuple(describe(entry, seen) for entry in value)
    if isinstance(value, (set, frozenset)):
        return (type(value).__qualname__,) + tuple(sorted(repr(entry) for entry in value))
    if hasattr(value, '__dict__'):
        return (type(value).__qualname__, describe(vars(value), seen))
    # anything else is identified by itself, so a segment holding one never shares a compiled residual wrongly
    return (type(value).__qualname__, id(value))


def read_input(reference):
    container, key, kind = reference[:3]
    return getattr(container, key) if kind == 'attribute' else container[key]


def write_input(reference, value):
    container, key, kind = reference[:3]
    if kind == 'attribute':
        setattr(container, key, value)
    elif isinstance(container, dict):
        dict.__setitem__(container, key, value)
    else:
        container[key] = value
    return

# ----------------------------------------------------------------------------------------------------------------------
#  State helpers
# ----------------------------------------------------------------------------------------------------------------------
def iterate_steps(process, frozen_steps=(), frozen=False):
    """The iterate process as a flat list of functions, in order, without the frozen steps."""
    steps = []
    for key, step in process.items():
        skip = frozen or key in frozen_steps
        if hasattr(step, 'items'):
            steps += iterate_steps(step, frozen_steps, skip)
        elif not skip:
            steps.append(step)
    return steps


def float_leaves(tree):
    """Float arrays of a state branch, in a fixed order."""
    leaves = []
    for key, value in dict.items(tree):
        if isinstance(value, dict):
            leaves += float_leaves(value)
        elif array_namespace(value) is not np or (isinstance(value, np.ndarray) and value.dtype.kind == 'f'):
            leaves.append(value)
    return leaves


def unknown_entries(unknowns, path='unknowns'):
    """One (container, key, path) per unknown, in a fixed order: float arrays and scalars (a segment time)."""
    entries = []
    for key, value in dict.items(unknowns):
        if isinstance(value, dict):
            entries += unknown_entries(value, f"{path}.{key}")
        elif isinstance(value, np.ndarray) and value.dtype.kind == 'f':
            entries.append((unknowns, key, f"{path}.{key}"))
        elif isinstance(value, (int, float, np.number)) and not isinstance(value, bool):
            entries.append((unknowns, key, f"{path}.{key}"))
    return entries


def unknowns_vector(unknowns):
    """The unknowns of a state as one flat NumPy vector, column-major like Data.pack_array and the solver."""
    return np.concatenate([np.ravel(np.asarray(container[key], dtype=float), order='F') for container, key, path in unknown_entries(unknowns)])


def set_unknowns(tree, numpy_tree, x):
    """Writes the flat vector x into the unknowns of tree, in the order and shapes of the NumPy unknowns."""
    by_path = {path: container for container, key, path in unknown_entries(tree)}
    start   = 0
    for container, key, path in unknown_entries(numpy_tree):
        shape = np.shape(container[key])
        size  = int(np.prod(shape))
        value = x[start:start + size].reshape(shape, order='F')
        # a NumPy scalar unknown stays a float
        dict.__setitem__(by_path[path], key, float(value) if isinstance(value, np.ndarray) and shape == () else value)
        start += size
    return start


def copy_state(state, convert):
    """A copy of the state tree with its float arrays passed through convert; initials are shared, not copied."""
    new = type(state)()
    dict.clear(new)
    # object attributes too (such as the conditions size), but not the binding to the preallocated layout
    new.__dict__.update({key: value for key, value in state.__dict__.items() if key != 'segment_layout'})
    for key, value in dict.items(state):
        if key == 'initials':
            dict.__setitem__(new, key, value)
        elif isinstance(value, Data):
            dict.__setitem__(new, key, copy_state(value, convert))
        elif isinstance(value, np.ndarray) and value.dtype.kind == 'f':
            dict.__setitem__(new, key, convert(value))
        else:
            dict.__setitem__(new, key, copy.copy(value))
    return new
