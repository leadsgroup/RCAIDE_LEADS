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
       compiled    Compiled_Segment with residual(x, inputs), jacobian(x, inputs) and
                   input_derivative(x, inputs, direction)
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
        self.residual         = jax.jit(residual)
        self.jacobian         = jax.jit(jax.jacfwd(residual))
        self.input_derivative = jax.jit(input_derivative)

# ----------------------------------------------------------------------------------------------------------------------
#  The residual as a function of the unknowns and the inputs
# ----------------------------------------------------------------------------------------------------------------------
def residual_of_inputs(segment, x, paths, values):
    """
    Residual of the segment's iterate steps with unknowns x and the float inputs at paths replaced by values, on NumPy
    or JAX. The steps run on a copy of the state, and the analyses are restored afterwards.
    """
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
        xp = array_namespace(x, *values)
        return xp.concatenate([xp.ravel(value) for value in float_leaves(fresh.residuals)])
    finally:
        segment.state, segment.conditions = state, state.conditions
        for reference, value in zip(replaced, saved):
            write_input(reference, value)

# ----------------------------------------------------------------------------------------------------------------------
#  Inputs and signature of a segment
# ----------------------------------------------------------------------------------------------------------------------
def segment_inputs(segment):
    """
    Walks the analyses, the state and the initials of a segment (one level, not the chain of earlier segments).

    Outputs:
       references   one (container, key, kind, path) per float input: finite floats and finite float arrays
       signature    everything else, by value (types, tags, flags, integers, non-finite floats, the shapes of the float
                    inputs and the iterate steps): segments with equal signatures run the same compiled residual
    """
    references = []
    static     = [('segment', type(segment).__qualname__)]
    static    += [('step', describe(step)) for step in iterate_steps(segment.process.iterate)]
    collect_inputs(segment.analyses, 'analyses', references, static, set())
    collect_inputs(segment.state, 'state', references, static, set(), skip_initials=True)
    if segment.state.initials:
        collect_inputs(segment.state.initials, 'initials', references, static, set(), skip_initials=True)
    return references, tuple(static)


def collect_inputs(item, path, references, static, seen, skip_initials=False):
    """Appends the float inputs under item to references and everything else to static."""
    if id(item) in seen:
        static.append((path, 'seen'))
        return
    seen.add(id(item))
    static.append((path, type(item).__qualname__))

    if isinstance(item, Linear_Interpolator):
        for i in range(len(item.axes)):
            collect_value(item.axes, i, item.axes[i], 'item', f"{path}.axes[{i}]", references, static, seen)
        collect_value(item, 'values', item.values, 'attribute', f"{path}.values", references, static, seen)
    elif isinstance(item, dict):
        for key, value in dict.items(item):
            if not (skip_initials and key == 'initials'):
                collect_value(item, key, value, 'item', f"{path}.{key}", references, static, seen)
        # object attributes of a Data (such as the conditions size), not the binding to the preallocated layout
        for key, value in getattr(item, '__dict__', {}).items():
            if key != 'segment_layout':
                static.append((f"{path}:{key}", describe(value)))
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


def unknowns_vector(unknowns):
    """The unknowns of a state as one flat NumPy vector."""
    return np.concatenate([np.ravel(value) for value in float_leaves(unknowns)])


def set_unknowns(tree, numpy_tree, x):
    """Writes the flat vector x into the unknowns of tree, in the order and shapes of the NumPy unknowns."""
    start = 0
    for key, value in dict.items(numpy_tree):
        if isinstance(value, dict):
            start += set_unknowns(tree[key], value, x[start:])
        elif isinstance(value, np.ndarray) and value.dtype.kind == 'f':
            dict.__setitem__(tree, key, x[start:start + value.size].reshape(value.shape))
            start += value.size
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
