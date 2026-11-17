# RCAIDE/Framework/Optimization/Common/reverse_derivatives.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import copy
import numpy as np

from RCAIDE.Framework.Core import Data

# ----------------------------------------------------------------------------------------------------------------------
#  reverse_derivatives
# ----------------------------------------------------------------------------------------------------------------------
def reverse_derivatives(problem, design, outputs):
    """
    Derivatives of every output of a solved design problem with respect to every variable, in reverse mode.

    Parameters
    ----------
    problem : Design_Problem
        Its missions solved at the design.
    design : dict
        {variable path: value}
    outputs : dict
        {output name: value} at the design.

    Returns
    -------
    derivatives : dict
        {output name: array, one entry per variable}

    Notes
    -----
    Per output, one backward sweep over each mission (mission_adjoint) gives its derivative with respect to the inputs
    of every segment, and one reverse pass of the surrogate training gives the part through the surrogate tables with
    respect to the numbers they are trained from (the defining numbers of every wing, the reference values and the
    center of gravity). Per variable, the two moved copies are set up without training and without solving; the
    change of their inputs, of the numbers the tables are trained from, and of the outputs read from the vehicle
    complete the derivative. A variable that reaches the tables through anything else raises a ValueError.
    """
    from RCAIDE.Library.Mission.Solver.mission_adjoint import mission_adjoint, inputs_after_setup
    from RCAIDE.Library.Mission.Solver.compiled_residual import compile_segment, unknowns_vector, copy_state

    names        = list(outputs.keys())
    plus, minus  = problem.moved_models.plus, problem.moved_models.minus
    tags         = list(problem.missions.keys())
    from .Design_Problem import set_design

    # one backward sweep per output and mission; the setup copy is the plus copy at the design itself, which builds
    # its surrogates from the tables of the design without training
    seeds = output_seeds(problem, names)
    set_design(problem, plus, design)
    for tag in tags:
        hold_training(problem.missions[tag], plus.missions[tag])
    adjoints = {name: {tag: mission_adjoint(problem.missions[tag], seeds[name][tag], plus.missions[tag])
                       if len(seeds[name][tag]) > 0 else None for tag in tags} for name in names}

    # the part through the surrogate tables: dJ/d(tables), then one reverse pass to the numbers they are trained from
    table_weights = {name: {tag: table_cotangents(problem.missions[tag], adjoints[name][tag]) for tag in tags} for name in names}
    table_numbers = {}
    for tag in tags:
        owner = training_analysis(problem.missions[tag])
        if owner is None or all(len(table_weights[name][tag]) == 0 for name in names):
            continue
        if problem.compiled_lattice_cases is None:
            problem.compiled_lattice_cases = {}
        if tag not in problem.compiled_lattice_cases:
            from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.compiled_lattice_cases import Compiled_Lattice_Cases
            from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates import train_model
            problem.compiled_lattice_cases[tag] = Compiled_Lattice_Cases(owner, owner.training_vehicle)
            # compiled once on plain values: a first trace inside the reverse pass would capture the vehicle's traced
            # values as constants
            train_model(owner, owner.training.Mach[owner.training.Mach < 1], owner.training_vehicle, compiled=problem.compiled_lattice_cases[tag])
        table_numbers[tag] = {name: table_number_cotangents(owner, problem.compiled_lattice_cases[tag], table_weights[name][tag])
                              for name in names if len(table_weights[name][tag]) > 0}

    derivatives = {name: np.zeros(len(problem.variables)) for name in names}
    for index, variable in enumerate(problem.variables):
        path = variable[0]
        step = problem.relative_steps.get(path, problem.relative_step) * variable[4]
        for moved, move in ((plus, +step), (minus, -step)):
            moved_design       = dict(design)
            moved_design[path] = design[path] + move
            set_design(problem, moved, moved_design)
            for tag in tags:
                hold_training(problem.missions[tag], moved.missions[tag])
                moved.missions[tag].process.initialize(moved.missions[tag])

        for tag in tags:
            mission  = problem.missions[tag]
            segments = list(mission.segments.values())

            # the change of each segment's inputs, set up at the moved designs from the solved segment before it
            sides = []
            for moved in (plus, minus):
                moved_segments = list(moved.missions[tag].segments.values())
                inputs         = []
                for k, segment in enumerate(segments):
                    compiled, _ = compile_segment(segment)
                    initials    = segments[k - 1].state if k > 0 else moved_segments[k].state.initials
                    inputs.append(inputs_after_setup(moved_segments[k], initials, unknowns_vector(segment.state.unknowns), compiled))
                sides.append(inputs)
            for name in names:
                adjoint = adjoints[name][tag]
                if adjoint is None:
                    continue
                for k in range(len(segments)):
                    for input_path, weight, upper, lower in zip(adjoint[k].paths, adjoint[k].inputs, sides[0][k], sides[1][k]):
                        if is_table_path(input_path):
                            continue
                        derivatives[name][index] += float(np.sum(np.asarray(weight) * (upper - lower) / (2 * step)))

            # the change of the numbers the tables are trained from, checked to explain the change of the surrogate inputs
            if tag in table_numbers:
                numbers = [training_numbers(training_analysis(moved.missions[tag])) for moved in (plus, minus)]
                for moved in (plus, minus):
                    check_table_numbers(training_analysis(problem.missions[tag]), training_analysis(moved.missions[tag]))
                for name, cotangents in table_numbers[tag].items():
                    for weight, upper, lower in zip(cotangents, numbers[0], numbers[1]):
                        derivatives[name][index] += float(np.sum(np.asarray(weight) * (upper - lower) / (2 * step)))

        # outputs that read the moved vehicle directly, at the solved state of the design
        direct = []
        for moved in (plus, minus):
            for tag in tags:
                for segment, moved_segment in zip(problem.missions[tag].segments.values(), moved.missions[tag].segments.values()):
                    moved_segment.state      = copy_state(segment.state, np.array)
                    moved_segment.conditions = moved_segment.state.conditions
            direct.append(problem.outputs(moved.missions))
        for name in names:
            derivatives[name][index] += (float(direct[0][name]) - float(direct[1][name])) / (2 * step)
    return derivatives

# ----------------------------------------------------------------------------------------------------------------------
#  Seeds of the outputs
# ----------------------------------------------------------------------------------------------------------------------
def output_seeds(problem, names):
    """{output name: {mission tag: {segment tag: {state path: array}}}}: the derivative of each output with respect to
    the state arrays it reads, by JAX through problem.outputs."""
    import jax
    import jax.numpy as jnp
    from RCAIDE.Library.Mission.Solver.compiled_residual import state_arrays

    located = []
    for tag, mission in problem.missions.items():
        for segment in mission.segments.values():
            for container, key, path in state_arrays(segment.state):
                located.append((tag, segment.tag, container, key, path))
    originals = [container[key] for (_, _, container, key, _) in located]

    def values_of(arrays):
        try:
            for (_, _, container, key, _), array in zip(located, arrays):
                container[key] = array
            values = problem.outputs(problem.missions)
            return tuple(jnp.asarray(values[name], dtype=float) for name in names)
        finally:
            for (_, _, container, key, _), array in zip(located, originals):
                container[key] = array

    try:
        _, pullback = jax.vjp(values_of, [jnp.asarray(array, dtype=float) for array in originals])
    except (jax.errors.ConcretizationTypeError, jax.errors.TracerArrayConversionError) as error:
        raise ValueError("reverse derivatives trace problem.outputs with JAX: write it with array operations, "
                         "without float() or numpy conversions of the mission values") from error

    seeds = {name: {tag: {} for tag in problem.missions.keys()} for name in names}
    for n, name in enumerate(names):
        weights = [jnp.zeros(()) for _ in names]
        weights[n] = jnp.ones(())
        cotangents, = pullback(tuple(weights))
        for (tag, segment_tag, _, _, path), cotangent in zip(located, cotangents):
            cotangent = np.asarray(cotangent)
            if np.any(cotangent != 0):
                seeds[name][tag].setdefault(segment_tag, {})[path] = cotangent
    return seeds

# ----------------------------------------------------------------------------------------------------------------------
#  Surrogate tables
# ----------------------------------------------------------------------------------------------------------------------
def training_analysis(mission):
    """The vortex lattice analysis that trains the surrogates the mission's segments share: the first segment's."""
    from RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method import Vortex_Lattice_Method
    for segment in mission.segments.values():
        aerodynamics = segment.analyses.aerodynamics
        if isinstance(aerodynamics, Vortex_Lattice_Method):
            return aerodynamics
    return None


def hold_training(mission, moved_mission):
    """The moved copy's training analysis builds its surrogates from the tables of the mission at its next setup,
    without training (train_surrogates False holds for one initialize)."""
    owner, moved = training_analysis(mission), training_analysis(moved_mission)
    if owner is None:
        return
    moved.settings.train_surrogates = False
    moved.training.subsonic         = owner.training.subsonic
    moved.training.supersonic       = owner.training.supersonic
    moved.training.transonic        = owner.training.transonic
    # the training also records which control surface tables it made
    for key in owner.keys():
        if str(key).endswith('_flag'):
            moved[key] = owner[key]
    return


def is_table_path(path):
    """True for an input of a segment that is a value of a surrogate table."""
    return '.surrogates.' in path and path.endswith('.values')


def table_cotangents(mission, adjoint):
    """{table key: dJ/d(table)} summed over the segments, from the adjoint of one output."""
    weights = {}
    if adjoint is None:
        return weights
    for derivative in adjoint:
        for path, weight in zip(derivative.paths, derivative.inputs):
            if not is_table_path(path):
                continue
            keys = path.split('.')
            regime, key = keys[-3], keys[-2]
            weight = np.asarray(weight)
            if regime != 'subsonic':
                if np.any(weight != 0):
                    raise ValueError(f"{path}: reverse derivatives cover the subsonic surrogate tables only")
                continue
            weights[key] = weights.get(key, 0.) + weight
    return weights


def training_numbers(owner):
    """The numbers the tables are trained from, on the analysis' training vehicle: the defining numbers of every wing,
    the reference area, chord and span, and the center of gravity; a list of float arrays."""
    from RCAIDE.Library.Methods.Geometry.Planform import wing_defining_numbers
    vehicle = owner.training_vehicle
    numbers = []
    for wing in vehicle.wings:
        numbers += list(wing_defining_numbers(wing)[1]())
    center   = vehicle.mass_properties.center_of_gravity
    numbers += [np.array([vehicle.reference_area, vehicle.reference_chord, vehicle.reference_span], dtype=float),
                np.array([center[0][0], center[0][1], center[0][2]], dtype=float)]
    return numbers


def write_training_numbers(vehicle, numbers):
    """Writes training_numbers to a vehicle and recomputes its wings; the values can be JAX arrays."""
    from RCAIDE.Library.Methods.Geometry.Planform import wing_defining_numbers, wing_planform
    position = 0
    for wing in vehicle.wings:
        names, read, write = wing_defining_numbers(wing)
        write(*numbers[position:position + len(names)])
        position += len(names)
        wing_planform(wing)
    references = numbers[position]
    center     = numbers[position + 1]
    vehicle.reference_area, vehicle.reference_chord, vehicle.reference_span = references[0], references[1], references[2]
    vehicle.mass_properties.center_of_gravity = [[center[0], center[1], center[2]]]
    return


def table_number_cotangents(owner, compiled, weights):
    """dJ/d(training numbers) for dJ/d(tables) given as {table key: array}: one reverse pass of the training."""
    import jax
    import jax.numpy as jnp
    from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates import train_model

    vehicle  = owner.training_vehicle
    original = training_numbers(owner)
    Mach     = owner.training.Mach[owner.training.Mach < 1]
    keys     = list(weights.keys())

    def weighted_tables(numbers):
        try:
            write_training_numbers(vehicle, numbers)
            training = train_model(owner, Mach, vehicle, compiled=compiled)
            for key in keys:
                if key not in training:
                    raise ValueError(f"surrogate table '{key}' is not a table of the training")
            return sum(jnp.sum(jnp.asarray(weights[key]) * training[key]) for key in keys)
        finally:
            write_training_numbers(vehicle, original)

    gradient = jax.grad(weighted_tables)([jnp.asarray(value) for value in original])
    return [np.asarray(value) for value in gradient]


def check_table_numbers(owner, moved_owner, tolerance=1e-9):
    """Raises a ValueError when the surrogate inputs of a moved training vehicle are not those of the design's vehicle
    with the moved training numbers written to it: the variable then reaches the tables through something else."""
    from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method import surrogate_inputs
    rebuilt = copy.deepcopy(owner.training_vehicle)
    write_training_numbers(rebuilt, training_numbers(moved_owner))
    expected = surrogate_inputs(moved_owner, rebuilt)
    actual   = surrogate_inputs(moved_owner, moved_owner.training_vehicle)
    differ   = []
    for path in set(expected) | set(actual):
        a, b = expected.get(path), actual.get(path)
        try:
            a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
            if a.shape != b.shape or np.any(np.abs(a - b) > tolerance * np.maximum(1., np.abs(b))):
                differ.append(path)
        except (TypeError, ValueError):
            if not (a == b if not isinstance(a, np.ndarray) else np.array_equal(a, b)):
                differ.append(path)
    if len(differ) > 0:
        raise ValueError("a variable reaches the surrogate tables through values the reverse pass does not cover: "
                         + ', '.join(sorted(differ)[:10]))
    return
