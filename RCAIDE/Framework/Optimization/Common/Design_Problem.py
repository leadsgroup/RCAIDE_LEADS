# RCAIDE/Framework/Optimization/Common/Design_Problem.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import copy
import numpy as np
import scipy.optimize

from RCAIDE.Framework.Core import Data, DataOrdered

# ----------------------------------------------------------------------------------------------------------------------
#  Design_Problem
# ----------------------------------------------------------------------------------------------------------------------
class Design_Problem(Data):
    """
    A design optimization problem on one vehicle flown over one or more missions, with gradients from the mission
    derivative.

    Attributes
    ----------
    vehicle : Vehicle
        The vehicle, as the vehicle script built it.
    missions : Data
        {tag: mission}, built on that vehicle and not solved.
    variables : list
        [path, initial, lower bound, upper bound, scale] per variable. A path starts at 'vehicle' or 'missions', such
        as 'vehicle.wings.main_wing.spans.projected' or 'missions.design.segments.cruise.distance'.
    update_vehicle : function, optional
        update_vehicle(vehicle), run after the variables are written: couplings only, such as a tail sized with the
        wing. It may only change values that a class declares.
    update_missions : function, optional
        update_missions(missions), run after update_vehicle: couplings only, such as a climb that ends at the cruise
        altitude. It may only change declared floats of the segments.
    outputs : function
        outputs(missions) gives {name: value} from the solved missions, written with array operations (no float())
        so that a reverse derivative can trace it.
    objective : list
        [output name, scale], minimized.
    constraints : list
        [output name, sense, edge, scale] per constraint, sense '>', '<' or '='.
    relative_step : float
        Step of the finite differences, relative to each variable's scale.
    relative_steps : dict
        {variable path: relative step} for variables that need their own step (see sweep_relative_steps).
    derivative_method : str
        'mission_derivative' differences the setup and solves the linear system of every segment;
        'finite_difference' solves the missions again at the two moved designs (the baseline without gradients);
        'reverse' takes one backward sweep of the missions and one reverse pass of the surrogate training per output,
        for every variable at once (see reverse_derivatives); outputs must then be written with array operations.
    moved_models : Data
        Two copies of the vehicle and missions for the setup at the moved designs, made by check_design_problem.
    compiled_lattice_cases : dict
        {mission tag: Compiled_Lattice_Cases} of the surrogate training, made by the first reverse derivative.

    Notes
    -----
    A variable must be a float that a component or a mission segment class declares, that the setup does not
    compute, and that changes the missions or the outputs. check_design_problem refuses any other path.

    Derivatives are forward, one pass per variable: with the mission derivative each pass differences the setup at
    two moved designs and solves the linear system of every segment, without solving the missions again; with finite
    differences each pass solves the missions at the two moved designs.
    """
    def __defaults__(self):
        self.vehicle           = None
        self.missions          = Data()
        self.variables         = []
        self.update_vehicle    = None
        self.update_missions   = None
        self.outputs           = None
        self.objective         = None
        self.constraints       = []
        self.relative_step     = 1e-4
        self.relative_steps    = {}
        self.derivative_method = 'mission_derivative'
        self.moved_models      = None
        self.compiled_lattice_cases = None

# ----------------------------------------------------------------------------------------------------------------------
#  check_design_problem
# ----------------------------------------------------------------------------------------------------------------------
def check_design_problem(problem):
    """
    Checks the variables, update_vehicle and update_missions of a design problem, and makes the copies for the moved
    designs.

    Parameters
    ----------
    problem : Design_Problem
        Not yet evaluated.

    Returns
    -------
    None
        problem.moved_models holds two copies of the vehicle and missions.

    Notes
    -----
    Each check raises a ValueError naming the path:
       1. the path resolves;
       2. the value's owner is a component, the vehicle or a mission segment, and its class declares the key;
       3. the value is a float (an integer is structure);
       4. update_vehicle adds no value that no class declares, changes no undeclared value and no variable;
       5. update_missions adds no value to a segment and changes only declared floats of the segments, not variables.
    That the setup does not compute a variable, and that a variable changes something, is checked by the first
    evaluation.
    """
    from RCAIDE.Library.Mission.Common.Pre_Process import undeclared_vehicle_paths

    model = Data(vehicle=problem.vehicle, missions=problem.missions)
    paths = [variable[0] for variable in problem.variables]
    for path in paths:
        check_variable_path(model, path)

    if problem.update_vehicle is not None:
        undeclared = undeclared_vehicle_paths(problem.vehicle)
        before     = {path: copy.deepcopy(value_at(model, path)) for path in undeclared + paths}
        problem.update_vehicle(problem.vehicle)
        added      = [path for path in undeclared_vehicle_paths(problem.vehicle) if path not in undeclared]
        if len(added) > 0:
            raise ValueError('update_vehicle added values that no class declares: ' + ', '.join(added))
        for path, value in before.items():
            if not same_value(value, value_at(model, path)):
                kind = 'a variable' if path in paths else 'a value that no class declares'
                raise ValueError(f"update_vehicle changed {path}, {kind}")

    if problem.update_missions is not None:
        before = segment_values(problem.missions)
        problem.update_missions(problem.missions)
        after  = segment_values(problem.missions)
        added  = [path for path in after if path not in before]
        if len(added) > 0:
            raise ValueError('update_missions added values to segments: ' + ', '.join(added))
        for path, value in before.items():
            if same_value(value, after[path]):
                continue
            if path in paths:
                raise ValueError(f"update_missions changed {path}, a variable")
            check_variable_path(model, path)

    # copied before any setup, together so that the copied missions fly the copied vehicle
    problem.moved_models = Data(plus=copy.deepcopy(model), minus=copy.deepcopy(model))

    # the undeclared values of the vehicle script are listed once, by the problem's own missions
    for moved in problem.moved_models.values():
        for mission in moved.missions.values():
            mission.undeclared_vehicle_values = []
    return


def check_variable_path(model, path):
    """Raises a ValueError unless the path is a float that a component, the vehicle or a segment class declares."""
    keys = path.split('.')
    if keys[0] not in ('vehicle', 'missions'):
        raise ValueError(f"{path}: a variable path starts at 'vehicle' or 'missions'")

    # the nearest owner with its own class (not a plain Data) declares the rest of the path
    node  = model
    owner = None
    start = 0
    for index, key in enumerate(keys):
        if not isinstance(node, (Data, DataOrdered)) or key not in node.keys():
            raise ValueError(f"{path}: '{key}' does not resolve")
        if index == len(keys) - 1:
            break
        node = node[key]
        if isinstance(node, (Data, DataOrdered)) and type(node) not in (Data, DataOrdered):
            owner = node
            start = index + 1
    value = node[keys[-1]]

    module = type(owner).__module__ if owner is not None else ''
    if not (module.startswith('RCAIDE.Library.Components') or module == 'RCAIDE.Vehicle' or module.startswith('RCAIDE.Framework.Mission.Segments')):
        raise ValueError(f"{path}: the owner is not a component, the vehicle or a mission segment")

    default = type(owner)()
    for key in keys[start:]:
        if not isinstance(default, (Data, DataOrdered)) or key not in default.keys():
            raise ValueError(f"{path}: {type(owner).__name__} does not declare '{'.'.join(keys[start:])}'")
        default = default[key]

    if isinstance(value, bool) or not isinstance(value, (float, np.floating)):
        raise ValueError(f"{path}: the value is a {type(value).__name__}, not a float")
    return


def segment_values(missions):
    """{path: copy of the value} of every value a mission's segments hold, outside their state, analyses and process."""
    values = {}
    for tag, mission in missions.items():
        for segment in mission.segments.values():
            path = f"missions.{tag}.segments.{segment.tag}"
            collect_values(segment, path, values, set(), skip=('state', 'analyses', 'process', 'conditions'))
    return values


def collect_values(node, path, values, seen, skip=()):
    """Adds a copy of every value under node that is not itself a Data, by path."""
    if id(node) in seen:
        return
    seen.add(id(node))
    for key in node.keys():
        if key in skip:
            continue
        value = node[key]
        if isinstance(value, (Data, DataOrdered)):
            collect_values(value, path + '.' + str(key), values, seen)
        else:
            values[path + '.' + str(key)] = copy.deepcopy(value)
    return


def value_at(model, path):
    """The value at a path from the model (vehicle and missions)."""
    node = model
    for key in path.split('.'):
        node = node[key]
    return node


def write_value(model, path, value):
    """Writes a value at a path from the model."""
    keys = path.split('.')
    node = model
    for key in keys[:-1]:
        node = node[key]
    node[keys[-1]] = value
    return


def same_value(first, second):
    """True when two values are equal, arrays element by element."""
    try:
        return bool(np.all(np.asarray(first) == np.asarray(second)))
    except Exception:
        return first is second

# ----------------------------------------------------------------------------------------------------------------------
#  evaluate_design
# ----------------------------------------------------------------------------------------------------------------------
def evaluate_design(problem, design, derivatives=True):
    """
    The outputs of a design and, when asked, their derivatives with respect to every variable.

    The missions are solved once on the problem's vehicle. For each variable the two moved copies are set up, not
    solved, at the design moved up and down by one step; mission_derivative gives the change of every segment's
    converged state between them, and the outputs are read from the moved copies at their moved states. The central
    difference of the two is the derivative, so the setup and the output formulas are differenced and the solve is
    not. With problem.derivative_method 'finite_difference' the moved copies are solved instead.

    Parameters
    ----------
    problem : Design_Problem
        Checked by check_design_problem.
    design : dict
        {variable path: value}
    derivatives : bool
        False gives the outputs only.

    Returns
    -------
    result : Data
        result.outputs       {output name: value}
        result.derivatives   {output name: array, one entry per variable in the order of problem.variables}
    """
    # imported here: the mission solver itself imports the optimization package
    from RCAIDE.Library.Mission.Solver.mission_derivative import mission_derivative, moved_state

    from .reverse_derivatives import reverse_derivatives, training_analysis

    if problem.derivative_method not in ('mission_derivative', 'finite_difference', 'reverse'):
        raise ValueError(f"Unsupported derivative method '{problem.derivative_method}'. Supported values are "
                         f"'mission_derivative', 'finite_difference' and 'reverse'.")

    first_evaluation = problem.moved_models is None
    if first_evaluation:
        check_design_problem(problem)

    model = Data(vehicle=problem.vehicle, missions=problem.missions)
    set_design(problem, model, design)

    # a variable the setup computes is overwritten by it: checked before anything is solved
    if first_evaluation:
        for mission in problem.missions.values():
            mission.process.initialize(mission)
        for path, value in design.items():
            if value_at(model, path) != value:
                raise ValueError(f"{path}: the setup computes this value ({value} written, {value_at(model, path)} after the setup)")

    for mission in problem.missions.values():
        mission.evaluate()
    outputs = {name: float(value) for name, value in problem.outputs(problem.missions).items()}
    result  = Data(outputs=outputs, derivatives=None)

    if not derivatives:
        return result

    if problem.derivative_method == 'reverse':
        result.derivatives = reverse_derivatives(problem, design, outputs)
        return result

    # the moved copies train their own surrogates (a reverse derivative may have left them holding the design's)
    for moved in problem.moved_models.values():
        for mission in moved.missions.values():
            owner = training_analysis(mission)
            if owner is not None:
                owner.settings.train_surrogates = True

    result.derivatives = {name: np.zeros(len(problem.variables)) for name in outputs}
    plus, minus        = problem.moved_models.plus, problem.moved_models.minus
    for index, variable in enumerate(problem.variables):
        path = variable[0]
        step = problem.relative_steps.get(path, problem.relative_step) * variable[4]
        for moved, move in ((plus, +step), (minus, -step)):
            moved_design       = dict(design)
            moved_design[path] = design[path] + move
            set_design(problem, moved, moved_design)

        changed = False
        if problem.derivative_method == 'finite_difference':
            # each moved copy solved again
            for moved in (plus, minus):
                for moved_mission in moved.missions.values():
                    moved_mission.evaluate()
        else:
            # each moved copy at its moved converged state: the solved state moved along its derivative
            for tag, mission in problem.missions.items():
                states = mission_derivative(mission, plus.missions[tag], step, minus.missions[tag])
                for moved, move in ((plus, +step), (minus, -step)):
                    for segment, moved_segment, derivative in zip(mission.segments.values(), moved.missions[tag].segments.values(), states):
                        moved_segment.state      = moved_state(segment.state, derivative.state, move)
                        moved_segment.conditions = moved_segment.state.conditions
                for derivative in states:
                    for array in derivative.state.values():
                        if np.any(np.asarray(array) != 0):
                            changed = True

        plus_outputs  = {name: float(value) for name, value in problem.outputs(plus.missions).items()}
        minus_outputs = {name: float(value) for name, value in problem.outputs(minus.missions).items()}
        for name in outputs:
            result.derivatives[name][index] = (plus_outputs[name] - minus_outputs[name]) / (2 * step)
            if result.derivatives[name][index] != 0:
                changed = True

        if first_evaluation and not changed:
            raise ValueError(f"{path}: a step of this value changes no mission state and no output")
    return result


def sweep_relative_steps(problem, design, relative_steps):
    """
    The derivatives of a design at several relative steps, to choose a step per variable: too large a step mixes in
    the curvature, too small a step is lost in the noise of the solve and the surrogates.

    Parameters
    ----------
    problem : Design_Problem
    design : dict
        {variable path: value}
    relative_steps : list of float
        Steps relative to each variable's scale, applied to every variable at once.

    Returns
    -------
    sweep : dict
        {relative step: {output name: array, one entry per variable}}
    """
    stored = problem.relative_steps
    sweep  = {}
    try:
        for relative_step in relative_steps:
            problem.relative_steps = {variable[0]: relative_step for variable in problem.variables}
            sweep[relative_step]   = evaluate_design(problem, design).derivatives
    finally:
        problem.relative_steps = stored
    return sweep


def set_design(problem, model, design):
    """Writes the design onto a model (vehicle and missions) and runs update_vehicle and update_missions."""
    for path, value in design.items():
        write_value(model, path, value)
    if problem.update_vehicle is not None:
        problem.update_vehicle(model.vehicle)
    if problem.update_missions is not None:
        problem.update_missions(model.missions)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  solve_design_problem
# ----------------------------------------------------------------------------------------------------------------------
def solve_design_problem(problem, optimizer='SLSQP', iterations=100, tolerance=1e-6):
    """
    Minimizes the objective of a design problem with a SciPy optimizer that takes gradients ('SLSQP' or
    'trust-constr'). Variables, objective and constraints are divided by their scales.

    Parameters
    ----------
    problem : Design_Problem
    optimizer : str
        SciPy minimize method.
    iterations : int
        Most optimizer iterations.
    tolerance : float
        Optimizer tolerance on the scaled problem.

    Returns
    -------
    solution : Data
        solution.design      {variable path: value} at the optimum
        solution.outputs     {output name: value} at the optimum
        solution.history     one Data(design, outputs) per design evaluated, in order
        solution.optimizer   the SciPy result
    """
    paths   = [variable[0] for variable in problem.variables]
    scales  = np.array([variable[4] for variable in problem.variables], dtype=float)
    x0      = np.array([variable[1] for variable in problem.variables], dtype=float) / scales
    bounds  = [(variable[2] / variable[4], variable[3] / variable[4]) for variable in problem.variables]
    history = []
    last    = Data(x=None, result=None)

    # the optimizer asks for the objective, the constraints and their gradients separately at the same design
    def evaluate(x):
        if last.x is None or not np.array_equal(x, last.x):
            design      = {path: float(value) for path, value in zip(paths, x * scales)}
            last.result = evaluate_design(problem, design)
            last.x      = np.array(x)
            history.append(Data(design=design, outputs=last.result.outputs))
            print(f"  design {len(history)}: " + ", ".join(f"{path} = {value:.6g}" for path, value in design.items()))
            print("     " + ", ".join(f"{name} = {value:.6g}" for name, value in last.result.outputs.items()))
        return last.result

    objective_name, objective_scale = problem.objective
    def objective(x):
        result = evaluate(x)
        return result.outputs[objective_name] / objective_scale, result.derivatives[objective_name] * scales / objective_scale

    # SciPy inequality constraints are satisfied when positive
    constraints = []
    for name, sense, edge, scale in problem.constraints:
        if sense not in ('>', '<', '='):
            raise ValueError(f"Unsupported constraint sense '{sense}' for {name}. Supported values are '>', '<', '='.")
        sign = -1. if sense == '<' else 1.
        def value(x, name=name, edge=edge, scale=scale, sign=sign):
            return sign * (evaluate(x).outputs[name] - edge) / scale
        def gradient(x, name=name, scale=scale, sign=sign):
            return sign * evaluate(x).derivatives[name] * scales / scale
        constraints.append({'type': 'eq' if sense == '=' else 'ineq', 'fun': value, 'jac': gradient})

    result = scipy.optimize.minimize(objective, x0, jac=True, method=optimizer, bounds=bounds, constraints=constraints,
                                     tol=tolerance, options={'maxiter': iterations})
    final  = evaluate(result.x)
    return Data(design={path: float(value) for path, value in zip(paths, result.x * scales)}, outputs=final.outputs,
                history=history, optimizer=result)
