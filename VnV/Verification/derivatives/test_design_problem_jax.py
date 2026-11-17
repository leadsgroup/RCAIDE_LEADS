# VnV/Verification/derivatives/test_design_problem_jax.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
import numpy as np

from RCAIDE.Framework.Core import Units, Data, base_vehicle

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # A Design_Problem holds the Cessna 172, its mission (climb, cruise, descent), the variables as paths, and the
    # outputs, objective and constraints. Its derivatives are compared with central differences of missions built
    # and solved again, the same design evaluated after another must give the same outputs, and a few optimizer
    # iterations must lower the fuel burned without losing range.
    try:
        import jax
    except ImportError:
        print("JAX is not installed (pip install RCAIDE[ad]); JAX checks skipped")
        return
    jax.config.update("jax_enable_x64", True)

    vnv_folder   = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    start_folder = os.getcwd()
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, os.path.join(vnv_folder, "Vehicles"))
    os.chdir(os.path.join(vnv_folder, "Verification", "network_internal_combustion_engine"))
    sys.path.insert(0, os.getcwd())

    check_refused_paths()
    check_computed_value_refused()
    check_refused_mission_updates()
    problem = define_problem()
    check_output_derivatives(problem, tolerance=1e-5)
    check_finite_difference_method(problem, tolerance=1e-5)
    check_reverse_method(problem, tolerance=1e-5)
    check_repeated_design(problem, tolerance=1e-9)
    check_optimization(define_problem())

    os.chdir(start_folder)
    print('test_design_problem_jax passed')
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_refused_paths():
    # paths that are not declared floats of a component or a segment are refused before anything is solved
    from RCAIDE.Framework.Optimization.Common.Design_Problem import check_variable_path

    vehicle, missions = build_model()
    model   = Data(vehicle=vehicle, missions=missions)
    refused = ['vehicle.wings.main_wing.made_up_value',                  # does not resolve
               'vehicle.wings.main_wing.areas.made_up_area',              # does not resolve
               'missions.design.segments.cruise.analyses.aerodynamics',   # owner is an analysis
               'missions.design.segments.cruise.state.numerics.number_of_control_points',
               'vehicle.networks.fuel.propulsors.ice_propeller.engine.origin']
    vehicle.wings.main_wing.made_up_by_script = 1.0
    refused.append('vehicle.wings.main_wing.made_up_by_script')             # set by a script, declared by no class
    for path in refused:
        try:
            check_variable_path(model, path)
        except ValueError as error:
            print(f"  refused {path}: {error}")
            continue
        raise AssertionError(f"{path} was accepted as a variable")

    for path in ['vehicle.wings.main_wing.aspect_ratio', 'vehicle.mass_properties.takeoff',
                 'missions.design.segments.cruise.distance']:
        check_variable_path(model, path)
    return


def check_computed_value_refused():
    # the Cessna wing is defined by its area and aspect ratio, so the setup computes the span and a span variable is
    # refused before the mission is solved
    from RCAIDE.Framework.Optimization.Common.Design_Problem import Design_Problem, evaluate_design

    problem                           = Design_Problem()
    problem.vehicle, problem.missions = build_model()
    problem.outputs                   = mission_outputs
    span                              = float(problem.vehicle.wings.main_wing.spans.projected)
    problem.variables                 = [['vehicle.wings.main_wing.spans.projected', 1.05 * span, span, 1.1 * span, span]]
    try:
        evaluate_design(problem, {'vehicle.wings.main_wing.spans.projected': 1.05 * span})
    except ValueError as error:
        print(f"  refused: {error}")
        assert 'the setup computes this value' in str(error), str(error)
        return
    raise AssertionError('a span variable was accepted although the setup computes the span')


def check_refused_mission_updates():
    # update_missions may change declared floats of the segments, but not a variable and not a new key
    from RCAIDE.Framework.Optimization.Common.Design_Problem import check_design_problem

    def write_variable(missions):
        missions.design.segments.cruise.distance = 2. * missions.design.segments.cruise.distance

    def add_key(missions):
        missions.design.segments.climb.made_up_value = 1.0

    for update, expected in ((write_variable, 'a variable'), (add_key, 'added values to segments')):
        problem                 = define_problem(first_range=1.)
        problem.update_missions = update
        try:
            check_design_problem(problem)
        except ValueError as error:
            print(f"  refused: {error}")
            assert expected in str(error), str(error)
            continue
        raise AssertionError(f"update_missions {update.__name__} was accepted")
    return


def check_output_derivatives(problem, tolerance):
    from RCAIDE.Framework.Optimization.Common.Design_Problem import evaluate_design, write_value

    design = {variable[0]: variable[1] for variable in problem.variables}
    result = evaluate_design(problem, design)

    # central differences of the mission built and solved again at the two moved designs
    for index, (path, initial, lower, upper, scale) in enumerate(problem.variables):
        step  = problem.relative_step * scale
        moved = []
        for move in (+step, -step):
            vehicle, missions = build_model()
            model = Data(vehicle=vehicle, missions=missions)
            for other, value in design.items():
                write_value(model, other, value)
            write_value(model, path, design[path] + move)
            problem.update_missions(missions)
            missions.design.evaluate()
            moved.append(mission_outputs(missions))
        for output in result.outputs:
            computed = result.derivatives[output][index]
            expected = (moved[0][output] - moved[1][output]) / (2 * step)
            error    = abs(computed - expected) / max(abs(expected), 1e-300)
            print(f"  d({output})/d({path}): design problem {computed:.8e}, solved again {expected:.8e}, relative difference {error:.1e}")
            assert error <= tolerance, f"d({output})/d({path}): {computed} against {expected}"
    return


def check_finite_difference_method(problem, tolerance):
    # the baseline without gradients: the moved designs solved again give the same derivatives
    from RCAIDE.Framework.Optimization.Common.Design_Problem import evaluate_design

    design = {variable[0]: variable[1] for variable in problem.variables}
    exact  = evaluate_design(problem, design).derivatives
    problem.derivative_method = 'finite_difference'
    try:
        solved = evaluate_design(problem, design).derivatives
    finally:
        problem.derivative_method = 'mission_derivative'
    for name in exact:
        for index, variable in enumerate(problem.variables):
            error = abs(solved[name][index] - exact[name][index]) / max(abs(exact[name][index]), 1e-300)
            print(f"  d({name})/d({variable[0]}): finite differences {solved[name][index]:.8e}, mission derivative {exact[name][index]:.8e}, relative difference {error:.1e}")
            assert error <= tolerance, f"d({name})/d({variable[0]}): {solved[name][index]} against {exact[name][index]}"
    return


def check_reverse_method(problem, tolerance):
    # one backward sweep of the mission and one reverse pass of the surrogate training per output give the same
    # derivatives as the forward mission derivative, one pass per variable
    import time
    from RCAIDE.Framework.Optimization.Common.Design_Problem import evaluate_design

    design = {variable[0]: variable[1] for variable in problem.variables}
    exact  = evaluate_design(problem, design).derivatives
    problem.derivative_method = 'reverse'
    try:
        start  = time.time()
        result = evaluate_design(problem, design).derivatives
        print(f"  reverse evaluation {time.time() - start:.1f} s")
    finally:
        problem.derivative_method = 'mission_derivative'
    for name in exact:
        for index, variable in enumerate(problem.variables):
            error = abs(result[name][index] - exact[name][index]) / max(abs(exact[name][index]), 1e-300)
            print(f"  d({name})/d({variable[0]}): reverse {result[name][index]:.8e}, mission derivative {exact[name][index]:.8e}, relative difference {error:.1e}")
            assert error <= tolerance, f"d({name})/d({variable[0]}): reverse {result[name][index]} against {exact[name][index]}"
    return


def check_repeated_design(problem, tolerance):
    # the vehicle persists between designs, so a design evaluated after another gives its own outputs again
    from RCAIDE.Framework.Optimization.Common.Design_Problem import evaluate_design

    design = {variable[0]: variable[1] for variable in problem.variables}
    other  = {variable[0]: variable[1] + 0.1 * variable[4] for variable in problem.variables}
    first  = evaluate_design(problem, design, derivatives=False).outputs
    evaluate_design(problem, other, derivatives=False)
    again  = evaluate_design(problem, design, derivatives=False).outputs
    for name in first:
        error = abs(again[name] - first[name]) / abs(first[name])
        print(f"  {name}: {first[name]:.12g}, after another design {again[name]:.12g}, relative difference {error:.1e}")
        assert error <= tolerance, f"{name} changed after another design: {first[name]} against {again[name]}"
    return


def check_optimization(problem):
    from RCAIDE.Framework.Optimization.Common.Design_Problem import solve_design_problem

    solution = solve_design_problem(problem, optimizer='SLSQP', iterations=5, tolerance=1e-6)
    first    = solution.history[0].outputs
    final    = solution.outputs
    print(f"  fuel burned {first['fuel_burn']:.6f} kg to {final['fuel_burn']:.6f} kg, range {first['range']:.3f} m to {final['range']:.3f} m, "
          f"{len(solution.history)} designs evaluated")
    assert final['fuel_burn'] < first['fuel_burn'], "the optimizer did not lower the fuel burned"
    assert final['range'] >= first['range'] * (1 - 1e-6), "the optimized design lost range"
    for path, initial, lower, upper, scale in problem.variables:
        assert lower <= solution.design[path] <= upper, f"{path} left its bounds"
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Problem
# ----------------------------------------------------------------------------------------------------------------------
def build_model():
    """The Cessna 172 and its mission, built and not solved."""
    import test_mission_derivative_jax as cessna

    mission  = cessna.build_mission()
    vehicle  = base_vehicle(mission.segments.climb.analyses.vehicle)
    return vehicle, Data(design=mission)


def define_problem(first_range=None):
    from RCAIDE.Framework.Optimization.Common.Design_Problem import Design_Problem

    problem                           = Design_Problem()
    problem.vehicle, problem.missions = build_model()
    problem.outputs                   = mission_outputs
    problem.update_missions           = climb_to_cruise_altitude

    # [path, initial, lower bound, upper bound, scale]
    aspect_ratio      = float(problem.vehicle.wings.main_wing.aspect_ratio)
    distance          = float(problem.missions.design.segments.cruise.distance)
    altitude          = float(problem.missions.design.segments.cruise.altitude)
    problem.variables = [['missions.design.segments.cruise.distance', distance,     0.5 * distance,     2. * distance,      distance],
                         ['vehicle.wings.main_wing.aspect_ratio',     aspect_ratio, 0.9 * aspect_ratio, 1.1 * aspect_ratio, aspect_ratio],
                         ['missions.design.segments.cruise.altitude', altitude,     0.9 * altitude,     1.1 * altitude,     altitude]]

    # the range of the first design is held
    if first_range is None:
        first_range = range_of_first_design()
    problem.objective   = ['fuel_burn', 1.0]
    problem.constraints = [['range', '>', first_range, 1000.]]
    return problem


def climb_to_cruise_altitude(missions):
    # the climb ends where the cruise flies
    segments                     = missions.design.segments
    segments.climb.altitude_end  = segments.cruise.altitude
    return


def range_of_first_design():
    vehicle, missions = build_model()
    missions.design.evaluate()
    return mission_outputs(missions)['range']


def mission_outputs(missions):
    first = missions.design.segments.climb.conditions
    last  = missions.design.segments.descent.conditions
    # array operations, no float(): a reverse derivative traces this function
    return {'fuel_burn': first.weights.vehicle.mass[0, 0] - last.weights.vehicle.mass[-1, 0],
            'range':     last.frames.inertial.aircraft_range[-1, 0]}


if __name__ == '__main__':
    main()
