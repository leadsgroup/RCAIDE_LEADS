# RCAIDE/Library/Mission/Solver/mission_derivative.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Mission.Solver.compiled_residual import (compile_segment, unknowns_vector, set_unknowns, copy_state,
                                                             state_arrays, update_position, signature_difference)

# ----------------------------------------------------------------------------------------------------------------------
#  mission_derivative
# ----------------------------------------------------------------------------------------------------------------------
def mission_derivative(mission, moved_mission, step, opposite_mission=None):
    """
    Derivative of every segment's converged state along a design change, without solving the mission again.

    The converged unknowns x of a segment satisfy R(x, inputs) = 0, so along a change of the inputs
    (dR/dx) dx = -(dR/dinputs) d_inputs, with both derivatives from the segment's compiled residual. The inputs of a
    segment change with the design and with the end state of the segment before it, so the segments are taken in
    order and each one's state derivative is passed on as the next one's initials.

    The change of the inputs with the design is a finite difference of the setup (geometry, weights, surrogates): a
    forward difference to the moved mission, or a central difference when the opposite mission is given. The surrogate
    tables carry round-off of about 1e-8, so a relative step near 1e-3 with a central difference is accurate where a
    very small step is not; settings that do not pass through the setup (a segment distance) are exact at any step.

    Inputs:
       mission            solved mission (mission.evaluate() has run)
       moved_mission      the same mission built at the design moved by +step along the change, not solved
       step               size of the move                                                              [design units]
       opposite_mission   optional, the same mission built at the design moved by -step, not solved

    Outputs:
       derivatives     one Data per segment, per unit of step:
                          tag        segment tag
                          unknowns   derivative of the converged unknowns
                          state      {state path: derivative of that state array}

    Assumptions:
       The moved missions have the same structure as the mission (same segments, control points and unknowns).
    """
    # the setup at the moved designs: geometry, weights, surrogates and energy network
    sides = [(moved_mission, +step)] + ([(opposite_mission, -step)] if opposite_mission is not None else [])
    for side, move in sides:
        side.process.initialize(side)
        assert list(side.segments.keys()) == list(mission.segments.keys()), "a moved mission has different segments"

    derivatives = []
    for index, segment in enumerate(mission.segments.values()):
        x = unknowns_vector(segment.state.unknowns)

        # the inputs of each moved segment at the converged unknowns, started from the moved end state of the segment
        # before it
        moved_inputs = []
        for side, move in sides:
            moved = list(side.segments.values())[index]
            if index > 0:
                moved.state.initials = moved_state(previous.state, derivatives[-1].state, move)
            expand_state = moved.process.initialize.expand_state
            expand_state(moved)
            moved.process.initialize.expand_state = RCAIDE.Library.Methods.skip
            moved.process.initialize(moved)
            moved.process.initialize.expand_state = expand_state
            set_unknowns(moved.state.unknowns, moved.state.unknowns, x)
            moved.process.iterate(moved)
            update_position(moved)
            moved_inputs.append((moved,) + compile_segment(moved))

        compiled, inputs = compile_segment(segment)
        for moved, compiled_moved, values in moved_inputs:
            if compiled_moved is not compiled:
                raise ValueError(f"{segment.tag}: a moved mission has a different structure ({signature_difference(segment, moved)})")
        if opposite_mission is None:
            direction = tuple((value - original) / step for value, original in zip(moved_inputs[0][2], inputs))
        else:
            direction = tuple((value - opposite) / (2 * step) for value, opposite in zip(moved_inputs[0][2], moved_inputs[1][2]))

        # change of the unknowns that keeps the residual at zero
        jacobian     = np.asarray(compiled.jacobian(x, inputs))
        forcing      = np.asarray(compiled.input_derivative(x, inputs, direction))
        for name, array in (('Jacobian', jacobian), ('derivative along the inputs', forcing)):
            if not np.all(np.isfinite(array)):
                changed = [path for path, change in zip(compiled.paths, direction) if np.any(change != 0)]
                raise ValueError(f"{segment.tag}: the {name} of the residual is not finite "
                                 f"({int(np.sum(~np.isfinite(array)))} of {array.size} entries; inputs that change: {changed[:8]})")
        # least squares: unknowns that do not reach the residual get no change
        d_unknowns   = np.linalg.lstsq(jacobian, -forcing, rcond=None)[0]
        d_state      = [np.asarray(array) for array in compiled.state_derivative(x, inputs, d_unknowns, direction)]
        derivatives.append(Data(tag=segment.tag, unknowns=d_unknowns, state=dict(zip(compiled.state_paths, d_state))))
        previous = segment
    return derivatives


def moved_state(state, derivative, move):
    """A copy of a converged state moved along its derivative: the end state a moved segment starts from."""
    moved = copy_state(state, np.array)
    for container, key, path in state_arrays(moved):
        if path in derivative:
            container[key] = container[key] + move * derivative[path]
    return moved
