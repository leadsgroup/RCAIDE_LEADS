# VnV/Verification/derivatives/test_mission_framework_jax.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
import copy
import numpy as np

from RCAIDE.Framework.Core import Data, array_namespace

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The mission framework (unknown unpacking, frames, kinematics, atmosphere, freestream, forces, moments and the
    # flight dynamics residuals), aerodynamics, stability and the electric rotor network (rotor, motor, ESC, bus,
    # battery) are written once for NumPy and JAX. For every segment of the 787 and tiltrotor missions, this runs the segment's own iterate steps on
    # JAX and checks values, compiled values and derivatives against the NumPy path. Steps not yet written for JAX
    # are held at their NumPy values.
    try:
        import jax
    except ImportError:
        print("JAX is not installed (pip install RCAIDE[ad]); JAX checks skipped")
        return
    jax.config.update("jax_enable_x64", True)
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives

    vnv_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    sys.path.insert(0, os.path.join(vnv_folder, "Vehicles"))
    start_folder = os.getcwd()

    # the imported VnV scripts expect to run from their own folders
    os.chdir(os.path.join(vnv_folder, "Validation", "aircraft_performance"))
    sys.path.insert(0, os.getcwd())
    # the turbofan network and the steps that read it are not written for JAX yet
    check_mission_on_jax("boeing_787", boeing_787_mission(), check_derivatives,
                         frozen_steps=('network', 'energy', 'thrust', 'weights'))

    # every iterate step of the tiltrotor, stopped rotor and tilt wing runs on JAX
    os.chdir(os.path.join(vnv_folder, "Verification", "network_vtol"))
    sys.path.insert(0, os.getcwd())
    check_mission_on_jax("tiltrotor", tiltrotor_mission(), check_derivatives,
                         frozen_steps=())
    check_mission_on_jax("stopped_rotor", stopped_rotor_mission(), check_derivatives,
                         frozen_steps=())
    check_mission_on_jax("tiltwing", tiltwing_mission(), check_derivatives,
                         frozen_steps=())

    os.chdir(start_folder)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_mission_on_jax(name, mission, check_derivatives, frozen_steps, values_only=()):
    import jax
    mission.process.initialize(mission)

    # initialize each segment from the previous one and iterate it once with NumPy, as a mission solve starts
    previous = None
    for tag, segment in mission.segments.items():
        if previous is not None:
            segment.state.initials = previous
        segment.process.initialize.expand_state(segment)
        segment.process.initialize(segment)
        segment.process.iterate(segment)
        previous = segment.state

        residual = Segment_Residual(segment, frozen_steps)
        report   = check_derivatives(residual, (residual.x0,), value_tolerance=1e-12, derivative_tolerance=1e-6)
        # unknowns that change the residual through the steps checked here; with none, the derivative check is empty
        jacobian = np.asarray(jax.jacfwd(residual)(jax.numpy.asarray(residual.x0)))
        reaching = int(np.sum(np.any(jacobian != 0., axis=0)))
        print(f"  {name} {tag}: {reaching} of {residual.x0.size} unknowns reach the residual, values {report.value_error:.1e}, "
              f"compiled {report.compiled_error:.1e}, derivatives {report.jvp_error:.1e}")
        assert report.passed, f"{name} {tag}: JAX check failed, {report}"
        assert (reaching == 0) == (tag in values_only), f"{name} {tag}: {reaching} unknowns reach the residual"
    return

# ----------------------------------------------------------------------------------------------------------------------
#  The residual of one segment as a function of its unknowns
# ----------------------------------------------------------------------------------------------------------------------
class Segment_Residual:
    """residual(x): the segment's iterate steps on a fresh copy of its state, with unknowns x, NumPy or JAX. The
    frozen steps (not yet written for JAX) are skipped, so their outputs stay at the values of the NumPy iterate."""

    def __init__(self, segment, frozen_steps):
        self.segment = segment
        self.state   = segment.state
        self.steps   = iterate_steps(segment.process.iterate, frozen_steps)
        self.x0      = np.concatenate([np.ravel(value) for value in float_leaves(self.state.unknowns)])

    def __call__(self, x):
        xp      = array_namespace(x)
        segment = self.segment

        # fresh copy of the state each call: a JAX evaluation must not write into the NumPy state
        fresh = copy_state(self.state, xp.asarray if xp is not np else np.array)
        set_unknowns(fresh.unknowns, self.state.unknowns, x)

        # segment.conditions is an alias of segment.state.conditions, so both are swapped
        segment.state, segment.conditions = fresh, fresh.conditions
        try:
            for step in self.steps:
                step(segment)
            return xp.concatenate([xp.ravel(value) for value in float_leaves(fresh.residuals)])
        finally:
            segment.state, segment.conditions = self.state, self.state.conditions


def iterate_steps(process, frozen_steps, frozen=False):
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

# ----------------------------------------------------------------------------------------------------------------------
#  Missions
# ----------------------------------------------------------------------------------------------------------------------
def boeing_787_mission():
    import test_Boeing_787_payload_range as b787
    vehicle  = b787.Boeing_787.vehicle_setup()
    analyses = b787.analyses_setup(b787.Boeing_787.configs_setup(vehicle))
    return b787.missions_setup(b787.payload_range_mission_setup(analyses)).base_mission


def tiltrotor_mission():
    import vtol_aircraft_test as vtol
    vehicle  = vtol.TR_vehicle_setup(False)
    analyses = vtol.TR_analyses_setup(vtol.TR_configs_setup(vehicle))
    return vtol.missions_setup(vtol.TR_mission_setup(analyses)).base_mission


def stopped_rotor_mission():
    import vtol_aircraft_test as vtol
    vehicle  = vtol.SR_vehicle_setup(False)
    analyses = vtol.SR_analyses_setup(vtol.SR_configs_setup(vehicle))
    return vtol.missions_setup(vtol.SR_mission_setup(analyses, vehicle)).base_mission


def tiltwing_mission():
    import vtol_aircraft_test as vtol
    vehicle  = vtol.TW_vehicle_setup(False)
    analyses = vtol.TW_analyses_setup(vtol.TW_configs_setup(vehicle))
    return vtol.missions_setup(vtol.TW_mission_setup(analyses)).base_mission


if __name__ == '__main__':
    main()
