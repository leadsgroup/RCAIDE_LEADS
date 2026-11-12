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
    # flight dynamics residuals), aerodynamics, stability, the fuel network with the off-design turbofan and the
    # electric rotor network (rotor, motor, ESC, bus, battery) and every other powertrain are written once for NumPy
    # and JAX. For each distinct kind of segment across the missions below (segment type, propulsors, flight regime;
    # see segment_signature), this runs the segment's own iterate steps on JAX and checks compiled values and forward
    # and reverse derivatives against the NumPy path. RCAIDE_JAX_EVERY_SEGMENT=1 checks every segment instead.
    try:
        import jax
    except ImportError:
        print("JAX is not installed (pip install RCAIDE[ad]); JAX checks skipped")
        return
    jax.config.update("jax_enable_x64", True)
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives
    CHECKED_SIGNATURES.clear()

    vnv_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    sys.path.insert(0, os.path.join(vnv_folder, "Vehicles"))
    start_folder = os.getcwd()

    # the imported VnV scripts expect to run from their own folders
    os.chdir(os.path.join(vnv_folder, "Validation", "aircraft_performance"))
    sys.path.insert(0, os.getcwd())
    # every iterate step of the 787 runs on JAX; the NumPy engine matching stops at a 1e-8 change and the JAX one
    # converges fully (its implicit derivative needs the matched state), so values agree to about 1e-7
    check_mission_on_jax("boeing_787", boeing_787_mission(), check_derivatives,
                         frozen_steps=(), value_tolerance=1e-6)

    # every iterate step of the tiltrotor, stopped rotor and tilt wing runs on JAX
    os.chdir(os.path.join(vnv_folder, "Verification", "network_vtol"))
    sys.path.insert(0, os.getcwd())
    check_mission_on_jax("tiltrotor", tiltrotor_mission(), check_derivatives,
                         frozen_steps=())
    check_mission_on_jax("stopped_rotor", stopped_rotor_mission(), check_derivatives,
                         frozen_steps=())
    check_mission_on_jax("tiltwing", tiltwing_mission(), check_derivatives,
                         frozen_steps=())

    # every other powertrain: turbojet (numba matching stops at a 1e-8 change, as the turbofan), turboprop, piston
    # engines, electric ducted fan, fuel cells, and series, parallel and all-electric hybrid networks
    for name, folder, build, value_tolerance in powertrain_missions():
        os.chdir(os.path.join(vnv_folder, "Verification", folder))
        sys.path.insert(0, os.getcwd())
        check_mission_on_jax(name, build(), check_derivatives, frozen_steps=(), value_tolerance=value_tolerance)

    os.chdir(start_folder)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_mission_on_jax(name, mission, check_derivatives, frozen_steps, values_only=(), value_tolerance=1e-12):
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

        # each distinct kind of segment is checked once, unless every segment is asked for
        signature = segment_signature(segment)
        if signature in CHECKED_SIGNATURES and not os.environ.get("RCAIDE_JAX_EVERY_SEGMENT"):
            continue
        CHECKED_SIGNATURES.add(signature)

        residual = Segment_Residual(segment, frozen_steps)
        report   = check_derivatives(residual, (residual.x0,), value_tolerance=value_tolerance, derivative_tolerance=1e-6)
        # unknowns that change the residual through the steps checked here; with none, the derivative check is empty
        reaching = int(np.count_nonzero(report.input_sensitivity[0]))
        print(f"  {name} {tag} {signature}: {reaching} of {residual.x0.size} unknowns reach the residual, "
              f"values {report.value_error:.1e}, derivatives {report.jvp_error:.1e}, reverse {report.vjp_consistency_error:.1e}")
        assert report.passed, f"{name} {tag}: JAX check failed, {report}"
        assert (reaching == 0) == (tag in values_only), f"{name} {tag}: {reaching} unknowns reach the residual"
    return


# kinds of segment already checked in this run (see segment_signature)
CHECKED_SIGNATURES = set()

def segment_signature(segment):
    """
    What distinguishes one segment's code path from another's: its type, the propulsor types it flies on, and its
    flight regime (subsonic, transonic or supersonic, which switch drag and engine models), read from the NumPy
    iterate. Segments with the same signature run the same code, so one of them is checked.
    """
    propulsors = sorted({type(propulsor).__name__ for network in segment.analyses.vehicle.networks
                         for propulsor in network.propulsors})
    mach       = float(np.max(segment.state.conditions.freestream.mach_number))
    regime     = 'subsonic' if mach < 0.7 else ('transonic' if mach < 1.0 else 'supersonic')
    return (type(segment).__name__, tuple(propulsors), regime)

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


def powertrain_missions():
    """(name, VnV folder, mission builder, value tolerance) for the powertrains beyond the 787 and the VTOLs."""
    return [("concorde",              "network_turbojet",                   concorde_mission,           1e-6),
            ("atr_72",                "network_turboprop",                  atr_72_mission,             1e-12),
            ("cessna_172",            "network_internal_combustion_engine", cessna_mission,             1e-12),
            ("cessna_constant_speed", "network_internal_combustion_engine", constant_speed_mission,     1e-12),
            ("x_48",                  "network_electric",                   x_48_mission,               1e-12),
            ("fuel_cell_pem",         "network_fuel_cell",                  lambda: fuel_cell_mission('PEM'),      1e-12),
            ("fuel_cell_larminie",    "network_fuel_cell",                  lambda: fuel_cell_mission('Larminie'), 1e-12),
            ("atr_72_series_hybrid",  "network_hybrid",                     lambda: hybrid_mission('series_hybrid'),   1e-12),
            ("atr_72_parallel_hybrid","network_hybrid",                     lambda: hybrid_mission('parallel_hybrid'), 1e-12),
            ("atr_72_all_electric",   "network_hybrid",                     lambda: hybrid_mission('all_electric'),    1e-12)]


def vnv_mission(test, vehicle):
    """The base mission of a VnV test script, for a vehicle it set up."""
    analyses = test.analyses_setup(test.configs_setup(vehicle))
    return test.missions_setup(test.mission_setup(analyses)).base_mission


def concorde_mission():
    import turbojet_network_test as test
    return vnv_mission(test, test.vehicle_setup())


def atr_72_mission():
    import turboprop_network_test as test
    return vnv_mission(test, test.vehicle_setup())


def cessna_mission():
    import ICE_test as test
    return vnv_mission(test, test.vehicle_setup())


def constant_speed_mission():
    import ICE_constant_speed_test as test
    vehicle  = test.ICE_CS(test.vehicle_setup())
    analyses = test.analyses_setup(test.configs_setup(vehicle))
    return test.missions_setup(analyses).base_mission


def x_48_mission():
    import electric_ducted_fan_aircraft_test as test
    return vnv_mission(test, test.vehicle_setup(True, 'Rankine_Froude_Momentum_Theory'))


def fuel_cell_mission(model):
    import hydrogen_fuel_cell_aircraft_test as test
    return vnv_mission(test, test.vehicle_setup(model))


def hybrid_mission(kind):
    import hybrid_network_test as test
    vehicle  = getattr(test, kind + '_vehicle_setup')()
    analyses = test.analyses_setup(getattr(test, kind + '_configs_setup')(vehicle), weights_method='electric')
    return test.missions_setup(analyses, 'optimize', 'energy').base_mission


if __name__ == '__main__':
    main()
