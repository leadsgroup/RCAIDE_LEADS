# VnV/Verification/derivatives/test_mission_framework_jax.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
import numpy as np

from RCAIDE.Framework.Core import array_namespace
from RCAIDE.Library.Mission.Solver.compiled_residual import compile_segment, residual_of_inputs, segment_inputs, read_input, unknowns_vector

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The mission framework (unknown unpacking, frames, kinematics, atmosphere, freestream, forces, moments and the
    # flight dynamics residuals), aerodynamics, stability, the fuel network with the off-design turbofan and the
    # electric rotor network (rotor, motor, ESC, bus, battery) and every other powertrain are written once for NumPy
    # and JAX. For each distinct kind of segment across the missions below (segment type, propulsors, flight regime;
    # see segment_signature), this runs the segment's own iterate steps on JAX and checks compiled values and forward
    # and reverse derivatives against the NumPy path, with respect to the unknowns and to every float input the
    # residual reads (vehicle, surrogate tables, engine design, initials). It also checks that no input is frozen into
    # the compiled residual, so one compiled residual serves every design with the same structure.
    # RCAIDE_JAX_EVERY_SEGMENT=1 checks every segment instead.
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
    check_mission_on_jax("boeing_787", boeing_787_mission(), check_derivatives, value_tolerance=1e-6)

    # every iterate step of the tiltrotor, stopped rotor and tilt wing runs on JAX
    os.chdir(os.path.join(vnv_folder, "Verification", "network_vtol"))
    sys.path.insert(0, os.getcwd())
    check_mission_on_jax("tiltrotor", tiltrotor_mission(), check_derivatives)
    check_mission_on_jax("stopped_rotor", stopped_rotor_mission(), check_derivatives)
    check_mission_on_jax("tiltwing", tiltwing_mission(), check_derivatives)

    # every other powertrain: turbojet (numba matching stops at a 1e-8 change, as the turbofan), turboprop, piston
    # engines, electric ducted fan, fuel cells, and series, parallel and all-electric hybrid networks
    for name, folder, build, value_tolerance in powertrain_missions():
        os.chdir(os.path.join(vnv_folder, "Verification", folder))
        sys.path.insert(0, os.getcwd())
        check_mission_on_jax(name, build(), check_derivatives, value_tolerance=value_tolerance)

    os.chdir(start_folder)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_mission_on_jax(name, mission, check_derivatives, value_tolerance=1e-12):
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

        # values and derivatives with respect to the unknowns, against NumPy
        compiled, inputs = compile_segment(segment)
        x0       = unknowns_vector(segment.state.unknowns)
        def residual(x):
            xp = array_namespace(x)
            return residual_of_inputs(segment, x, compiled.paths, [xp.asarray(value) for value in inputs])
        report   = check_derivatives(residual, (x0,), value_tolerance=value_tolerance, derivative_tolerance=1e-6)
        reaching = int(np.count_nonzero(report.input_sensitivity[0]))

        # the compiled residual shared by every design with this structure: no input frozen in, and its derivative
        # with respect to the inputs
        frozen   = check_no_frozen_input(segment, compiled, x0)
        along    = check_input_derivative(compiled, x0, inputs)
        print(f"  {name} {tag} {signature}: {reaching} of {x0.size} unknowns and {len(compiled.paths)} inputs reach the residual, "
              f"values {report.value_error:.1e}, derivatives {report.jvp_error:.1e}, reverse {report.vjp_consistency_error:.1e}, "
              f"every input changed {frozen:.1e}, input derivative {along:.1e}")
        assert report.passed, f"{name} {tag}: JAX check failed, {report}"
        assert reaching > 0, f"{name} {tag}: no unknown reaches the residual"
        assert frozen <= max(value_tolerance, 1e-10), f"{name} {tag}: an input is frozen into the compiled residual, {frozen}"
        assert along <= 1e-6, f"{name} {tag}: derivative with respect to the inputs, {along}"
    return


def check_no_frozen_input(segment, compiled, x0):
    """
    Changes every float input of the segment by about 1e-4 and compares NumPy with the compiled residual, which is
    given only the inputs its trace reads. An input that NumPy reads but the compiled residual holds as a constant
    shows as a difference. Also checks that asking for the compiled residual again does not compile again.
    """
    references = segment_inputs(segment)[0]
    rng        = np.random.default_rng(0)
    paths      = [reference[3] for reference in references]
    values     = [np.asarray(read_input(reference), dtype=float) * (1 + 1e-4 * rng.uniform(0.5, 1.0)) for reference in references]
    expected   = residual_of_inputs(segment, x0, paths, values)

    by_path    = dict(zip(paths, values))
    result     = np.asarray(compiled.residual(x0, tuple(by_path[path] for path in compiled.paths)))

    again, inputs = compile_segment(segment)
    traces        = compiled.traces
    compiled.residual(x0, inputs)
    assert again is compiled and compiled.traces == traces, "the compiled residual was compiled again for the same segment"
    return float(np.max(np.abs(result - expected)) / max(np.max(np.abs(expected)), 1e-300))


def check_input_derivative(compiled, x0, inputs):
    """
    The compiled derivative of the residual along a random relative change of every input it reads, against central
    differences of the compiled residual. The inputs are first moved by about 1e-3, off the knots of the
    piecewise-linear tables some of them sit on (a flap deflection at a tabulated angle).
    """
    rng       = np.random.default_rng(1)
    base      = tuple(value * (1 + 1e-3 * rng.uniform(0.5, 1.0, value.shape)) for value in inputs)
    direction = tuple(value * rng.standard_normal(value.shape) for value in base)
    exact     = np.asarray(compiled.input_derivative(x0, base, direction))
    errors    = []
    for h in (1e-4, 1e-5, 1e-6):
        plus   = np.asarray(compiled.residual(x0, tuple(value + h * change for value, change in zip(base, direction))))
        minus  = np.asarray(compiled.residual(x0, tuple(value - h * change for value, change in zip(base, direction))))
        errors.append(float(np.max(np.abs(exact - (plus - minus) / (2 * h))) / max(np.max(np.abs(exact)), 1e-300)))
    return min(errors)


def powertrain_component_types(vehicle):
    """Class names of every powertrain component on the vehicle's networks, including battery modules and their heat
    acquisition systems."""
    names = set()
    for network in vehicle.networks:
        for group in ('propulsors', 'converters', 'sources', 'distributors'):
            for component in getattr(network, group, {}).values() if hasattr(getattr(network, group, None), 'values') else []:
                names.add(type(component).__name__)
                for module in getattr(component, 'modules', {}).values() if hasattr(getattr(component, 'modules', None), 'values') else []:
                    names.add(type(module).__name__)
                    if getattr(module, 'heat_acquisition_system', None) is not None:
                        names.add(type(module.heat_acquisition_system).__name__)
    return names


# kinds of segment already checked in this run (see segment_signature)
CHECKED_SIGNATURES = set()

def segment_signature(segment):
    """
    What distinguishes one segment's code path from another's: its type, the powertrain component types it flies on
    (propulsors, converters, sources, and the battery modules and their heat acquisition systems), and its flight
    regime (subsonic, transonic or supersonic, which switch drag and engine models), read from the NumPy iterate.
    Segments with the same signature run the same code, so one of them is checked.
    """
    propulsors = sorted(powertrain_component_types(segment.analyses.vehicle))
    mach       = float(np.max(segment.state.conditions.freestream.mach_number))
    regime     = 'subsonic' if mach < 0.7 else ('transonic' if mach < 1.0 else 'supersonic')
    return (type(segment).__name__, tuple(propulsors), regime)

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
