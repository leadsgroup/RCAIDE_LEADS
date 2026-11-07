# VnV/Verification/network_hydrogen/bwb_hydrogen_boil_off_fidelity_test.py
#
#
# Created: Aug 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import Units

# python imports
import numpy as np
import sys
import os

base_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, base_dir)
from bwb_hydrogen_test import (
    BWB_vehicle_setup, BWB_configs_setup, analyses_setup,
)

# ----------------------------------------------------------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    """
    Confirms the BWB hydrogen vehicle flies a sane cruise under both
    Cryogenic_Tank.boil_off_model settings -- 'none' (plain burn-down at engine
    offtake rate only, no boil-off physics -- the fast/simple path) and
    'quasi_steady' (the full two-phase in-flight boil-off IVP). This is a fidelity
    on/off check, not a physics-accuracy check (bwb_hydrogen_test.py's CL_truth and
    verify_powertrain/verify_ground_ops already cover the quasi_steady physics in
    detail): both settings should converge, and 'quasi_steady' should burn strictly
    more total tank mass than 'none' -- boil-off/venting is a pure additional loss
    on top of engine offtake, never a subtraction from it -- by a modest, bounded
    amount, not a negligible rounding difference or a runaway blowup.

    Deliberately cruise-only (mirrors 12_PtX_Boeing/Task_4/Aircraft_Comparisons/
    run_powertrain_fidelity_comparison.py's scope): Ground.Dormancy/Ground.Refuel
    are specific to the quasi_steady model (that's the only path
    Cryogenic_Tank.append_segment_conditions's cross-segment mass/temperature
    chaining and Refuel's terminal-event cutoff logic were built for), so a
    dormancy/refuel leg under 'none' would just show a trivially flat hold and a
    min()-clamped refill -- not a meaningful comparison point.
    """
    results = {}
    for boil_off_model in ('none', 'quasi_steady'):
        vehicle = BWB_vehicle_setup()
        for network in vehicle.networks:
            for source in network.sources:
                if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank):
                    source.boil_off_model = boil_off_model

        configs  = BWB_configs_setup(vehicle)
        analyses = analyses_setup(configs)
        mission  = cruise_only_mission_setup(analyses)
        missions = RCAIDE.Framework.Mission.Missions()
        mission.tag = 'base_mission'
        missions.append(mission)

        mission_results = missions.base_mission.evaluate()
        cruise = mission_results.segments.cruise

        converged = bool(cruise.state.numerics.mission_solver.converged)
        assert converged, f"boil_off_model='{boil_off_model}': cruise failed to converge"

        fuel_burned_kg = total_tank_mass_burned(vehicle, cruise)
        assert np.isfinite(fuel_burned_kg) and fuel_burned_kg > 0, (
            f"boil_off_model='{boil_off_model}': non-physical total fuel burned = {fuel_burned_kg}"
        )
        results[boil_off_model] = fuel_burned_kg
        print(f"  boil_off_model={boil_off_model!r}: converged={converged}  "
              f"total_tank_mass_burned={fuel_burned_kg:.4f} kg")

    none_burn, quasi_steady_burn = results['none'], results['quasi_steady']
    assert quasi_steady_burn > none_burn, (
        f"quasi_steady boil-off should add loss on top of engine offtake, but burned "
        f"{quasi_steady_burn:.4f} kg vs 'none'-model's {none_burn:.4f} kg"
    )
    boil_off_fraction = (quasi_steady_burn - none_burn) / none_burn
    assert 1e-4 < boil_off_fraction < 0.5, (
        f"boil-off's added fuel loss should be a modest, bounded fraction of plain "
        f"engine burn, got {boil_off_fraction:.2%} ({quasi_steady_burn:.4f} kg vs "
        f"{none_burn:.4f} kg)"
    )
    print(f"  boil-off added {boil_off_fraction:.2%} on top of plain engine offtake burn")

    return


def total_tank_mass_burned(vehicle, cruise):
    """Sum of each Cryogenic_Tank's own fuel_mass[0] - fuel_mass[-1] over the cruise
    segment -- works identically for both boil_off_model settings, since both paths
    populate tank_conditions.fuel_mass the same way (explicit integration for 'none',
    the IVP solve for 'quasi_steady')."""
    total = 0.0
    for network in vehicle.networks:
        for source in network.sources:
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank):
                fuel_mass = cruise.conditions.energy.sources[source.tag].fuel_mass[:, 0]
                total += fuel_mass[0] - fuel_mass[-1]
    return total


# ----------------------------------------------------------------------
#   Cruise-Only Mission
# ---------------------------------------------------------------------
def cruise_only_mission_setup(analyses):
    """Same Cruise segment bwb_hydrogen_test.py's mission_setup() flies (same
    shortened-for-sanity 1300 km leg -- see that function's own docstring for why),
    without the Ground.Dormancy/Ground.Refuel legs, since those aren't a meaningful
    comparison point under 'none' (see main()'s docstring)."""
    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments
    base_segment = Segments.Segment()
    base_segment.state.numerics.mission_solver.type = 'root_finder'
    base_segment.state.numerics.mission_solver.max_evaluations = 800

    segment = Segments.Cruise.Constant_Mach_Constant_Altitude(base_segment)
    segment.tag = "Cruise"
    segment.analyses.extend(analyses.cruise)
    segment.altitude    = 40000 * Units['ft']
    segment.mach_number = 0.78
    segment.distance     = 1300 * Units.km

    segment.flight_dynamics.force_x = True
    segment.flight_dynamics.force_z = True

    segment.assigned_control_variables.throttle.active              = True
    segment.assigned_control_variables.throttle.assigned_propulsors = [['propulsor_1', 'propulsor_2']]
    segment.assigned_control_variables.pitch_angle.active            = True

    mission.append_segment(segment)

    return mission


if __name__ == '__main__':
    main()
