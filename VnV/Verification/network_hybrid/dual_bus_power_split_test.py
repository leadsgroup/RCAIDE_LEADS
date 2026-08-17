# dual_bus_power_split_test.py
#
# Created: 2026, network robustness regression
#
# Regression guard for per-distributor phi/psi resolution. Uses a vehicle
# (series_hybrid_electric_ATR_72_dual_bus) with two electrically-isolated
# buses:
#
#   - propulsive_bus: providers = 2x turboelectric_generator ONLY (no battery)
#   - essential_bus:  provider  = a small dedicated battery ONLY, consumer = avionics
#
# segment.hybrid_power_split_ratio / battery_fuel_cell_power_split_ratio are
# deliberately left unset so the network's auto-resolution logic
# (RCAIDE.Library.Mission.Common.Pre_Process.energy.resolve_hybridization)
# actually runs. Each bus is individually unambiguous, but a vehicle-wide
# (rather than per-distributor) existence check sees a battery AND generators
# somewhere in the network and can wrongly resolve one shared, ambiguous psi
# for the whole vehicle -- which would silently zero out the generators'
# dispatch on the propulsive bus (it has no battery to fall back on) even
# though the mission may still numerically converge on battery power alone,
# or fail to converge outright once the essential battery's tiny capacity is
# exhausted. This test asserts each bus behaves correctly on its own,
# not just that the mission runs.

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units, Data

import numpy as np
import sys
import os
import time

base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from series_hybrid_electric_ATR_72_dual_bus import vehicle_setup as dual_bus_vehicle_setup
from series_hybrid_electric_ATR_72_dual_bus import configs_setup as dual_bus_configs_setup

# ----------------------------------------------------------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    ti = time.time()

    vehicle  = dual_bus_vehicle_setup()
    configs  = dual_bus_configs_setup(vehicle)
    analyses = analyses_setup(configs)
    mission  = mission_setup(analyses)
    missions = missions_setup(mission)

    results = missions.base_mission.evaluate()

    cruise_conditions = results.segments.cruise.conditions

    # Propulsive bus: generator-only, should dispatch real power (see header).
    gen_1_power = np.linalg.norm(cruise_conditions.energy.converters['turboelectric_generator'].outputs.power.electrical)
    gen_2_power = np.linalg.norm(cruise_conditions.energy.converters['turboelectric_generator_2'].outputs.power.electrical)

    print('Generator 1 electrical power (W):', gen_1_power)
    print('Generator 2 electrical power (W):', gen_2_power)

    assert gen_1_power > 1e3, (
        f"turboelectric_generator dispatched ~0 electrical power ({gen_1_power:.3e} W) -- "
        f"psi was likely resolved as an ambiguous vehicle-wide value instead of "
        f"per-distributor, incorrectly treating the generator-only propulsive bus "
        f"as if it had a battery to fall back on."
    )
    assert gen_2_power > 1e3, (
        f"turboelectric_generator_2 dispatched ~0 electrical power ({gen_2_power:.3e} W) -- "
        f"same per-distributor psi resolution issue as turboelectric_generator."
    )

    # Essential bus: battery-only, should dispatch and drop SOC (see header).
    essential_battery_conditions = cruise_conditions.energy.sources['essential_battery_pack']
    soc_initial = essential_battery_conditions.state_of_charge[0, 0]
    soc_final   = essential_battery_conditions.state_of_charge[-1, 0]

    print('Essential battery SOC: initial =', soc_initial, ' final =', soc_final)

    assert soc_final < soc_initial, (
        f"essential_battery_pack state of charge did not drop during cruise "
        f"(initial={soc_initial}, final={soc_final}) -- the essential bus's battery "
        f"was not dispatched, meaning avionics power was not actually drawn from it."
    )
    assert 0.0 <= soc_final <= 1.0, (
        f"essential_battery_pack state of charge left the physical [0, 1] range "
        f"(final={soc_final}) -- it is reading a demand that does not belong to "
        f"essential_bus, most likely the propulsive_bus's much larger demand."
    )
    assert soc_initial - soc_final < 0.05, (
        f"essential_battery_pack dropped {soc_initial - soc_final:.4f} in state of "
        f"charge for a single small 30 W avionics load over one cruise segment -- "
        f"that is far too much and points to the same cross-bus contamination bug."
    )

    elapsed_time = time.time() - ti
    elapsed_time_min = elapsed_time / 60
    print('Elapsed time (min): ', elapsed_time_min)
    return


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------
def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    for tag, config in list(configs.items()):
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses


def base_analysis(vehicle):

    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle

    weights = RCAIDE.Framework.Analyses.Weights.Electric_General_Aviation()
    analyses.append(weights)

    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    analyses.append(geometry)

    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.number_of_spanwise_vortices    = 10
    aerodynamics.settings.number_of_chordwise_vortices   = 5
    analyses.append(aerodynamics)

    energy = RCAIDE.Framework.Analyses.Energy.Energy()
    analyses.append(energy)

    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)

    return analyses


# ----------------------------------------------------------------------
#   Define the Mission
# ----------------------------------------------------------------------
def mission_setup(analyses):

    mission = RCAIDE.Framework.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    Segments = RCAIDE.Framework.Mission.Segments
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_of_control_points = 5

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"
    segment.analyses.extend(analyses.base)
    segment.altitude                                   = 25000  * Units.feet
    segment.air_speed                                  = 270    * Units.kts
    segment.distance                                    = 100.  * Units.nautical_mile
    segment.initial_battery_conditions.state_of_charge = 1.0

    # hybrid_power_split_ratio / battery_fuel_cell_power_split_ratio are
    # deliberately left unset (None) -- the whole point of this test is to
    # exercise the network's auto-resolution logic per bus.

    segment.flight_dynamics.force_x                                  = True
    segment.flight_dynamics.force_z                                  = True

    segment.assigned_control_variables.throttle.active                  = True
    segment.assigned_control_variables.throttle.assigned_propulsors     = [['starboard_propulsor','port_propulsor']]
    segment.assigned_control_variables.throttle.initial_guess_values    = [[0.7]]
    segment.assigned_control_variables.pitch_angle.active                = True

    mission.append_segment(segment)

    return mission


def missions_setup(mission):

    missions     = RCAIDE.Framework.Mission.Missions()
    mission.tag  = 'base_mission'
    missions.append(mission)

    return missions


if __name__ == '__main__':
    main()
