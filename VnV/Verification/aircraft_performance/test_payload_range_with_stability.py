# VnV/Verification/aircraft_performance/test_payload_range_with_stability.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Container
from RCAIDE.Library.Methods.Performance.compute_payload_range_diagram import compute_payload_range_diagram

import os
import sys
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'Vehicles'))
from Embraer_190 import vehicle_setup

import payload_range_test as reference

# A payload range diagram asked for before any mission has run, on a vehicle with a stability analysis, must
# compute the neutral point itself. The reference is the same aircraft without the stability analysis: the
# stability analysis does not change the range.

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    check_range_with_stability_analysis()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def payload_range(with_stability):
    vehicle = vehicle_setup()
    for wing in vehicle.wings:
        wing.control_surfaces = Container()      # as in payload_range_test.py, to shorten the run

    configs  = reference.configs_setup(vehicle)
    analyses = reference.fuel_aircraft_analyses_setup(configs)
    if with_stability:
        for analysis in analyses:
            stability                                      = RCAIDE.Framework.Analyses.Stability.Vortex_Lattice_Method()
            stability.settings.number_of_spanwise_vortices  = 10
            stability.settings.number_of_chordwise_vortices = 5
            analysis.append(stability)

    mission  = reference.fuel_aircraft_mission_setup(analyses)
    missions = reference.missions_setup(mission)
    results  = compute_payload_range_diagram(mission=missions.base_mission, fuel_reserve_percentage=0.15)
    return results.range

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_range_with_stability_analysis():
    range_reference = payload_range(with_stability=False)
    range_stability = payload_range(with_stability=True)
    print('Range without and with a stability analysis [m]: ', range_reference[-1], range_stability[-1])
    assert abs(range_stability[-1] / range_reference[-1] - 1) < 1e-3
    return


if __name__ == '__main__':
    main()
    print('test_payload_range_with_stability passed')
