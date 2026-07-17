# V_n_diagram_test.py
#
# Created: Dec 2024, M Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core              import Data
from RCAIDE.Library.Methods.Performance import generate_V_n_diagram

import numpy as np
import sys
import os
import time

base_dir      = os.path.dirname(os.path.abspath(__file__))
vehicles_path = os.path.abspath(os.path.join(base_dir, "..", "..", "Vehicles"))
if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)

from Cessna_172 import vehicle_setup as GA_vehicle_setup
from Cessna_172 import configs_setup  as GA_configs_setup
from Boeing_737 import vehicle_setup as Transport_vehicle_setup
from Boeing_737 import configs_setup  as Transport_configs_setup

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    ti = time.time()

    test_part_23_normal()
    test_part_23_utility()
    test_part_23_acrobatic()
    test_part_25()

    elapsed_time = time.time() - ti
    print(f'\nElapsed time (min): {elapsed_time / 60:.4f}')
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def GA_analyses_setup(configs):
    analyses = RCAIDE.Framework.Analyses.Analysis.Container()
    for tag, config in configs.items():
        analyses[tag] = GA_base_analysis(config)
    return analyses

def GA_base_analysis(vehicle):
    analyses            = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle    = vehicle
    analyses.append(RCAIDE.Framework.Analyses.Geometry.Geometry())
    weights             = RCAIDE.Framework.Analyses.Weights.Conventional_General_Aviation()
    weights.method      = 'Raymer'
    analyses.append(weights)
    aero = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aero.settings.number_of_spanwise_vortices  = 10
    aero.settings.number_of_chordwise_vortices = 5
    analyses.append(aero)
    analyses.append(RCAIDE.Framework.Analyses.Energy.Energy())
    analyses.append(RCAIDE.Framework.Analyses.Planets.Earth())
    analyses.append(RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976())
    return analyses

def Transport_analyses_setup(configs):
    analyses = RCAIDE.Framework.Analyses.Analysis.Container()
    for tag, config in configs.items():
        analyses[tag] = TR_base_analysis(config)
    return analyses

def TR_base_analysis(vehicle):
    analyses         = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle
    analyses.append(RCAIDE.Framework.Analyses.Geometry.Geometry())
    analyses.append(RCAIDE.Framework.Analyses.Weights.Conventional_Transport())
    aero = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aero.settings.number_of_spanwise_vortices  = 10
    aero.settings.number_of_chordwise_vortices = 5
    analyses.append(aero)
    analyses.append(RCAIDE.Framework.Analyses.Energy.Energy())
    analyses.append(RCAIDE.Framework.Analyses.Planets.Earth())
    analyses.append(RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976())
    return analyses

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_part_23_normal():
    """FAR Part 23 normal category — positive gust at Va exceeds structural limit,
    triggering the Vb intersection branch and extended stall line."""
    print('\n--- FAR Part 23 normal category (Cessna 172) ---')
    vehicle  = GA_vehicle_setup()
    configs  = GA_configs_setup(vehicle)
    analyses = GA_analyses_setup(configs)

    V_n_data = generate_V_n_diagram(analyses=analyses.cruise)

    truth = Data()
    truth.Vs1_pos             = 37.98585717834934
    truth.Vs1_neg             = 53.720114399989036
    truth.Va_pos              = 74.04806758573127
    truth.Va_neg              = 66.23060508967755
    truth.Vc                  = 126.33084642567567
    truth.Vd                  = 176.86318499594594
    truth.limit_load_pos      = 3.9899834399932685
    truth.limit_load_neg      = -1.9899834399932685
    truth.dive_limit_load_pos = 3.8
    truth.dive_limit_load_neg = -1.0929884079952883

    error = Data()
    error.Vs1_pos             = (truth.Vs1_pos             - V_n_data.Vs1.positive)            / truth.Vs1_pos
    error.Vs1_neg             = (truth.Vs1_neg             - V_n_data.Vs1.negative)            / truth.Vs1_neg
    error.Va_pos              = (truth.Va_pos              - V_n_data.Va.positive)             / truth.Va_pos
    error.Va_neg              = (truth.Va_neg              - V_n_data.Va.negative)             / truth.Va_neg
    error.Vc                  = (truth.Vc                  - V_n_data.Vc)                      / truth.Vc
    error.Vd                  = (truth.Vd                  - V_n_data.Vd)                      / truth.Vd
    error.limit_load_pos      = (truth.limit_load_pos      - V_n_data.positive_limit_load)     / truth.limit_load_pos
    error.limit_load_neg      = (truth.limit_load_neg      - V_n_data.negative_limit_load)     / truth.limit_load_neg
    error.dive_limit_load_pos = (truth.dive_limit_load_pos - V_n_data.limit_loads.dive.positive) / truth.dive_limit_load_pos
    error.dive_limit_load_neg = (truth.dive_limit_load_neg - V_n_data.limit_loads.dive.negative)

    for k, v in error.items():
        assert np.abs(v) < 1e-6, f'Part 23 normal: {k} error {v:.2e} exceeds tolerance'

    print('  PASSED')


def test_part_23_utility():
    """FAR Part 23 utility category — positive limit enforced to minimum 4.4 g,
    negative limit = -0.4 * positive. Dive speed scaled by 1.5."""
    print('\n--- FAR Part 23 utility category ---')
    vehicle = GA_vehicle_setup()
    vehicle.flight_envelope.category           = 'utility'
    vehicle.flight_envelope.positive_limit_load = 3.8   # below minimum → clamped to 4.4
    vehicle.flight_envelope.negative_limit_load = -1.5
    configs  = GA_configs_setup(vehicle)
    analyses = GA_analyses_setup(configs)

    V_n_data = generate_V_n_diagram(analyses=analyses.cruise)

    truth = Data()
    truth.Vs1_pos             = 37.98585717834934
    truth.Vs1_neg             = 53.720114399989036
    truth.Va_pos              = 79.67980622796094
    truth.Va_neg              = 71.26778526389269
    truth.Vc                  = 126.33084642567567
    truth.Vd                  = 189.49626963851352
    truth.limit_load_pos      = 4.4
    truth.limit_load_neg      = -1.9899834399932685
    truth.dive_limit_load_pos = 4.4
    truth.dive_limit_load_neg = -1.2424875799949517

    error = Data()
    error.Vs1_pos             = (truth.Vs1_pos             - V_n_data.Vs1.positive)              / truth.Vs1_pos
    error.Vs1_neg             = (truth.Vs1_neg             - V_n_data.Vs1.negative)              / truth.Vs1_neg
    error.Va_pos              = (truth.Va_pos              - V_n_data.Va.positive)               / truth.Va_pos
    error.Va_neg              = (truth.Va_neg              - V_n_data.Va.negative)               / truth.Va_neg
    error.Vc                  = (truth.Vc                  - V_n_data.Vc)                        / truth.Vc
    error.Vd                  = (truth.Vd                  - V_n_data.Vd)                        / truth.Vd
    error.limit_load_pos      = (truth.limit_load_pos      - V_n_data.positive_limit_load)       / truth.limit_load_pos
    error.limit_load_neg      = (truth.limit_load_neg      - V_n_data.negative_limit_load)       / truth.limit_load_neg
    error.dive_limit_load_pos = (truth.dive_limit_load_pos - V_n_data.limit_loads.dive.positive) / truth.dive_limit_load_pos
    error.dive_limit_load_neg = (truth.dive_limit_load_neg - V_n_data.limit_loads.dive.negative) / truth.dive_limit_load_neg

    for k, v in error.items():
        assert np.abs(v) < 1e-6, f'Part 23 utility: {k} error {v:.2e} exceeds tolerance'

    print('  PASSED')


def test_part_23_acrobatic():
    """FAR Part 23 acrobatic category — positive limit enforced to minimum 6.0 g,
    negative limit = -0.5 * positive. Special gust intersection formula applies."""
    print('\n--- FAR Part 23 acrobatic category ---')
    vehicle = GA_vehicle_setup()
    vehicle.flight_envelope.category           = 'acrobatic'
    vehicle.flight_envelope.positive_limit_load = 3.8   # below minimum → clamped to 6.0
    vehicle.flight_envelope.negative_limit_load = -1.5
    configs  = GA_configs_setup(vehicle)
    analyses = GA_analyses_setup(configs)

    V_n_data = generate_V_n_diagram(analyses=analyses.cruise)

    truth = Data()
    truth.Vs1_pos             = 37.98585717834934
    truth.Vs1_neg             = 53.720114399989036
    truth.Va_pos              = 93.04596752919348
    truth.Va_neg              = 93.04596752919348
    truth.Vc                  = 137.81546882800984
    truth.Vd                  = 213.61397668341527
    truth.limit_load_pos      = 6.0
    truth.limit_load_neg      = -3.0
    truth.dive_limit_load_pos = 6.0
    truth.dive_limit_load_neg = -1.527895090176128

    error = Data()
    error.Vs1_pos             = (truth.Vs1_pos             - V_n_data.Vs1.positive)              / truth.Vs1_pos
    error.Vs1_neg             = (truth.Vs1_neg             - V_n_data.Vs1.negative)              / truth.Vs1_neg
    error.Va_pos              = (truth.Va_pos              - V_n_data.Va.positive)               / truth.Va_pos
    error.Va_neg              = (truth.Va_neg              - V_n_data.Va.negative)               / truth.Va_neg
    error.Vc                  = (truth.Vc                  - V_n_data.Vc)                        / truth.Vc
    error.Vd                  = (truth.Vd                  - V_n_data.Vd)                        / truth.Vd
    error.limit_load_pos      = (truth.limit_load_pos      - V_n_data.positive_limit_load)       / truth.limit_load_pos
    error.limit_load_neg      = (truth.limit_load_neg      - V_n_data.negative_limit_load)       / truth.limit_load_neg
    error.dive_limit_load_pos = (truth.dive_limit_load_pos - V_n_data.limit_loads.dive.positive) / truth.dive_limit_load_pos
    error.dive_limit_load_neg = (truth.dive_limit_load_neg - V_n_data.limit_loads.dive.negative) / truth.dive_limit_load_neg

    for k, v in error.items():
        assert np.abs(v) < 1e-6, f'Part 23 acrobatic: {k} error {v:.2e} exceeds tolerance'

    print('  PASSED')


def test_part_25():
    """FAR Part 25 transport category (Boeing 737) — load limits follow
    the 2.1 + 24000/(W+10000) formula, capped at 3.8."""
    print('\n--- FAR Part 25 transport category (Boeing 737) ---')
    vehicle  = Transport_vehicle_setup()
    configs  = Transport_configs_setup(vehicle)
    analyses = Transport_analyses_setup(configs)

    V_n_data = generate_V_n_diagram(analyses=analyses.cruise)

    truth = Data()
    truth.Vs1_pos             = 113.20236642595314
    truth.Vs1_neg             = 160.09232189231165
    truth.Va_pos              = 178.9886572134933
    truth.Va_neg              = 196.0722501867801
    truth.Vc                  = 515.9537531823821
    truth.Vd                  = 644.9421914779776
    truth.limit_load_pos      = 3.538568908482968
    truth.limit_load_neg      = -1.5385689084829681
    truth.dive_limit_load_pos = 2.586605567801855
    truth.dive_limit_load_neg = -0.5866055678018549

    error = Data()
    error.Vs1_pos             = (truth.Vs1_pos             - V_n_data.Vs1.positive)              / truth.Vs1_pos
    error.Vs1_neg             = (truth.Vs1_neg             - V_n_data.Vs1.negative)              / truth.Vs1_neg
    error.Va_pos              = (truth.Va_pos              - V_n_data.Va.positive)               / truth.Va_pos
    error.Va_neg              = (truth.Va_neg              - V_n_data.Va.negative)               / truth.Va_neg
    error.Vc                  = (truth.Vc                  - V_n_data.Vc)                        / truth.Vc
    error.Vd                  = (truth.Vd                  - V_n_data.Vd)                        / truth.Vd
    error.limit_load_pos      = (truth.limit_load_pos      - V_n_data.positive_limit_load)       / truth.limit_load_pos
    error.limit_load_neg      = (truth.limit_load_neg      - V_n_data.negative_limit_load)       / truth.limit_load_neg
    error.dive_limit_load_pos = (truth.dive_limit_load_pos - V_n_data.limit_loads.dive.positive) / truth.dive_limit_load_pos
    error.dive_limit_load_neg = (truth.dive_limit_load_neg - V_n_data.limit_loads.dive.negative) / truth.dive_limit_load_neg

    for k, v in error.items():
        assert np.abs(v) < 1e-6, f'Part 25: {k} error {v:.2e} exceeds tolerance'

    print('  PASSED')


if __name__ == '__main__':
    main()
