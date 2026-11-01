# VnV/Verification/network_hydrogen/test_refuel_segment.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
import pytest

import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Mission.Segments.Ground.Refuel import initialize_conditions

FULL_MASS = 1000.0
DURATION  = 1800.0


def refuel_segment(fill_fraction=None, fill_rate=None, mission_fuel=0.0, initial_fuel=200.0, chained_fuel=None):
    """Ground refuel of one 1,000 kg cryogenic tank over 30 minutes."""
    tank                              = RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank()
    tank.tag                          = 'lh2_tank'
    tank.design_full_liquid_mass      = FULL_MASS
    tank.fuel                         = RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen()
    tank.fuel.mass_properties.mass    = mission_fuel
    network                           = RCAIDE.Framework.Networks.Fuel()
    network.sources.append(tank)

    n       = 4
    segment = Data()
    segment.time                        = DURATION
    segment.refuel_target_fill_fraction = fill_fraction
    segment.nominal_fill_rate           = fill_rate
    segment.analyses                    = Data(vehicle=Data(networks=[network]))
    segment.state                       = Data()
    segment.state.numerics              = Data(dimensionless=Data(control_points=np.linspace(0, 1, n)[:, None]))
    segment.state.conditions            = Data()
    segment.state.conditions.frames     = Data(inertial=Data(time=100.0 * np.ones((n, 1))))
    segment.state.conditions.energy     = Data(sources=Data())
    segment.state.conditions.energy.sources[tank.tag] = Data(fuel_mass=initial_fuel * np.ones((n, 1)),
                                                             refuel_mass_flow_rate=np.zeros((n, 1)),
                                                             refuel_target_mass=np.zeros((n, 1)))
    segment.state.initials = Data()
    if chained_fuel is not None:
        segment.state.initials.conditions = Data(energy=Data(sources=Data()))
        segment.state.initials.conditions.energy.sources[tank.tag] = Data(fuel_mass=chained_fuel * np.ones((n, 1)))
    return segment, segment.state.conditions.energy.sources[tank.tag]


@pytest.mark.parametrize("fill_fraction, mission_fuel, target", [(0.8, 0.0, 800.0),        # set fraction of full
                                                                 (None, 600.0, 600.0),      # the mission fuel load
                                                                 (None, 1500.0, FULL_MASS), # mission load capped at full
                                                                 (None, 0.0, FULL_MASS)])   # no load set: full
def test_refuel_target(fill_fraction, mission_fuel, target):
    segment, tank_conditions = refuel_segment(fill_fraction=fill_fraction, mission_fuel=mission_fuel)
    initialize_conditions(segment)
    assert np.all(tank_conditions.refuel_target_mass == target)
    # default rate: 1.2 times the rate that reaches the target over the segment
    assert np.allclose(tank_conditions.refuel_mass_flow_rate, 1.2 * (target - 200.0) / DURATION)
    np.testing.assert_allclose(segment.state.conditions.frames.inertial.time[:, 0], 100.0 + np.linspace(0, DURATION, 4))


def test_refuel_nominal_rate_and_chained_start():
    segment, tank_conditions = refuel_segment(fill_rate=0.5)
    initialize_conditions(segment)
    assert np.all(tank_conditions.refuel_mass_flow_rate == 0.5)

    # a chained segment starts from the previous segment's final fuel mass; a full tank gets no flow
    segment, tank_conditions = refuel_segment(fill_fraction=0.8, chained_fuel=900.0)
    initialize_conditions(segment)
    assert np.all(tank_conditions.refuel_mass_flow_rate == 0.0)
