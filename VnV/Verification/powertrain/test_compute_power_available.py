# VnV/Verification/powertrain/test_compute_power_available.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Powertrain.Converters.Engine import compute_power_available

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # A flat-rated piston engine holds its sea-level power up to the flat-rate altitude, then loses power with altitude
    check_flat_rated_below_and_lapsed_above()
    check_inputs_not_modified()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_flat_rated_below_and_lapsed_above():
    sea_level_power    = 120e3   # sea-level static power [W]
    flat_rate_altitude = 1500.   # altitude up to which the engine delivers sea-level power [m]
    engine             = Data(sealevel_static_power=sea_level_power, flat_rate_altitude=flat_rate_altitude)

    altitude = np.linspace(0., 9000., 37)[:, None]
    power    = compute_power_available(engine, altitude, np.zeros_like(altitude))

    # below the flat-rate altitude: exactly sea-level power
    below = altitude[:, 0] < flat_rate_altitude
    assert np.all(power[below] == sea_level_power)

    # above it: the Gagg and Farrar lapse with the density ratio sigma, as given in Gudmundsson (2014)
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    sigma      = (atmosphere.compute_values(altitude[~below] - flat_rate_altitude, 0.).density
                  / atmosphere.compute_values(0., 0.).density[0, 0])
    assert np.allclose(power[~below], sea_level_power * (sigma - 0.117) / 0.883, rtol=1e-14)

    # power is continuous at the flat-rate altitude and only falls above it
    assert np.isclose(power[~below][0, 0], sea_level_power)
    assert np.all(np.diff(power[:, 0]) <= 0.)
    return


def check_inputs_not_modified():
    engine   = Data(sealevel_static_power=120e3, flat_rate_altitude=1500.)
    altitude = np.array([[0.], [500.], [4000.]])
    before   = altitude.copy()

    compute_power_available(engine, altitude, np.zeros_like(altitude))

    # the altitude is the mission's array, so the function must not change it
    assert np.array_equal(altitude, before)
    return


if __name__ == '__main__':
    main()
