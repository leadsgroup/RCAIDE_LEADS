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

PSLS, H_FLAT = 120e3, 1500.


def test_flat_rated_below_and_lapsed_above():
    engine   = Data(sealevel_static_power=PSLS, flat_rate_altitude=H_FLAT)
    altitude = np.linspace(0., 9000., 37)[:, None]
    power    = compute_power_available(engine, altitude, np.zeros_like(altitude))
    below    = altitude[:, 0] < H_FLAT
    assert np.all(power[below] == PSLS)
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    sigma      = (atmosphere.compute_values(altitude[~below] - H_FLAT, 0.).density
                  / atmosphere.compute_values(0., 0.).density[0, 0])
    assert np.allclose(power[~below], PSLS * (sigma - 0.117) / 0.883, rtol=1e-14)      # Gudmundsson (2014)
    assert np.isclose(power[~below][0, 0], PSLS) and np.all(np.diff(power[:, 0]) <= 0.)  # continuous, lapsing


def test_inputs_not_modified():
    engine   = Data(sealevel_static_power=PSLS, flat_rate_altitude=H_FLAT)
    altitude = np.array([[0.], [500.], [4000.]])
    before   = altitude.copy()
    compute_power_available(engine, altitude, np.zeros_like(altitude))
    assert np.array_equal(altitude, before)
