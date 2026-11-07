# VnV/Verification/powertrain/test_compute_rotor_wake_induced_velocity.py
#
# Created: Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.compute_rotor_wake_induced_velocity import compute_rotor_wake_induced_velocity

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_fidelity_without_wake_model_induces_no_velocity():
    rotor          = RCAIDE.Library.Components.Powertrain.Converters.Propeller()
    rotor.fidelity = 'Blade_Element_Momentum_Theory'
    points         = Data(XC=np.zeros((2, 5)), YC=np.zeros((2, 5)), ZC=np.zeros((2, 5)))
    V_ind          = compute_rotor_wake_induced_velocity(rotor, None, points)
    np.testing.assert_array_equal(V_ind, np.zeros((2, 5, 3)))
