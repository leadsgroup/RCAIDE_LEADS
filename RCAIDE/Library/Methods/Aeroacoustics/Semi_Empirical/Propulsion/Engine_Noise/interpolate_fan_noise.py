# RCAIDE/Library/Methods/Aeroacoustics/Semi_Empirical/Propulsion/Engine_Noise/interpolate_fan_noise.py
#
# Created:  Jul 2026, P. Siripun, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Data

import numpy as np
from scipy.interpolate import interp1d

# ----------------------------------------------------------------------------------------------------------------------
#  Fan Noise Directivity Tables
# ----------------------------------------------------------------------------------------------------------------------
# Table I: Low Frequency Core Noise (C1)
# log_Sc rows: -3.6, -2.2 to 2.0 (increments of 0.1), and 3.6
angles = np.array([0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180])

data = Data()
data["Inlet Broadband"]   = [-0.5, -1, -1.25, -1.41, -1.4, -2.2, -4.5, -8.5, -13, -18.5, -24, -30, -36, -42, -48, -54, -60, -66, -73]
data["Inlet Tones"]       = [-3, -1.5, 0, 0, 0, -1.2, -3.5, -6.8, -10.5, -15.5, -19, -25, -32, -40, -49, -59, -70, -80, -90]
data["Combination Tones"] = [-28, -23, -18, -13, -8, -3, 0, -1.3, -2.6, -3.9, -5.2, -6.5, -7.9, -9.4, -11, -12.7, -14.5, -16.4, -18.4]
data["Aft Broadband"]     = [-30, -25, -20.8, -19.5, -18.4, -16.7, -14.5, -12, -9.6, -6.9, -4.5, -1.8, -0.3, 0.5, 0.7, -1.9, -4.5, -9, -15]
data["Aft Tones"]         = [-50, -41, -33, -26, -20.6, -17.9, -14.7, -11.2, -9.3, -7.1, -4.7, -2, 0, 0.8, 1, -1.6, -4.2, -9, -15]

def create_interpolator_fan():
    """Builds a linear interpolator for each fan noise directivity table."""
    interpolators = Data()
    for label, values in data.items():
        interpolators[label] = interp1d(angles, values, kind='linear', fill_value="extrapolate")
    return interpolators

def get_spl_fan(interpolators, source_name, angle):
    if source_name not in interpolators:
        raise ValueError(f"Source '{source_name}' not found. Choose from: {list(data.keys())}")

    return interpolators[source_name](angle)
