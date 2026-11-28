# VnV/Verification/aircraft_performance/test_take_off_field_length_turboprop.py
#
# Created:  Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Performance.estimate_take_off_field_length import estimate_take_off_field_length

import os
import sys
import numpy as np
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'Vehicles'))
from ATR_72 import vehicle_setup, configs_setup

import take_off_field_length_test as reference

# The takeoff field length estimate on a turboprop, whose off-design model reads the freestream state as column
# arrays. The reference is the manufacturer's value for the ATR 72-600: 1,304 m at maximum takeoff mass, ISA, sea
# level (ATR 72-600 brochure, "Take-off field length"). The estimate is an empirical correlation run on the clean
# configuration at 22,800 kg, so the two are compared to 15%.

# ----------------------------------------------------------------------------------------------------------------------
#  Main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    check_turboprop_take_off_field_length()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_turboprop_take_off_field_length():
    vehicle  = vehicle_setup()
    configs  = configs_setup(vehicle)
    analyses = reference.analyses_setup(configs)

    field_length, climb_gradient = estimate_take_off_field_length(analyses=analyses.base, compute_2nd_seg_climb=True)
    print('Takeoff field length [m]: ', field_length, ' second segment climb gradient: ', climb_gradient)

    brochure_field_length = 1304.0
    assert np.isfinite(field_length) and np.isfinite(climb_gradient) and climb_gradient > 0
    assert abs(field_length / brochure_field_length - 1) < 0.15
    return


if __name__ == '__main__':
    main()
    print('test_take_off_field_length_turboprop passed')
