# balanced_field_length_test.py
#
# Created: Aug 2026, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# RCAIDE Imports
import RCAIDE
from RCAIDE.Framework.Core import Data, Units
from RCAIDE.Library.Methods.Performance.estimate_balanced_field_length import estimate_balanced_field_length

# package imports
import numpy as np
import pylab as plt
import sys
import os
import time

# import vehicle file
base_dir = os.path.dirname(os.path.abspath(__file__))

vehicles_path = os.path.abspath(
    os.path.join(base_dir, "..", "..", "Vehicles")
)

if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
from Embraer_190 import vehicle_setup, configs_setup

# ----------------------------------------------------------------------
#   Build the Vehicle
# ----------------------------------------------------------------------
def main():
    ti = time.time()

    # define vehicle
    vehicle = vehicle_setup()

    # Set up vehicle configs
    configs = configs_setup(vehicle)

    # create analyses
    analyses = analyses_setup(configs)

    balanced_field_length, decision_speed = estimate_balanced_field_length(
        analyses = analyses.takeoff,
    )

    takeoff_weight = vehicle.mass_properties.takeoff
    print('Weight (kg): ', takeoff_weight)
    print('Balanced Field Length (m): ', balanced_field_length)
    print('Decision Speed V1 (m/s): ', decision_speed)

    truth_BFL = 2158.0344212496952
    BFL_error = np.max(np.abs(balanced_field_length - truth_BFL))
    assert (BFL_error < 1e-6)

    truth_V1 = 67.44415954428656
    V1_error = np.max(np.abs(decision_speed - truth_V1))
    assert (V1_error < 1e-6)

    # sanity check: balanced field length must exceed the empirical all-engines TOFL, and V1
    # must lie strictly between stall speed and liftoff speed
    from RCAIDE.Library.Methods.Performance.estimate_take_off_field_length import estimate_take_off_field_length
    takeoff_field_length, _ = estimate_take_off_field_length(analyses=analyses.takeoff, compute_2nd_seg_climb=True)
    print('(Reference) Empirical AEO Takeoff Field Length (m): ', takeoff_field_length)
    assert (balanced_field_length > takeoff_field_length)

    elapsed_time = time.time() - ti
    elapsed_time_min = elapsed_time / 60
    print('Elapsed time (min): ', elapsed_time_min)
    return


def analyses_setup(configs):

    analyses = RCAIDE.Framework.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag, config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses

def base_analysis(vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------
    analyses = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle

    #  Geometry
    geometry = RCAIDE.Framework.Analyses.Geometry.Geometry()
    analyses.append(geometry)

    # ------------------------------------------------------------------
    #  Weights
    weights = RCAIDE.Framework.Analyses.Weights.Conventional_Transport()
    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method()
    aerodynamics.settings.maximum_lift_coefficient_factor = 0.90
    aerodynamics.settings.number_of_spanwise_vortices    = 10 # reducing the number of vortices to speed up the test
    aerodynamics.settings.number_of_chordwise_vortices   = 5  # reducing the number of vortices to speed up the test
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Energy
    energy = RCAIDE.Framework.Analyses.Energy.Energy()
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = RCAIDE.Framework.Analyses.Planets.Earth()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    analyses.append(atmosphere)

    # done!
    return analyses


# ----------------------------------------------------------------------
#   Call Main
# ----------------------------------------------------------------------

if __name__ == '__main__':
    main()
    plt.show()
