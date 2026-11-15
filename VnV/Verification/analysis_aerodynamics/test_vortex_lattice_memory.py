# VnV/Verification/analysis_aerodynamics/test_vortex_lattice_memory.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
import tracemalloc
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Mission.Common.Pre_Process.geometry import geometry_preprocess_routine
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method import VLM
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.compute_wing_induced_velocity import ROW_BLOCK_ENTRIES
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates import control_surface_conditions

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The memory of the vortex lattice method grows with the square of the number of panels. The influence matrix is
    # computed a block of control points at a time, so that only the matrix itself is of that size and the working
    # arrays are not. This measures the peak of one evaluation of the Embraer 190 at the default mesh, at a finer one
    # and at a supersonic Mach number, and fails if working arrays of the size of the matrix come back.
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Vehicles")))
    import Embraer_190
    vehicle          = Embraer_190.vehicle_setup()
    analyses         = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle
    analyses.append(RCAIDE.Framework.Analyses.Geometry.Geometry())
    geometry_preprocess_routine(analyses)

    check_peak_memory(vehicle, spanwise=30, chordwise=15, mach=0.3)
    check_peak_memory(vehicle, spanwise=40, chordwise=20, mach=0.3)
    check_peak_memory(vehicle, spanwise=30, chordwise=15, mach=1.6)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_peak_memory(vehicle, spanwise, chordwise, mach):
    settings   = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method().settings
    settings.number_of_spanwise_vortices  = spanwise
    settings.number_of_chordwise_vortices = chordwise
    conditions = control_surface_conditions(np.array([mach]))
    conditions.aerodynamics.angles.alpha[:] = 3.0 * Units.deg
    conditions.freestream.velocity[:]       = mach * 340.0

    tracemalloc.start()
    results = VLM(conditions, settings, vehicle)
    peak    = tracemalloc.get_traced_memory()[1]
    tracemalloc.stop()

    # allowed: the influence matrix, its copy in the linear solve and two more of that size, and 40 working arrays
    panels  = np.shape(results.gamma)[1]
    matrix  = 8. * panels**2
    working = 8. * ROW_BLOCK_ENTRIES
    allowed = 4 * matrix + 40 * working
    assert peak < allowed, (f"Mach {mach}, {panels} panels: peak {peak / 1e9:.2f} GB, allowed {allowed / 1e9:.2f} GB "
                            f"(the influence matrix is {matrix / 1e9:.2f} GB)")
    print(f"  Mach {mach}, {panels} panels: peak {peak / 1e9:.2f} GB, {peak / matrix:.1f} times the influence matrix; allowed {allowed / 1e9:.2f} GB")
    return


if __name__ == '__main__':
    main()
