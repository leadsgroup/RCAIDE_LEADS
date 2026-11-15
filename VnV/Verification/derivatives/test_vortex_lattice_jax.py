# VnV/Verification/derivatives/test_vortex_lattice_jax.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Data, set_at
from RCAIDE.Library.Mission.Common.Pre_Process.geometry import geometry_preprocess_routine
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method import generate_vortex_distribution, compute_wing_induced_velocity
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates import control_surface_conditions

# lattice coordinates the influence matrix is computed from
COORDINATES = ('XAH', 'YAH', 'ZAH', 'XBH', 'YBH', 'ZBH', 'XC', 'YC', 'ZC')

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The vortex lattice method is written once for NumPy and JAX, so that the aerodynamic coefficients can be
    # differentiated with respect to the geometry. Each stage is checked on a conventional transport (wing, tails) and
    # on a blended wing body: JAX values against NumPy, the compiled function, the forward-mode derivative against
    # central differences of the NumPy path, forward against reverse mode, and that every input reaches an output.
    try:
        import jax
    except ImportError:
        print("JAX is not installed (pip install RCAIDE[ad]); JAX checks skipped")
        return
    jax.config.update("jax_enable_x64", True)
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Vehicles")))

    for name in ('Embraer_190', 'BWB'):
        lattice = coarse_lattice(name)
        check_influence_matrix(name, lattice)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_influence_matrix(name, lattice):
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives
    mach  = np.array([[0.6]])
    phi   = np.arctan((lattice.ZBC - lattice.ZAC) / (lattice.YBC - lattice.YAC))
    delta = np.arctan((lattice.ZC - lattice.ZCH) / (lattice.XC - lattice.XCH))

    # The two ends of a horseshoe vortex on a vertical surface have the same lateral coordinate, and which end is
    # taken first follows from which is larger. Moving one end sideways alone flips that order, a jump no change of
    # the geometry produces, so the lateral coordinates of those panels are held and the others are inputs.
    vertical = np.isclose(lattice.YAH, lattice.YBH)[0]
    free     = np.flatnonzero(~vertical)

    def influence(*arrays):
        moved = Data()
        for key in lattice.keys():
            moved[key] = lattice[key]
        for key, value in zip(COORDINATES, arrays):
            if key in ('YAH', 'YBH'):
                value = set_at(np.array(lattice[key], dtype=float), (slice(None), free), value)
            moved[key] = value
        A, s, RFLAG, EW = compute_wing_induced_velocity(moved, mach, compute_EW=True, delta=arrays[-2], phi=arrays[-1])
        return A, s, EW

    inputs  = tuple(np.array(lattice[key], dtype=float)[:, free] if key in ('YAH', 'YBH') else np.array(lattice[key], dtype=float)
                    for key in COORDINATES) + (np.array(delta, dtype=float), np.array(phi, dtype=float))
    report  = check_derivatives(influence, inputs, steps=(1e-5, 1e-6, 1e-7), n_directions=2)
    reached = [float(np.max(np.abs(sensitivity))) for sensitivity in report.input_sensitivity]
    print(f"  {name} influence matrix, {len(vertical)} panels ({int(np.sum(vertical))} on vertical surfaces): values {report.value_error:.1e}, "
          f"derivatives {report.jvp_error:.1e}, reverse {report.vjp_consistency_error:.1e}, weakest input {min(reached):.1e}")
    assert report.passed, f"{name}: influence matrix JAX check failed, {report}"
    assert min(reached) > 0, f"{name}: an input does not reach the influence matrix, {dict(zip(COORDINATES + ('delta', 'phi'), reached))}"
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def coarse_lattice(name):
    """The vortex lattice of a VnV vehicle with 8 spanwise and 3 chordwise panels per surface."""
    vehicle          = __import__(name).vehicle_setup()
    analyses         = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle
    analyses.append(RCAIDE.Framework.Analyses.Geometry.Geometry())
    geometry_preprocess_routine(analyses)
    settings = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method().settings
    settings.number_of_spanwise_vortices  = 8
    settings.number_of_chordwise_vortices = 3
    return generate_vortex_distribution(control_surface_conditions(np.array([0.6])), settings, vehicle)


if __name__ == '__main__':
    main()
