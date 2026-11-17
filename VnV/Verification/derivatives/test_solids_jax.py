# VnV/Verification/derivatives/test_solids_jax.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

from RCAIDE.Library.Methods.Geometry.Airfoil import compute_naca_4series
from RCAIDE.Library.Methods.Geometry.Solids  import cut_section, loft_properties, combine_properties, mirror_properties, scale_properties

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The volume, center of gravity and inertia of a symmetric two-panel wing tank, as functions of the chords, the
    # sweep and dihedral offsets, the span and the spar positions, run on NumPy, on JAX and compiled with jax.jit; the
    # forward and reverse derivatives are compared with central differences of the NumPy path and with each other.
    try:
        import jax
    except ImportError:
        print("JAX is not installed (pip install RCAIDE[ad]); JAX checks skipped")
        return
    jax.config.update("jax_enable_x64", True)
    from RCAIDE.Library.Methods.Utilities.derivative_check import check_derivatives

    inputs = (np.array([6.0, 3.5, 1.2]),          # chords at the root, the break and the tip
              np.array([2.5, 7.0]),               # leading-edge x of the break and the tip
              np.array([0.4, 1.0]),               # height of the break and the tip
              np.array([5.0, 14.0]),              # spanwise position of the break and the tip
              np.array([0.15, 0.65]))             # front and rear spar chord fractions
    report = check_derivatives(tank_properties, inputs, steps=(1e-4, 1e-5, 1e-6), n_directions=3)
    print(f"  tank volume, center of gravity and inertia: values {report.value_error:.1e}, compiled {report.compiled_error:.1e}, "
          f"derivatives {report.jvp_error:.1e}, reverse {report.vjp_consistency_error:.1e}")
    assert report.passed, f"solids JAX check failed: {report}"
    reached = [float(np.max(np.abs(s))) for s in report.input_sensitivity]
    assert min(reached) > 0, f"inputs that do not reach the tank properties: {reached}"
    print('test_solids_jax passed')
    return


# NACA 2412 per unit chord, built once: the airfoil is fixed, the planform and spars move
AIRFOIL = compute_naca_4series('2412', 201)


def tank_properties(chords, x_offsets, z_offsets, spans, spars):
    """Volume, center of gravity and inertia per unit mass of a symmetric two-panel tank scaled to 85% usable."""
    x, z     = cut_section(AIRFOIL.x_upper_surface, AIRFOIL.y_upper_surface, AIRFOIL.x_lower_surface,
                           AIRFOIL.y_lower_surface, spars[0], spars[1], 41)
    sections = [(x * chords[0], z * chords[0]),
                (x * chords[1] + x_offsets[0], z * chords[1] + z_offsets[0]),
                (x * chords[2] + x_offsets[1], z * chords[2] + z_offsets[1])]
    ys       = [2.0, spans[0], spans[1]]
    panels   = [loft_properties(*sections[i], *sections[i + 1], ys[i], ys[i + 1]) for i in range(2)]
    half     = scale_properties(combine_properties(panels), 0.85)
    whole    = combine_properties([half, mirror_properties(half)])
    return whole.volume, whole.center_of_gravity, whole.inertia_per_mass


if __name__ == '__main__':
    main()
