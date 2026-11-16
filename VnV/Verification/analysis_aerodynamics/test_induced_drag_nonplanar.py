# VnV/Verification/analysis_aerodynamics/test_induced_drag_nonplanar.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Mission.Common.Pre_Process.geometry import geometry_preprocess_routine
from RCAIDE.Library.Methods.Geometry.Planform import wing_planform
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM import VLM

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The Trefftz-plane induced drag of surfaces that do not lie flat: wings with dihedral, a surface without a mirror
    # image, and a vertical fin. The drag of a strip is its force normal to its own trace in the Trefftz plane times
    # the angle the wake turns the flow by, so a surface is treated alike at any dihedral angle.
    check_dihedral_against_AVL()
    check_fin_is_half_wing()
    check_continuous_through_vertical()
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_dihedral_against_AVL():
    # Rectangular wing of aspect ratio 8 on its projected span, 4 degrees angle of attack, Mach 0.2. Reference: AVL 3.35
    # (Drela and Youngren), 30 spanwise by 8 chordwise vortices, lift coefficient and Trefftz-plane induced drag. The two
    # codes differ by 2 percent in span efficiency on the flat wing, and by the same amount with dihedral.
    AVL = {0.: (0.31961, 0.004190), 30.: (0.29553, 0.003380), 60.: (0.20177, 0.001320)}
    for dihedral, (CL_AVL, CDi_AVL) in AVL.items():
        results    = coefficients(rectangular_wing(dihedral), alpha=4., beta=0., n_sw=30, n_cw=8)
        CL, CDi    = results.CLift[0, 0], results.CDrag_induced[0, 0]
        efficiency = CL**2 / (np.pi * 8. * CDi)
        reference  = CL_AVL**2 / (np.pi * 8. * CDi_AVL)
        print(f"  dihedral {dihedral:2.0f} deg: CL {CL:.5f} (AVL {CL_AVL:.5f}), CDi {CDi:.6f} (AVL {CDi_AVL:.6f}), span efficiency {efficiency:.4f} (AVL {reference:.4f})")
        assert abs(CL - CL_AVL) < 0.03 * CL_AVL, f"dihedral {dihedral}: lift coefficient {CL} against AVL {CL_AVL}"
        assert abs(efficiency - reference) < 0.04 * reference, f"dihedral {dihedral}: span efficiency {efficiency} against AVL {reference}"
    return


def check_fin_is_half_wing():
    # one tapered surface without a mirror image, laid flat at an angle of attack and stood vertical at the same angle
    # of sideslip: the same lifting surface turned a quarter turn, so the induced drag is the same
    for sweep in (0., 20.):
        half_wing = one_surface(vertical=False, sweep=sweep)
        fin       = one_surface(vertical=True,  sweep=sweep)
        for angle in (4., -4., 8.):
            flat     = coefficients(half_wing, alpha=angle, beta=0.).CDrag_induced[0, 0]
            upright  = coefficients(fin,       alpha=0.,    beta=angle).CDrag_induced[0, 0]
            assert flat > 0, f"sweep {sweep}, {angle} deg: induced drag of the half wing {flat}"
            assert abs(upright - flat) < 1e-9 * flat, f"sweep {sweep}, {angle} deg: induced drag {upright} upright against {flat} flat"
        print(f"  sweep {sweep:.0f} deg: the induced drag of the fin in sideslip is that of the half wing at angle of attack ({flat:.6f} at 8 deg)")

    # reference for the surface alone: AVL 3.35 at 4 degrees gives CL 0.17528 and a Trefftz-plane induced drag of 0.005080
    results = coefficients(one_surface(vertical=False, sweep=0.), alpha=4., beta=0.)
    assert abs(results.CDrag_induced[0, 0] - 0.005080) < 0.08 * 0.005080, f"induced drag of the surface alone {results.CDrag_induced[0, 0]} against AVL 0.005080"
    return


def check_continuous_through_vertical():
    # a fin canted by a small angle to either side has the induced drag of the upright fin
    fin     = one_surface(vertical=True, sweep=20.)
    surface = fin.wings.surface
    upright = coefficients(fin, alpha=2., beta=4.).CDrag_induced[0, 0]
    for cant in (-1e-4, -1e-9, 1e-9, 1e-4):
        surface.dihedral = cant
        wing_planform(surface)
        canted = coefficients(fin, alpha=2., beta=4.).CDrag_induced[0, 0]
        assert abs(canted - upright) < 1e-3 * upright, f"cant {cant} rad: induced drag {canted} against {upright} upright"
    surface.dihedral = 0.
    wing_planform(surface)
    print(f"  the induced drag of a fin is continuous in its cant through the vertical ({upright:.6f})")
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def rectangular_wing(dihedral):
    """A vehicle of one untwisted rectangular wing with a mirror image, aspect ratio 8 on its projected span of 20 m2."""
    area    = 20.
    span    = np.sqrt(8. * area)
    vehicle = RCAIDE.Vehicle()
    vehicle.reference_area                    = area
    vehicle.reference_chord                   = area / span
    vehicle.reference_span                    = span
    vehicle.mass_properties.center_of_gravity = [[0., 0., 0.]]

    wing                      = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                  = 'main_wing'
    wing.sweeps.quarter_chord = 0.
    wing.thickness_to_chord   = 0.12
    wing.areas.reference      = area
    wing.aspect_ratio         = 8.
    wing.taper                = 1.
    wing.spans.projected      = span
    wing.chords.root          = area / span
    wing.chords.tip           = area / span
    wing.twists.root          = 0.
    wing.twists.tip           = 0.
    wing.dihedral             = dihedral * Units.deg
    wing.origin               = [[0., 0., 0.]]
    wing.vertical             = False
    wing.xz_plane_symmetric   = True
    vehicle.append_component(wing)
    return preprocessed(vehicle)


def one_surface(vertical, sweep):
    """A vehicle of one tapered surface without a mirror image: a vertical fin, or a half wing."""
    vehicle                                   = RCAIDE.Vehicle()
    vehicle.reference_area                    = 1.163
    vehicle.reference_chord                   = 0.939
    vehicle.reference_span                    = 1.4816
    vehicle.mass_properties.center_of_gravity = [[0., 0., 0.]]

    wing                      = RCAIDE.Library.Components.Wings.Vertical_Tail() if vertical else RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                  = 'surface'
    wing.sweeps.quarter_chord = sweep * Units.deg
    wing.thickness_to_chord   = 0.12
    wing.areas.reference      = 1.163
    wing.aspect_ratio         = 1.8874
    wing.taper                = 0.482
    wing.spans.projected      = 1.4816
    wing.chords.root          = 1.2176
    wing.chords.tip           = 0.5870
    wing.twists.root          = 0.
    wing.twists.tip           = 0.
    wing.origin               = [[0., 0., 0.]]
    wing.vertical             = vertical
    wing.xz_plane_symmetric   = False
    vehicle.append_component(wing)
    return preprocessed(vehicle)


def preprocessed(vehicle):
    analyses         = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle
    analyses.append(RCAIDE.Framework.Analyses.Geometry.Geometry())
    geometry_preprocess_routine(analyses)
    return vehicle


def coefficients(vehicle, alpha, beta, n_sw=16, n_cw=6):
    """The vortex lattice results of the vehicle at an angle of attack and a sideslip angle in degrees, Mach 0.2."""
    settings = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method().settings
    settings.number_of_spanwise_vortices  = n_sw
    settings.number_of_chordwise_vortices = n_cw
    one        = np.ones((1, 1))
    conditions = RCAIDE.Framework.Mission.Common.Results()
    conditions.freestream.mach_number      = 0.2 * one
    conditions.freestream.velocity         = 68. * one
    conditions.aerodynamics.angles.alpha   = alpha * Units.deg * one
    conditions.aerodynamics.angles.beta    = beta * Units.deg * one
    conditions.static_stability.pitch_rate = 0. * one
    conditions.static_stability.roll_rate  = 0. * one
    conditions.static_stability.yaw_rate   = 0. * one
    return VLM(conditions, settings, vehicle)


if __name__ == '__main__':
    main()
