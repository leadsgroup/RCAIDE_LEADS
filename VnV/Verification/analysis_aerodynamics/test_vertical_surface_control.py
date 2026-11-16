# VnV/Verification/analysis_aerodynamics/test_vertical_surface_control.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Mission.Common.Pre_Process.geometry import geometry_preprocess_routine
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.VLM import VLM

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # A vertical fin with a rudder is the same lifting surface as a half wing with a flap, turned a quarter turn. The
    # vortex lattice must give the fin the side force that it gives the half wing as lift, and left rudder must be the
    # mirror image of right rudder. Checked on one tapered surface, unswept and swept.
    for sweep in (0., 20.):
        half_wing, flap = one_surface(vertical=False, sweep=sweep)
        fin, rudder     = one_surface(vertical=True,  sweep=sweep)
        check_sideslip_is_angle_of_attack(sweep, half_wing, fin)
        check_left_rudder_mirrors_right_rudder(sweep, fin, rudder)
        check_rudder_is_flap(sweep, half_wing, flap, fin, rudder)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_sideslip_is_angle_of_attack(sweep, half_wing, fin):
    # positive sideslip pushes the fin toward negative y, as positive angle of attack lifts the half wing
    lift       = coefficients(half_wing, alpha=4., beta=0.).CLift[0, 0]
    side_force = coefficients(fin,       alpha=0., beta=4.).CY[0, 0]
    assert abs(side_force + lift) < 1e-9 * abs(lift), f"sweep {sweep}: side force {side_force} against lift {lift}"
    print(f"  sweep {sweep:.0f} deg: fin at 4 deg of sideslip {side_force:+.6f}, half wing at 4 deg angle of attack {lift:+.6f}")
    return


def check_left_rudder_mirrors_right_rudder(sweep, fin, rudder):
    for deflection in (2., 10., 25.):
        rudder.deflection = deflection * Units.deg
        right             = coefficients(fin, alpha=0., beta=0.)
        rudder.deflection = -deflection * Units.deg
        left              = coefficients(fin, alpha=0., beta=0.)
        rudder.deflection = 0.
        for key in ('CY', 'CL', 'CN'):
            assert abs(right[key][0, 0] + left[key][0, 0]) < 1e-9 * abs(right[key][0, 0]), \
                   f"sweep {sweep}, rudder {deflection} deg: {key} is {right[key][0, 0]} to the right and {left[key][0, 0]} to the left"
    print(f"  sweep {sweep:.0f} deg: side force, rolling and yawing moment of left rudder mirror those of right rudder")
    return


def check_rudder_is_flap(sweep, half_wing, flap, fin, rudder):
    # the two differ in the second order of the deflection: 0.05 percent at 2 degrees, 0.5 percent at 10 degrees
    for deflection, tolerance in ((2., 5e-4), (-2., 5e-4), (10., 5e-3), (-10., 5e-3)):
        flap.deflection = rudder.deflection = deflection * Units.deg
        lift            = coefficients(half_wing, alpha=0., beta=0.).CLift[0, 0]
        side_force      = coefficients(fin,       alpha=0., beta=0.).CY[0, 0]
        flap.deflection = rudder.deflection = 0.
        assert abs(side_force - lift) < tolerance * abs(lift), f"sweep {sweep}, {deflection} deg: side force {side_force} against lift {lift}"
        print(f"  sweep {sweep:.0f} deg, control at {deflection:+.0f} deg: side force of the fin {side_force:+.6f}, lift of the half wing {lift:+.6f}")
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def one_surface(vertical, sweep):
    """A vehicle of one tapered surface with a trailing-edge control over 10 to 90 percent of its span: a vertical fin
    with a rudder, or a half wing with a flap."""
    vehicle                                   = RCAIDE.Vehicle()
    vehicle.reference_area                    = 1.163
    vehicle.reference_chord                   = 0.939
    vehicle.reference_span                    = 1.4816
    vehicle.mass_properties.center_of_gravity = [[0., 0., 0.]]

    if vertical:
        wing    = RCAIDE.Library.Components.Wings.Vertical_Tail()
        control = RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder()
    else:
        wing    = RCAIDE.Library.Components.Wings.Main_Wing()
        control = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
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

    control.tag                 = 'control'
    control.span_fraction_start = 0.1
    control.span_fraction_end   = 0.9
    control.chord_fraction      = 0.3
    control.deflection          = 0.
    wing.append_control_surface(control)
    vehicle.append_component(wing)

    analyses         = RCAIDE.Framework.Analyses.Vehicle()
    analyses.vehicle = vehicle
    analyses.append(RCAIDE.Framework.Analyses.Geometry.Geometry())
    geometry_preprocess_routine(analyses)
    return vehicle, control


def coefficients(vehicle, alpha, beta):
    """The vortex lattice results of the vehicle at an angle of attack and a sideslip angle in degrees, Mach 0.2."""
    settings = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method().settings
    settings.number_of_spanwise_vortices  = 16
    settings.number_of_chordwise_vortices = 6
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
