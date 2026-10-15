# VnV/Verification/analysis_aerodynamics/test_high_lift_induced_drag.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                                          import Units
from RCAIDE.Library.Methods.Geometry.Planform                                       import wing_planform
from RCAIDE.Library.Methods.Aerodynamics.Vortex_Lattice_Method.train_VLM_surrogates import train_high_lift_induced_drag, control_surface_conditions, call_VLM
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_wing_vehicle():
    """Swept, tapered transport-like wing (AR 8.5) with a part-span flap and a slat."""
    vehicle                       = RCAIDE.Vehicle()
    wing                          = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.tag                      = 'main_wing'
    wing.spans.projected          = 26.0
    wing.taper                    = 0.27
    wing.areas.reference          = 79.3
    wing.aspect_ratio             = wing.spans.projected**2 / wing.areas.reference
    wing.sweeps.quarter_chord     = 23.0 * Units.deg
    wing.thickness_to_chord       = 0.12
    wing.dihedral                 = 5.0 * Units.deg
    wing.origin                   = [[10.0, 0.0, 0.0]]
    wing.vertical                 = False
    wing.xz_plane_symmetric       = True
    wing_planform(wing)

    flap                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.tag                      = 'flap'
    flap.span_fraction_start      = 0.15
    flap.span_fraction_end        = 0.70
    flap.chord_fraction           = 0.25
    wing.append_control_surface(flap)

    slat                          = RCAIDE.Library.Components.Wings.Control_Surfaces.Slat()
    slat.tag                      = 'slat'
    slat.span_fraction_start      = 0.20
    slat.span_fraction_end        = 0.95
    slat.chord_fraction           = 0.10
    wing.append_control_surface(slat)

    vehicle.append_component(wing)
    vehicle.reference_area        = wing.areas.reference
    return vehicle, wing


def induced_drag(settings, vehicle, mach_number, alpha, surface, delta):
    conditions = control_surface_conditions(np.array([mach_number]))
    conditions.aerodynamics.angles.alpha[:] = alpha
    surface.deflection = delta
    CDi = call_VLM(conditions, settings, vehicle).CDrag_induced[0, 0]
    surface.deflection = 0.0
    return CDi

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_high_lift_induced_drag_matches_direct_vlm():
    vehicle, wing = build_wing_vehicle()
    settings      = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method().settings
    mach_number   = 0.2
    CDi_reference = np.array([induced_drag(settings, vehicle, mach_number, 1E-12, wing.control_surfaces.flap, 0.0)])
    training      = train_high_lift_induced_drag(settings, vehicle, np.array([mach_number]), CDi_reference)

    for surface, letter, cases in [(wing.control_surfaces.flap, 'f', [(9.7, 30.0), (5.0, 30.0), (9.7, 10.0), (2.0, 10.0), (9.7, 25.0)]),
                                   (wing.control_surfaces.slat, 's', [(9.7, 25.0), (5.0, 25.0)])]:
        p, q, r0, r1 = [training['high_lift_Cdrag_' + term + '_' + letter][0] for term in ('p', 'q', 'r0', 'r1')]
        for alpha_deg, delta_deg in cases:
            alpha, delta = alpha_deg * Units.deg, delta_deg * Units.deg
            direct       = induced_drag(settings, vehicle, mach_number, alpha, surface, delta)
            clean        = induced_drag(settings, vehicle, mach_number, alpha, surface, 0.0)
            model        = clean + p * delta + q * delta**2 + (r0 + r1 * delta) * alpha * delta
            # the deflection must change the induced drag, or the comparison below is vacuous
            assert abs(direct - clean) > 0.05 * clean, (letter, alpha_deg, delta_deg, direct, clean)
            # the zero-incidence linear derivative this replaces is 33-71% low for the flap here
            assert np.isclose(model, direct, rtol=0.05), (letter, alpha_deg, delta_deg, model, direct)


def main():
    test_high_lift_induced_drag_matches_direct_vlm()
    return


if __name__ == '__main__':
    main()
