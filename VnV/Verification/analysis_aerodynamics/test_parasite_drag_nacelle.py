# VnV/Verification/analysis_aerodynamics/test_parasite_drag_nacelle.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                                 import Units, Data
from RCAIDE.Framework.Mission.Common                       import Results
from RCAIDE.Library.Methods.Aerodynamics.Common.Drag       import parasite_drag_nacelle, parasite_drag_pylon, parasite_total
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def build_vehicle(has_pylon):
    """Wingless, fuselage-free vehicle with one nacelle, so parasite_total sums only nacelle and pylon drag."""
    vehicle                = RCAIDE.Vehicle()
    vehicle.reference_area = 72.72

    nacelle                = RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle()
    nacelle.tag            = 'test_nacelle'
    nacelle.diameter       = 1.5
    nacelle.length         = 2.751
    nacelle.areas.wetted   = 1.1 * np.pi * nacelle.diameter * nacelle.length
    nacelle.has_pylon      = has_pylon

    propulsor              = RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan()
    propulsor.nacelle      = nacelle
    network                = RCAIDE.Framework.Networks.Fuel()
    network.propulsors.append(propulsor)
    vehicle.append_energy_network(network)
    return vehicle, nacelle


def build_state(altitude, mach_number):
    atmosphere = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    atmo       = atmosphere.compute_values(altitude)
    state      = Data()
    state.conditions = Results()
    freestream = state.conditions.freestream
    freestream.mach_number     = np.array([[mach_number]])
    freestream.temperature     = atmo.temperature
    freestream.reynolds_number = atmo.density * mach_number * atmo.speed_of_sound / atmo.dynamic_viscosity
    return state


def evaluate(has_pylon):
    vehicle, nacelle = build_vehicle(has_pylon)
    settings         = RCAIDE.Framework.Analyses.Aerodynamics.Vortex_Lattice_Method().settings
    state            = build_state(35000 * Units.ft, 0.78)
    parasite_drag_nacelle(state, settings, vehicle)
    raw = state.conditions.aerodynamics.coefficients.drag.parasite[nacelle.tag]
    # component build-up on the vehicle reference area (Raymer, Aircraft Design, Eq. 12.24)
    build_up = raw.form_factor[0, 0] * raw.skin_friction[0, 0] * nacelle.areas.wetted / vehicle.reference_area
    parasite_drag_pylon(state, settings, vehicle)
    parasite_total(state, settings, vehicle)
    return state, build_up, settings

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_nacelle_parasite_drag_matches_component_build_up():
    state, build_up, settings = evaluate(has_pylon=False)
    nacelles = state.conditions.aerodynamics.coefficients.drag.parasite.nacelles[0, 0]
    assert np.isclose(nacelles, build_up * (1 - settings.drag_reduction_factors.parasite_drag), rtol=1e-12)


def test_pylon_parasite_drag_is_summed():
    state, build_up, settings = evaluate(has_pylon=True)
    drag     = state.conditions.aerodynamics.coefficients.drag.parasite
    nacelles = drag.nacelles[0, 0]
    pylon    = drag['test_nacelle_pylon'].total[0, 0]
    assert np.isclose(pylon, 0.2 * build_up, rtol=1e-12)
    assert np.isclose(nacelles, (build_up + pylon) * (1 - settings.drag_reduction_factors.parasite_drag), rtol=1e-12)
    assert np.isclose(drag.total[0, 0], nacelles, rtol=1e-12)


def main():
    test_nacelle_parasite_drag_matches_component_build_up()
    test_pylon_parasite_drag_is_summed()
    return


if __name__ == '__main__':
    main()
