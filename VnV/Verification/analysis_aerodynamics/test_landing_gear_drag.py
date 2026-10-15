# VnV/Verification/analysis_aerodynamics/test_landing_gear_drag.py
#
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core                            import Units, Data
from RCAIDE.Framework.Mission.Common                  import Results
from RCAIDE.Library.Methods.Aerodynamics.Common.Drag  import miscellaneous_drag
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def gear_drag(gear_extended, flap_deg, max_takeoff=46800.0, reference_area=79.84):
    """Landing-gear part of the miscellaneous drag for an E175-sized vehicle."""
    vehicle                          = RCAIDE.Vehicle()
    vehicle.reference_area           = reference_area
    vehicle.mass_properties.max_takeoff = max_takeoff
    wing                             = RCAIDE.Library.Components.Wings.Main_Wing()
    wing.areas.wetted                = 2.0 * reference_area
    flap                             = RCAIDE.Library.Components.Wings.Control_Surfaces.Flap()
    flap.deflection                  = flap_deg * Units.deg
    wing.append_control_surface(flap)
    vehicle.append_component(wing)
    for gear in (RCAIDE.Library.Components.Landing_Gear.Main_Landing_Gear(), RCAIDE.Library.Components.Landing_Gear.Nose_Landing_Gear()):
        gear.gear_extended = gear_extended
        vehicle.append_component(gear)

    def total():
        state = Data(); state.conditions = Results()
        state.conditions.freestream.mach_number = np.array([[0.2]])
        miscellaneous_drag(state, Data(), vehicle)
        return state.conditions.aerodynamics.coefficients.drag.miscellaneous.total[0, 0]

    with_gear = total()
    for gear in vehicle.landing_gears:
        gear.gear_extended = False
    return with_gear - total()

# ----------------------------------------------------------------------------------------------------------------------
#  Tests
# ----------------------------------------------------------------------------------------------------------------------
def test_gear_drag_matches_gasp_correlation():
    # NASA GASP (Aviary GearDragIncrement): f = 0.0033 W^0.785 ft^2 with W in lb, CD = f/S (1 - 0.4545 flap/50)
    W_lb     = 46800.0 / 0.45359237
    f_m2     = 0.0033 * W_lb**0.785 * 0.3048**2
    for flap_deg in (0.0, 10.0, 30.0):
        expected = f_m2 / 79.84 * (1 - 0.454545 * flap_deg / 50)
        assert np.isclose(gear_drag(True, flap_deg), expected, rtol=1e-9)


def test_e175_gear_drag_magnitude():
    # flaps up ~0.033 and landing flaps ~0.024: between Raymer's 0.02 and Roskam Fig. 4.60 for this weight
    assert 0.030 < gear_drag(True, 0.0) < 0.036
    assert 0.021 < gear_drag(True, 30.0) < 0.027


def test_no_gear_drag_when_retracted():
    assert gear_drag(False, 30.0) == 0.0


def main():
    test_gear_drag_matches_gasp_correlation()
    test_e175_gear_drag_magnitude()
    test_no_gear_drag_when_retracted()
    return


if __name__ == '__main__':
    main()
