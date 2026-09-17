# turboprop_offdesign_pathways_test.py
#
# Verifies RCAIDE's turboprop performance pathways -- analytical cycle model
# and live off-design matching (turboprop.offdesign_matching) -- at design
# point and sea-level static, plus a generate_turboprop_deck() round trip
# through Turbofan_Surrogate (engine-agnostic despite the name -- it just
# interpolates thrust_N/fuel_mass_flow_rate vs. altitude/Mach/rating code;
# there is no compute_turboprop_performance dispatch to a `.surrogate` yet,
# so this only checks the deck itself, not a wired third pathway). Reuses
# the ATR 72's starboard_propulsor as the test engine.
#
# Created: Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core             import Data, Units
from RCAIDE.Library.Methods.Powertrain import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.design_turboprop_offdesign_matching import design_turboprop_offdesign_matching
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turboprop.generate_turboprop_deck import generate_turboprop_deck
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_Surrogate       import Turbofan_Surrogate

# reuse the already-validated ATR 72 turboprop rather than redefining a test engine
from VnV.Vehicles.ATR_72 import vehicle_setup as atr_72_vehicle_setup

# Python package imports
import tempfile
from pathlib import Path

import numpy as np

ENGINE_TAG = 'starboard_propulsor'

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def check(name, computed, truth, tol, results):
    error = abs(float(computed) - truth) / abs(truth)
    results.append((name, float(computed), truth, error, tol))
    return error


def evaluate_thrust(turboprop, fuel_line, altitude, mach_number, throttle=1.0):
    atmosphere     = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    speed_of_sound = float(np.ravel(atmosphere.compute_values(altitude).speed_of_sound)[0])
    state = setup_operating_conditions(turboprop, fuel_line, velocity_range=np.array([speed_of_sound * mach_number]),
                                        altitude=altitude)
    state.conditions.energy.propulsors[turboprop.tag].throttle[:, 0] = throttle
    turboprop.compute_performance(state, fuel_line)
    return float(state.conditions.energy.propulsors[turboprop.tag].thrust[0, 0])


def main():
    turboprop = atr_72_vehicle_setup().networks.fuel.propulsors[ENGINE_TAG]   # already design_turboprop()-ed
    fuel_line = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    # design_turboprop() stores this as a (1,1) array (derived from design_freestream_velocity), unlike
    # Turbofan/Turbojet's plain float -- flatten once here rather than at every call site below
    turboprop.design_mach_number = float(np.ravel(turboprop.design_mach_number)[0])

    results = []

    # ------------------------------------------------------------------------------------
    # 1. Design point: analytical and off-design matching both vs. turboprop.design_thrust
    # ------------------------------------------------------------------------------------
    design_thrust = turboprop.design_thrust

    F_analytical_design = evaluate_thrust(turboprop, fuel_line, turboprop.design_altitude, turboprop.design_mach_number)
    check('design point, analytical vs design_thrust [N]', F_analytical_design, design_thrust, 1e-6, results)

    design_constants, reference_point = design_turboprop_offdesign_matching(turboprop)
    turboprop.offdesign_matching = Data(design_constants=design_constants, reference_point=reference_point)

    F_offdesign_design = evaluate_thrust(turboprop, fuel_line, turboprop.design_altitude, turboprop.design_mach_number)
    check('design point, off-design matching vs design_thrust [N]', F_offdesign_design, design_thrust, 5e-2, results)

    # ------------------------------------------------------------------------------------
    # 2. Near-static behavior
    # ------------------------------------------------------------------------------------
    # Not checked here: compute_thrust.py's propeller term is F=P/V0, which -- by design,
    # per design_turboprop.py's own Step 26 comment -- blows up approaching V0=0. That's why
    # turboprop.sealevel_static_thrust is computed there with a *different* actuator-disk
    # formula instead of reading it off compute_performance() at low Mach, and both the
    # analytical and off-design-matching dispatch paths inherit the same F=P/V0 blowup at
    # M=0.01 -- unlike Turbofan/Turbojet, there is no well-behaved low-Mach point to check
    # continuity against. Sanity-check sealevel_static_thrust on its own terms instead: a
    # propeller's static thrust should exceed its cruise-design thrust, but by a bounded amount.
    assert design_thrust < turboprop.sealevel_static_thrust < 5 * design_thrust, \
        f"sealevel_static_thrust ({turboprop.sealevel_static_thrust:.0f} N) should exceed " \
        f"design_thrust ({design_thrust:.0f} N) but not implausibly so"

    # Off-design matching at a moderate off-design point (not the design altitude/Mach, not
    # near-static). Not cross-compared against the analytical model, same reasoning as
    # Turbofan's own test: the analytical model's fixed pressure ratios are known to diverge
    # away from its one design point, so this only checks off-design matching itself converges
    # to a physically sane (positive, sub-static) value.
    turboprop.offdesign_matching = Data(design_constants=design_constants, reference_point=reference_point)
    F_offdesign_offdesign_pt = evaluate_thrust(turboprop, fuel_line, 3000.0, 0.3)
    assert 0 < F_offdesign_offdesign_pt < 1.5 * turboprop.sealevel_static_thrust, \
        f"off-design matching thrust at an off-design point ({F_offdesign_offdesign_pt:.0f} N) " \
        f"should be positive and roughly bounded by sea-level-static thrust ({turboprop.sealevel_static_thrust:.0f} N)"

    # ------------------------------------------------------------------------------------
    # 3. Deck generation: generate_turboprop_deck() -> Turbofan_Surrogate build -> query() round trip
    # ------------------------------------------------------------------------------------
    turboprop.offdesign_matching = None

    # Turbofan_Surrogate requires an exact (ALT=0, XM=0) row to normalize against (see its
    # validate_deck()), so Mach=0 can't be dropped here despite the F=P/V0 caveat above --
    # this section only exercises the deck-generation/surrogate code paths (round-trip
    # self-consistency), not turboprop's near-static thrust physics (checked in section 2).
    altitude_range = np.array([0.0, turboprop.design_altitude, 12000.0])
    mach_range     = np.array([0.0, 0.2, turboprop.design_mach_number])
    deck_full = generate_turboprop_deck(turboprop, altitude_range, mach_range)
    deck_part = generate_turboprop_deck(turboprop, altitude_range, mach_range,
                                         combustor_exit_temperature=0.9 * reference_point.Tt4)
    deck = Data()
    for field in ['altitude_m', 'mach_number', 'thrust_N', 'fuel_mass_flow_rate', 'isa_deviation_k', 'rating_code']:
        deck[field] = np.concatenate([deck_full[field], deck_part[field]])

    surrogate = Turbofan_Surrogate().build(deck=deck)
    F_tabulated, _ = surrogate.query(deck.altitude_m[0], deck.mach_number[0])
    check('deck: query() reproduces its own tabulated point [N]', F_tabulated[0], deck.thrust_N[0], 1e-4, results)

    with tempfile.TemporaryDirectory() as tmp_dir:
        deck_xlsx = Path(tmp_dir) / "turboprop_deck.xlsx"
        Turbofan_Surrogate().build(deck=deck, save_path=deck_xlsx)
        reloaded_surrogate = Turbofan_Surrogate().build(deck_path=deck_xlsx)
    F_reloaded, _ = reloaded_surrogate.query(deck.altitude_m[0], deck.mach_number[0])
    check('deck: query() after save_path/deck_path xlsx round-trip [N]', F_reloaded[0], F_tabulated[0], 1e-6, results)

    # ---- Report ----
    width = max(len(r[0]) for r in results)
    print(f"\n{'Quantity':<{width}}  {'Computed':>14}  {'Reference':>14}  {'Error':>10}  {'Tol':>8}")
    print('-' * (width + 54))
    all_pass = True
    for name, computed, truth, error, tol in results:
        ok = error < tol
        all_pass &= ok
        flag = '' if ok else '  <-- FAIL'
        print(f"{name:<{width}}  {computed:>14.6g}  {truth:>14.6g}  {error:>10.2e}  {tol:>8.1e}{flag}")

    assert all_pass, "One or more quantities exceeded tolerance across the turboprop performance pathways"
    print('\nTurboprop analytical, off-design matching, and deck generation are mutually and self consistent within tolerance.')


if __name__ == '__main__':
    main()
