# turbojet_offdesign_pathways_test.py
#
# Verifies RCAIDE's turbojet performance pathways -- analytical cycle model
# and live off-design matching (turbojet.offdesign_matching) -- at design
# point and sea-level static, plus a generate_turbojet_deck() round trip
# through Turbofan_Surrogate (engine-agnostic despite the name -- it just
# interpolates thrust_N/fuel_mass_flow_rate vs. altitude/Mach/rating code;
# there is no compute_turbojet_performance dispatch to a `.surrogate` yet,
# so this only checks the deck itself, not a wired third pathway). Reuses
# Concorde's outer_right_turbojet as the test engine.
#
# Created: Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core             import Data, Units
from RCAIDE.Library.Methods.Powertrain import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.design_turbojet_offdesign_matching import design_turbojet_offdesign_matching
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbojet.generate_turbojet_deck    import generate_turbojet_deck
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_Surrogate       import Turbofan_Surrogate

# reuse the already-validated Concorde turbojet rather than redefining a test engine
from VnV.Vehicles.Concorde import vehicle_setup as concorde_vehicle_setup

# Python package imports
import tempfile
from pathlib import Path

import numpy as np

ENGINE_TAG = 'outer_right_turbojet'

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def check(name, computed, truth, tol, results):
    error = abs(float(computed) - truth) / abs(truth)
    results.append((name, float(computed), truth, error, tol))
    return error


def evaluate_thrust(turbojet, fuel_line, altitude, mach_number, throttle=1.0):
    atmosphere     = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    speed_of_sound = float(np.ravel(atmosphere.compute_values(altitude).speed_of_sound)[0])
    state = setup_operating_conditions(turbojet, fuel_line, velocity_range=np.array([speed_of_sound * mach_number]),
                                        altitude=altitude)
    state.conditions.energy.propulsors[turbojet.tag].throttle[:, 0] = throttle
    turbojet.compute_performance(state, fuel_line)
    return float(state.conditions.energy.propulsors[turbojet.tag].thrust[0, 0])


def main():
    turbojet  = concorde_vehicle_setup().networks.fuel.propulsors[ENGINE_TAG]   # already design_turbojet()-ed
    fuel_line = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()

    results = []

    # ------------------------------------------------------------------------------------
    # 1. Design point: analytical and off-design matching both vs. turbojet.design_thrust
    # ------------------------------------------------------------------------------------
    design_thrust = turbojet.design_thrust

    F_analytical_design = evaluate_thrust(turbojet, fuel_line, turbojet.design_altitude, turbojet.design_mach_number)
    check('design point, analytical vs design_thrust [N]', F_analytical_design, design_thrust, 1e-6, results)

    design_constants, reference_point = design_turbojet_offdesign_matching(turbojet)
    turbojet.offdesign_matching = Data(design_constants=design_constants, reference_point=reference_point)

    F_offdesign_design = evaluate_thrust(turbojet, fuel_line, turbojet.design_altitude, turbojet.design_mach_number)
    # ~7% gap here vs. the turbofan test's ~2% (same fixed-gamma_c-vs-real-gas-model cause) --
    # Concorde's M2.02 design point runs far hotter/denser than the turbofan's M0.8, widening
    # the gap between a single representative gamma_c and RCAIDE's temperature-dependent gas model
    check('design point, off-design matching vs design_thrust [N]', F_offdesign_design, design_thrust, 1e-1, results)

    # ------------------------------------------------------------------------------------
    # 2. Sea-level static
    # ------------------------------------------------------------------------------------
    turbojet.offdesign_matching = None
    F_analytical_sls = evaluate_thrust(turbojet, fuel_line, 0.0, 0.01)
    check('SLS (M=0.01), analytical vs turbojet.sealevel_static_thrust [N]',
          F_analytical_sls, turbojet.sealevel_static_thrust, 1e-6, results)

    turbojet.offdesign_matching = Data(design_constants=design_constants, reference_point=reference_point)
    F_offdesign_sls_001    = evaluate_thrust(turbojet, fuel_line, 0.0, 0.01)
    F_offdesign_sls_exact0 = evaluate_thrust(turbojet, fuel_line, 0.0, 0.0)

    assert F_offdesign_sls_exact0 > 0
    check('SLS: off-design matching, M=0 vs M=0.01 continuity', F_offdesign_sls_exact0, F_offdesign_sls_001, 5e-2, results)

    assert F_offdesign_sls_exact0 > F_offdesign_design, \
        f"off-design SLS thrust ({F_offdesign_sls_exact0:.0f} N) should exceed design-point " \
        f"cruise thrust ({F_offdesign_design:.0f} N)"

    # ------------------------------------------------------------------------------------
    # 3. Deck generation: generate_turbojet_deck() -> Turbofan_Surrogate build -> query() round trip
    # ------------------------------------------------------------------------------------
    turbojet.offdesign_matching = None

    altitude_range = np.array([0.0, turbojet.design_altitude, 12000.0])
    mach_range     = np.array([0.0, 0.5, turbojet.design_mach_number])
    deck_full = generate_turbojet_deck(turbojet, altitude_range, mach_range)
    deck_part = generate_turbojet_deck(turbojet, altitude_range, mach_range,
                                        combustor_exit_temperature=0.9 * reference_point.Tt4)
    deck = Data()
    for field in ['altitude_m', 'mach_number', 'thrust_N', 'fuel_mass_flow_rate', 'isa_deviation_k', 'rating_code']:
        deck[field] = np.concatenate([deck_full[field], deck_part[field]])

    surrogate = Turbofan_Surrogate().build(deck=deck)
    F_tabulated, _ = surrogate.query(deck.altitude_m[0], deck.mach_number[0])
    check('deck: query() reproduces its own tabulated point [N]', F_tabulated[0], deck.thrust_N[0], 1e-4, results)

    with tempfile.TemporaryDirectory() as tmp_dir:
        deck_xlsx = Path(tmp_dir) / "turbojet_deck.xlsx"
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

    assert all_pass, "One or more quantities exceeded tolerance across the turbojet performance pathways"
    print('\nTurbojet analytical, off-design matching, and deck generation are mutually and self consistent within tolerance.')


if __name__ == '__main__':
    main()
