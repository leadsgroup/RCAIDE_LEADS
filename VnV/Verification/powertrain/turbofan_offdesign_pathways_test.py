# turbofan_offdesign_pathways_test.py
#
# Verifies RCAIDE's three turbofan performance pathways -- analytical cycle
# model, live off-design matching (turbofan.offdesign_matching), and the
# table-driven surrogate (turbofan.surrogate) -- at design point, sea-level
# static, and an interpolated surrogate point. Reuses the literature-
# validated GE90-94B from test_turbofan_validation.py as the test engine.
#
# Not asserted, and why (both noted again inline): analytical vs. off-design
# matching thrust are not cross-compared at SLS -- the analytical model's
# fixed pressure ratios are known to diverge far from its one design point
# (see ENGINE_MODEL_NOTES.md), so each pathway is checked against its own
# reference instead. The analytical model is also not run at exact M0=0, a
# known singularity RCAIDE itself works around via M0=0.01 in
# design_turbofan.py; off-design matching has no such singularity and is
# checked at exact M0=0 instead.
#
# Created: Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core             import Data, Units
from RCAIDE.Library.Methods.Powertrain import setup_operating_conditions
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.design_turbofan_offdesign_matching import design_turbofan_offdesign_matching
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.generate_turbofan_offdesign_deck    import generate_turbofan_offdesign_deck
from RCAIDE.Library.Methods.Powertrain.Propulsors.Turbofan.Turbofan_Surrogate                  import Turbofan_Surrogate

# reuse the literature-validated GE90-94B definition rather than redefining a test engine
from VnV.Validation.propulsors.test_turbofan_validation import GE90_94B

# Python package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  Helpers
# ----------------------------------------------------------------------------------------------------------------------
def check(name, computed, truth, tol, results):
    error = abs(float(computed) - truth) / abs(truth)
    results.append((name, float(computed), truth, error, tol))
    return error


def evaluate_thrust(turbofan, fuel_line, altitude, mach_number, throttle=1.0):
    """Runs turbofan.compute_performance at one flight condition through
    whichever pathway is currently wired (offdesign_matching / surrogate /
    analytical, checked in that dispatch order by
    compute_turbofan_performance) and returns net thrust [N]."""
    atmosphere     = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    speed_of_sound = float(np.ravel(atmosphere.compute_values(altitude).speed_of_sound)[0])
    state = setup_operating_conditions(turbofan, fuel_line, velocity_range=np.array([speed_of_sound * mach_number]),
                                        altitude=altitude)
    state.conditions.energy.propulsors[turbofan.tag].throttle[:, 0] = throttle
    turbofan.compute_performance(state, fuel_line)
    return float(state.conditions.energy.propulsors[turbofan.tag].thrust[0, 0])


def main():
    turbofan  = GE90_94B()   # already design_turbofan()-ed
    fuel_line = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()

    results = []

    # ------------------------------------------------------------------------------------
    # 1. Design point: analytical and off-design matching both vs. turbofan.design_thrust
    # ------------------------------------------------------------------------------------
    design_thrust = turbofan.design_thrust

    F_analytical_design = evaluate_thrust(turbofan, fuel_line, turbofan.design_altitude, turbofan.design_mach_number)
    check('design point, analytical vs design_thrust [N]', F_analytical_design, design_thrust, 1e-6, results)

    # built with offdesign_matching still None -- it calls compute_performance() itself and
    # would recurse into this same solver otherwise
    design_constants, reference_point = design_turbofan_offdesign_matching(turbofan)
    turbofan.offdesign_matching = Data(design_constants=design_constants, reference_point=reference_point)

    F_offdesign_design = evaluate_thrust(turbofan, fuel_line, turbofan.design_altitude, turbofan.design_mach_number)
    # ~2% gap: one representative gamma_c vs RCAIDE's temperature-dependent gas model
    check('design point, off-design matching vs design_thrust [N]', F_offdesign_design, design_thrust, 5e-2, results)

    # ------------------------------------------------------------------------------------
    # 2. Sea-level static
    # ------------------------------------------------------------------------------------
    turbofan.offdesign_matching = None
    F_analytical_sls = evaluate_thrust(turbofan, fuel_line, 0.0, 0.01)
    check('SLS (M=0.01), analytical vs turbofan.sealevel_static_thrust [N]',
          F_analytical_sls, turbofan.sealevel_static_thrust, 1e-6, results)

    F_analytical_sls_exact0 = evaluate_thrust(turbofan, fuel_line, 0.0, 0.0)
    # known singularity in the analytical model at exact M0=0 (see header comment)
    assert np.isnan(F_analytical_sls_exact0), \
        "expected NaN from the analytical model's known M0=0 singularity -- got a finite value"

    turbofan.offdesign_matching = Data(design_constants=design_constants, reference_point=reference_point)
    F_offdesign_sls_001  = evaluate_thrust(turbofan, fuel_line, 0.0, 0.01)
    F_offdesign_sls_exact0 = evaluate_thrust(turbofan, fuel_line, 0.0, 0.0)

    # off-design matching handles exact M0=0 cleanly (ENGINE_MODEL_NOTES.md Sec. 10.8)
    assert F_offdesign_sls_exact0 > 0
    check('SLS: off-design matching, M=0 vs M=0.01 continuity', F_offdesign_sls_exact0, F_offdesign_sls_001, 5e-2, results)

    # physical sanity bound: SLS thrust > design-point cruise thrust for a high-bypass turbofan
    assert F_offdesign_sls_exact0 > F_offdesign_design, \
        f"off-design SLS thrust ({F_offdesign_sls_exact0:.0f} N) should exceed design-point " \
        f"cruise thrust ({F_offdesign_design:.0f} N) for a high-bypass turbofan"

    # ------------------------------------------------------------------------------------
    # 3. Surrogate: deck generation -> build -> query round-trip, plus dispatch wiring
    # ------------------------------------------------------------------------------------
    turbofan.offdesign_matching = None

    altitude_range = np.array([0.0, turbofan.design_altitude, 12000.0])
    mach_range     = np.array([0.0, 0.3, turbofan.design_mach_number])
    # two throttle settings per RC=0 point -- Turbofan_Surrogate needs >=2 rows per
    # (ALT ft, XM, ISA k) group to build its throttle axis
    deck_full = generate_turbofan_offdesign_deck(turbofan, altitude_range, mach_range)
    deck_part = generate_turbofan_offdesign_deck(turbofan, altitude_range, mach_range,
                                                  combustor_exit_temperature=0.9 * reference_point.Tt4)
    deck = Data()
    for field in ['altitude_m', 'mach_number', 'thrust_N', 'fuel_mass_flow_rate', 'isa_deviation_k', 'rating_code']:
        deck[field] = np.concatenate([deck_full[field], deck_part[field]])

    turbofan.surrogate = Turbofan_Surrogate().build(deck=deck)

    # exact reproduction of a tabulated grid point
    F_tabulated, _ = turbofan.surrogate.query(deck.altitude_m[0], deck.mach_number[0])
    check('surrogate: query() reproduces its own tabulated point [N]',
          F_tabulated[0], deck.thrust_N[0], 1e-4, results)

    # interpolated point should fall between its bracketing tabulated altitudes
    F_low_alt, _  = turbofan.surrogate.query(np.array([0.0]), np.array([0.3]))
    F_high_alt, _ = turbofan.surrogate.query(np.array([turbofan.design_altitude]), np.array([0.3]))
    F_mid_alt, _  = turbofan.surrogate.query(np.array([17500 * Units.ft]), np.array([0.3]))
    assert F_high_alt[0] < F_mid_alt[0] < F_low_alt[0], \
        f"interpolated thrust ({F_mid_alt[0]:.0f} N at 17,500 ft) should fall strictly between " \
        f"the bracketing tabulated altitudes ({F_high_alt[0]:.0f} N at design altitude, " \
        f"{F_low_alt[0]:.0f} N at sea level)"

    # full dispatch through compute_turbofan_performance should match a direct query() call
    # (this caught compute_turbofan_performance_surrogate.py rescaling every output against
    # turbofan.design_thrust instead of the deck's own SLS reference -- now fixed)
    F_via_dispatch = evaluate_thrust(turbofan, fuel_line, deck.altitude_m[0], deck.mach_number[0])
    check('surrogate: compute_performance dispatch vs direct query() [N]',
          F_via_dispatch, F_tabulated[0], 1e-8, results)

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

    assert all_pass, "One or more quantities exceeded tolerance across the three turbofan performance pathways"
    print('\nAll three turbofan performance pathways (analytical, off-design matching, surrogate) '
          'are mutually and self consistent within tolerance.')


if __name__ == '__main__':
    main()
