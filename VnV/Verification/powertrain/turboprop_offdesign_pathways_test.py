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


def evaluate_performance(turboprop, fuel_line, altitude, mach_number, throttle=1.0):
    atmosphere     = RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976()
    speed_of_sound = float(np.ravel(atmosphere.compute_values(altitude).speed_of_sound)[0])
    state = setup_operating_conditions(turboprop, fuel_line, velocity_range=np.array([speed_of_sound * mach_number]),
                                        altitude=altitude)
    state.conditions.energy.propulsors[turboprop.tag].throttle[:, 0] = throttle
    _, outputs, _, _ = turboprop.compute_performance(state, fuel_line)
    return outputs


def evaluate_thrust(turboprop, fuel_line, altitude, mach_number, throttle=1.0):
    return float(evaluate_performance(turboprop, fuel_line, altitude, mach_number, throttle).thrust[0, 0])


def main():
    turboprop = atr_72_vehicle_setup().networks.fuel.propulsors[ENGINE_TAG]   # already design_turboprop()-ed, sized to its rated takeoff power
    fuel_line = RCAIDE.Library.Components.Powertrain.Distributors.Fuel_Line()
    # design_turboprop() stores this as a (1,1) array (derived from design_freestream_velocity), unlike
    # Turbofan/Turbojet's plain float -- flatten once here rather than at every call site below
    turboprop.design_mach_number = float(np.ravel(turboprop.design_mach_number)[0])

    results = []

    # ------------------------------------------------------------------------------------
    # 1. Design point: analytical and off-design matching both vs. turboprop.design_thrust
    # ------------------------------------------------------------------------------------
    design_thrust = turboprop.design_thrust

    # sized to the rated takeoff power: sea-level static shaft power at the takeoff rating, through the
    # off-design matching model design_turboprop attaches by default
    P_sls = evaluate_performance(turboprop, fuel_line, 0.0, 0.01, turboprop.rated_takeoff_throttle).power.mechanical[0, 0]
    check('sea-level static shaft power at the takeoff rating vs rated_takeoff_power [W]', P_sls, turboprop.rated_takeoff_power, 1e-5, results)

    # analytical cycle model: the design point it was sized at
    offdesign_matching           = turboprop.offdesign_matching
    turboprop.offdesign_matching = None
    F_analytical_design = evaluate_thrust(turboprop, fuel_line, turboprop.design_altitude, turboprop.design_mach_number)
    check('design point, analytical vs design_thrust [N]', F_analytical_design, design_thrust, 1e-6, results)

    design_constants, reference_point = design_turboprop_offdesign_matching(turboprop)
    turboprop.offdesign_matching = Data(design_constants=design_constants, reference_point=reference_point)

    F_offdesign_design = evaluate_thrust(turboprop, fuel_line, turboprop.design_altitude, turboprop.design_mach_number)
    check('design point, off-design matching vs design_thrust [N]', F_offdesign_design, design_thrust, 5e-2, results)

    # ------------------------------------------------------------------------------------
    # 2. Near-static behavior
    # ------------------------------------------------------------------------------------
    # The propeller is an actuator disk (compute_actuator_disk_propeller_thrust), so thrust stays finite
    # at zero speed. Sea-level static thrust at the takeoff rating: the matching model's shaft power
    # through the actuator-disk static relation T = (2 rho A)^(1/3) (eta_pc P)^(2/3), plus the core jet,
    # which is small at static conditions -- checked to 2%. And thrust falls monotonically with speed.
    turboprop.offdesign_matching = offdesign_matching
    rho_sl       = float(np.ravel(RCAIDE.Framework.Analyses.Atmospheric.US_Standard_1976().compute_values(0.0).density)[0])
    disk_area    = np.pi * turboprop.propeller.tip_radius ** 2
    ideal_static = (2 * rho_sl * disk_area) ** (1 / 3) * (turboprop.propeller_polytropic_efficiency * P_sls) ** (2 / 3)
    check('sea-level static thrust vs actuator-disk static propeller thrust [N]', turboprop.sealevel_static_thrust, ideal_static, 2e-2, results)
    thrust_vs_speed = [evaluate_thrust(turboprop, fuel_line, 0.0, mach_number, turboprop.rated_takeoff_throttle)
                       for mach_number in [0.01, 0.05, 0.1, 0.2, 0.3]]
    assert np.all(np.diff(thrust_vs_speed) < 0), f"sea-level thrust should fall with speed: {thrust_vs_speed}"

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
    # validate_deck()) -- this section only exercises the deck-generation/surrogate code paths
    # (round-trip self-consistency), not turboprop's near-static thrust physics (checked in section 2).
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
