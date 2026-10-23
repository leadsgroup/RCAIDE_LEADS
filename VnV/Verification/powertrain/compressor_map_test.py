# compressor_map_test.py
#
# Verifies Generic_Compressor_Map on its own terms (no engine/network involved):
# design-point reproduction, scaling with/without an explicit design efficiency,
# both extrapolation branches (below 60% / above 110% corrected speed) including
# the adiabatic-efficiency floor and the pressure-ratio-below-1 floor, and the
# query_by_temperature_ratio() inverse lookup (round trip, plus its out-of-range
# error path).
#
# Created: Sep 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Library.Methods.Powertrain.Converters.Compressor.Generic_Compressor_Map import Generic_Compressor_Map

import numpy as np


def check(name, computed, truth, tol, results):
    error = abs(float(computed) - truth) / abs(truth)
    results.append((name, float(computed), truth, error, tol))
    return error


def main():
    results = []
    generic = Generic_Compressor_Map()

    # ------------------------------------------------------------------------------------
    # 1. Design point (100% corrected speed) reproduces the tabulated row exactly
    # ------------------------------------------------------------------------------------
    pressure_ratio, mass_flow, efficiency = generic.query(100.0)
    check('design point: percent pressure ratio [%]', pressure_ratio[0], 100.0, 1e-6, results)
    check('design point: percent mass flow [%]',       mass_flow[0],      100.0, 1e-6, results)
    check('design point: adiabatic efficiency',        efficiency[0],     0.870, 1e-6, results)

    # ------------------------------------------------------------------------------------
    # 2. scale_to_design_point: pressure ratio always rescaled, efficiency only if given
    # ------------------------------------------------------------------------------------
    scaled_pr_only = generic.scale_to_design_point(design_pressure_ratio=20.0)
    pressure_ratio, _, efficiency = scaled_pr_only.query(100.0)
    check('scaled (pressure ratio only): absolute pressure ratio', pressure_ratio[0], 20.0, 1e-6, results)
    check('scaled (pressure ratio only): efficiency unchanged',    efficiency[0],     0.870, 1e-6, results)

    scaled_pr_and_eta = generic.scale_to_design_point(design_pressure_ratio=20.0, design_adiabatic_efficiency=0.90)
    pressure_ratio, _, efficiency = scaled_pr_and_eta.query(100.0)
    check('scaled (pressure ratio + efficiency): absolute pressure ratio', pressure_ratio[0], 20.0, 1e-6, results)
    check('scaled (pressure ratio + efficiency): peak efficiency',         efficiency[0],     0.90, 1e-6, results)

    # ------------------------------------------------------------------------------------
    # 3. Extrapolation below the tabulated range (60%): efficiency floor, mass-flow floor,
    #    log-space pressure-ratio extrapolation staying positive, and pressure-ratio-below-1 floor
    # ------------------------------------------------------------------------------------
    pressure_ratio, mass_flow, efficiency = scaled_pr_only.query(10.0)
    assert efficiency[0] == 0.30, f"expected the 0.30 efficiency floor at deep part-power, got {efficiency[0]}"
    assert mass_flow[0] >= 0.0, "corrected mass flow must not go negative"
    assert pressure_ratio[0] >= 1.0, "a compressor cannot have pressure ratio below 1"

    # ------------------------------------------------------------------------------------
    # 4. Extrapolation above the tabulated range (110%)
    # ------------------------------------------------------------------------------------
    pressure_ratio, mass_flow, efficiency = scaled_pr_only.query(130.0)
    assert pressure_ratio[0] > 20.0, "pressure ratio should keep rising with corrected speed above 110%"
    assert efficiency[0] > 0.0

    # ------------------------------------------------------------------------------------
    # 5. query_by_temperature_ratio: round trip, and the out-of-range error path
    # ------------------------------------------------------------------------------------
    gamma = 1.4
    target_speed = 85.0
    pressure_ratio, _, efficiency = scaled_pr_only.query(target_speed)
    temperature_ratio_target = 1 + (pressure_ratio[0]**((gamma - 1) / gamma) - 1) / efficiency[0]

    found_speed, found_pr, found_eta = scaled_pr_only.query_by_temperature_ratio(temperature_ratio_target, gamma)
    check('query_by_temperature_ratio: recovers the corrected speed [%]', found_speed, target_speed, 1e-4, results)
    check('query_by_temperature_ratio: recovers the pressure ratio',      found_pr,     pressure_ratio[0], 1e-4, results)
    check('query_by_temperature_ratio: recovers the efficiency',          found_eta,    efficiency[0],     1e-4, results)

    try:
        scaled_pr_only.query_by_temperature_ratio(1e6, gamma)
        raise AssertionError("expected ValueError for an unreachable temperature ratio")
    except ValueError:
        pass

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

    assert all_pass, "One or more quantities exceeded tolerance in Generic_Compressor_Map"
    print('\nGeneric_Compressor_Map matches its own design point and round-trips through query_by_temperature_ratio.')


if __name__ == '__main__':
    main()
