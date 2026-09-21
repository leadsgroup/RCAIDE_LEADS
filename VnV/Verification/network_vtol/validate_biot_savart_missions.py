# validate_biot_savart_missions.py
#
# End-to-end validation: runs real isolated mission segments TWICE each -- once with the
# Numba fast path forced off (pure numpy, the pre-existing behavior), once with it on
# (the new default when numba is installed) -- and compares the converged results
# bit-for-bit. Exercises the actual threaded free_wake pipeline (concurrent calls into
# the kernel from ThreadPoolExecutor), not just the isolated kernel like
# test_biot_savart_bitexact.py already covers.
#
# Segments chosen to span the geometrically distinct regimes:
#   Vertical_Climb  -- hover: near-axisymmetric wake spiral, nearly axial induced flow
#   climb_1         -- transition/edgewise: skewed, distorted wake geometry
#   cruise          -- forward flight: high advance ratio, mostly-planar wake
#
# Usage: python validate_biot_savart_missions.py [segment_tag ...]
#   (default: all three above)

import os
import sys
import time
import numpy as np

import importlib
import RCAIDE
# Plain "import ...biot_savart_velocity_induction as bs_module" binds the FUNCTION, not the
# module -- Lifting_Line_Theory/__init__.py does "from .biot_savart_velocity_induction import
# biot_savart_velocity_induction", which shadows the package's own submodule attribute of the
# same name. importlib.import_module bypasses that and gets the actual module object.
bs_module = importlib.import_module(
    "RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.biot_savart_velocity_induction")

base_dir = os.path.dirname(os.path.abspath(__file__))
vehicles_path = os.path.abspath(os.path.join(base_dir, "..", "..", "Vehicles"))
if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
sys.path.insert(0, base_dir)

from vtol_aircraft_test import TR_vehicle_setup, TR_configs_setup, TR_analyses_setup, TR_mission_setup, missions_setup

SEGMENTS = sys.argv[1:] if len(sys.argv) > 1 else ["Vertical_Climb", "climb_1", "cruise"]


def run_segment(tag, force_numba_off):
    original = bs_module._NUMBA_AVAILABLE
    bs_module._NUMBA_AVAILABLE = (not force_numba_off) and original
    try:
        vehicle  = TR_vehicle_setup(redesign_rotors=False)
        configs  = TR_configs_setup(vehicle)
        analyses = TR_analyses_setup(configs)
        mission_full = TR_mission_setup(analyses)

        segment = None
        for _, seg in mission_full.segments.items():
            if seg.tag == tag:
                segment = seg
                break
        if segment is None:
            available = [s.tag for s in mission_full.segments.values()]
            raise ValueError(f"'{tag}' not found. Available: {available}")

        single_mission = RCAIDE.Framework.Mission.Sequential_Segments()
        single_mission.append_segment(segment)
        missions = missions_setup(single_mission)

        ti = time.time()
        results = missions.base_mission.evaluate()
        elapsed = time.time() - ti
        seg_result = list(results.segments.values())[0]
        return seg_result, elapsed
    finally:
        bs_module._NUMBA_AVAILABLE = original


def compare(tag, seg_numpy, seg_numba):
    print(f"\n=== '{tag}' comparison ===", flush=True)
    converters = seg_numpy.conditions.energy.converters
    converters_nb = seg_numba.conditions.energy.converters

    all_exact = True
    for rotor_tag in converters.keys():
        c_np = converters[rotor_tag]
        c_nb = converters_nb[rotor_tag]
        for field in ['throttle', 'rpm', 'thrust', 'power', 'omega']:
            if field not in c_np:
                continue
            v_np = np.asarray(c_np[field], dtype=float)
            v_nb = np.asarray(c_nb[field], dtype=float)
            exact = np.array_equal(v_np, v_nb, equal_nan=True)
            max_err = np.nanmax(np.abs(v_np - v_nb)) if not exact else 0.0
            all_exact = all_exact and exact
            flag = "OK" if exact else f"DIFF (max_abs_err={max_err:.3e})"
            print(f"  {rotor_tag}.{field}: {flag}", flush=True)

        # Converged wake geometry -- the actual output of the free_wake pipeline this
        # whole change targets.
        rotor_np = None
        for prop in seg_numpy.analyses.vehicle.networks.electric.propulsors.values():
            if prop.rotor.tag == rotor_tag:
                rotor_np = prop.rotor
        rotor_nb = None
        for prop in seg_numba.analyses.vehicle.networks.electric.propulsors.values():
            if prop.rotor.tag == rotor_tag:
                rotor_nb = prop.rotor
        wake_np = getattr(getattr(rotor_np, 'blades', None), 'wake', {}).get('nodes_body', None) if rotor_np else None
        wake_nb = getattr(getattr(rotor_nb, 'blades', None), 'wake', {}).get('nodes_body', None) if rotor_nb else None
        if wake_np is not None and wake_nb is not None:
            exact = np.array_equal(np.asarray(wake_np), np.asarray(wake_nb), equal_nan=True)
            max_err = np.nanmax(np.abs(np.asarray(wake_np) - np.asarray(wake_nb))) if not exact else 0.0
            all_exact = all_exact and exact
            flag = "OK" if exact else f"DIFF (max_abs_err={max_err:.3e})"
            print(f"  {rotor_tag}.wake.nodes_body: {flag}", flush=True)

    print(f"  converged: numpy={bool(seg_numpy.converged)}, numba={bool(seg_numba.converged)}", flush=True)
    return all_exact


if __name__ == '__main__':
    assert bs_module._NUMBA_AVAILABLE, "numba not installed in this environment -- nothing to validate"

    overall_ok = True
    for tag in SEGMENTS:
        print(f"\n########## Segment '{tag}': numpy path ##########", flush=True)
        seg_numpy, t_numpy = run_segment(tag, force_numba_off=True)
        print(f"'{tag}' numpy path: {t_numpy:.1f}s", flush=True)

        print(f"\n########## Segment '{tag}': numba path ##########", flush=True)
        seg_numba, t_numba = run_segment(tag, force_numba_off=False)
        print(f"'{tag}' numba path: {t_numba:.1f}s  (speedup {t_numpy/t_numba:.2f}x)", flush=True)

        ok = compare(tag, seg_numpy, seg_numba)
        overall_ok = overall_ok and ok
        print(f"'{tag}': {'BIT-EXACT' if ok else 'MISMATCH -- investigate before trusting the fast path'}", flush=True)

    print()
    print("ALL SEGMENTS BIT-EXACT" if overall_ok else "SOME SEGMENTS MISMATCHED")
    sys.exit(0 if overall_ok else 1)
