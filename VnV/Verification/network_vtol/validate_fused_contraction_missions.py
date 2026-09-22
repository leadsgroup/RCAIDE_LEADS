# validate_fused_contraction_missions.py
#
# Mission-level bit-exactness check for the biot_savart_induced_velocity fusion in
# free_wake.py (replaces biot_savart_velocity_induction + np.einsum('mnk,n->mk', ...) with
# one fused Numba pass). Runs a real free_wake=True segment twice -- once with the current
# (fused) free_wake.py, once with the pre-fusion two-step version restored from git -- and
# compares converged rotor conditions + wake geometry bit-for-bit (NaN-aware).

import os
import sys
import time
import subprocess
import numpy as np

base_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.abspath(os.path.join(base_dir, "..", "..", ".."))
vehicles_path = os.path.abspath(os.path.join(base_dir, "..", "..", "Vehicles"))
FREE_WAKE_PATH = os.path.join(repo_root, "RCAIDE", "Library", "Methods", "Powertrain",
                               "Converters", "Rotor", "Performance", "Lifting_Line_Theory", "free_wake.py")

SEGMENTS = sys.argv[1:] if len(sys.argv) > 1 else ["Vertical_Climb"]


def run_segment_subprocess(tag, use_old_free_wake):
    """Runs the segment evaluation in a FRESH subprocess so swapping free_wake.py's source
    on disk before each run is guaranteed to take effect (avoids any module-caching issues
    with in-process file swaps + reload)."""
    script = f"""
import sys, os
import numpy as np
sys.path.insert(0, r"{repo_root}")
sys.path.insert(0, r"{vehicles_path}")
import RCAIDE
from vtol_aircraft_test import TR_vehicle_setup, TR_configs_setup, TR_analyses_setup, TR_mission_setup, missions_setup
import pickle

vehicle  = TR_vehicle_setup(redesign_rotors=False)
configs  = TR_configs_setup(vehicle)
analyses = TR_analyses_setup(configs)
mission_full = TR_mission_setup(analyses)

segment = None
for _, seg in mission_full.segments.items():
    if seg.tag == "{tag}":
        segment = seg
        break
if segment is None:
    available = [s.tag for s in mission_full.segments.values()]
    raise ValueError(f"'{tag}' not found. Available: {{available}}")

single_mission = RCAIDE.Framework.Mission.Sequential_Segments()
single_mission.append_segment(segment)
missions = missions_setup(single_mission)
results = missions.base_mission.evaluate()
seg_result = list(results.segments.values())[0]

out = {{}}
for rotor_tag, conv in seg_result.conditions.energy.converters.items():
    out[rotor_tag] = {{}}
    for field in ['throttle', 'rpm', 'thrust', 'torque', 'omega']:
        if field in conv:
            out[rotor_tag][field] = np.asarray(conv[field])

for prop in seg_result.analyses.vehicle.networks.electric.propulsors.values():
    rt = prop.rotor.tag
    wake = getattr(getattr(prop.rotor, 'blades', None), 'wake', None)
    if wake is not None and wake.get('nodes_body', None) is not None:
        out.setdefault(rt, {{}})['wake_nodes_body'] = np.asarray(wake.nodes_body)

out['converged'] = bool(seg_result.converged)
with open(r"{os.path.join(base_dir, '_seg_result.pkl')}", 'wb') as f:
    pickle.dump(out, f)
"""
    script_path = os.path.join(base_dir, "_run_one_segment.py")
    with open(script_path, "w") as f:
        f.write(script)

    venv_python = os.path.join(repo_root, ".venv", "Scripts", "python.exe")
    ti = time.time()
    result = subprocess.run([venv_python, script_path], cwd=base_dir, capture_output=True, text=True)
    elapsed = time.time() - ti
    if result.returncode != 0:
        print(result.stdout[-3000:])
        print(result.stderr[-3000:])
        raise RuntimeError(f"subprocess failed for tag={tag} use_old_free_wake={use_old_free_wake}")

    import pickle
    with open(os.path.join(base_dir, "_seg_result.pkl"), "rb") as f:
        out = pickle.load(f)
    return out, elapsed


def compare(tag, res_new, res_old):
    print(f"\n=== '{tag}' comparison (fused vs pre-fusion) ===", flush=True)
    all_exact = True
    for rotor_tag in res_new.keys():
        if rotor_tag == 'converged':
            continue
        for field, v_new in res_new[rotor_tag].items():
            v_old = res_old[rotor_tag][field]
            exact = np.array_equal(v_new, v_old, equal_nan=True)
            max_err = np.nanmax(np.abs(v_new.astype(float) - v_old.astype(float))) if not exact else 0.0
            all_exact = all_exact and exact
            flag = "OK" if exact else f"DIFF (max_abs_err={max_err:.3e})"
            print(f"  {rotor_tag}.{field}: {flag}", flush=True)
    print(f"  converged: new={res_new['converged']}, old={res_old['converged']}", flush=True)
    return all_exact


if __name__ == '__main__':
    # Back up current (fused) free_wake.py, fetch pre-fusion version from git HEAD.
    with open(FREE_WAKE_PATH, "r") as f:
        fused_source = f.read()

    old_source = subprocess.run(
        ["git", "show", "HEAD:RCAIDE/Library/Methods/Powertrain/Converters/Rotor/Performance/Lifting_Line_Theory/free_wake.py"],
        cwd=repo_root, capture_output=True, text=True, check=True
    ).stdout

    overall_ok = True
    for tag in SEGMENTS:
        # NEW (fused) -- current working-tree file already in place.
        with open(FREE_WAKE_PATH, "w") as f:
            f.write(fused_source)
        print(f"\n########## '{tag}': fused (current) ##########", flush=True)
        res_new, t_new = run_segment_subprocess(tag, use_old_free_wake=False)
        print(f"'{tag}' fused: {t_new:.1f}s, converged={res_new['converged']}", flush=True)

        # OLD (pre-fusion) -- restore from git HEAD.
        with open(FREE_WAKE_PATH, "w") as f:
            f.write(old_source)
        print(f"\n########## '{tag}': pre-fusion (HEAD) ##########", flush=True)
        res_old, t_old = run_segment_subprocess(tag, use_old_free_wake=True)
        print(f"'{tag}' pre-fusion: {t_old:.1f}s, converged={res_old['converged']}", flush=True)

        # Restore fused version before comparing/exiting either way.
        with open(FREE_WAKE_PATH, "w") as f:
            f.write(fused_source)

        ok = compare(tag, res_new, res_old)
        overall_ok = overall_ok and ok
        print(f"'{tag}': {'BIT-EXACT' if ok else 'MISMATCH'}  (speedup {t_old/t_new:.2f}x)", flush=True)

    print()
    print("ALL SEGMENTS BIT-EXACT" if overall_ok else "SOME SEGMENTS MISMATCHED")
    sys.exit(0 if overall_ok else 1)
