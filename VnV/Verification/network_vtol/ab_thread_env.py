# ab_thread_env.py -- clean (no cProfile/tracemalloc overhead) wall-time A/B test:
# does constraining BLAS's own internal thread count avoid oversubscription against
# free_wake's ThreadPoolExecutor? Same math, same results either way -- purely an
# environment/scheduling question, zero physics risk.
import os
import sys
import time

import RCAIDE

base_dir = os.path.dirname(os.path.abspath(__file__))
vehicles_path = os.path.abspath(os.path.join(base_dir, "..", "..", "Vehicles"))
if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
sys.path.insert(0, base_dir)

from vtol_aircraft_test import TR_vehicle_setup, TR_configs_setup, TR_analyses_setup, TR_mission_setup, missions_setup

SEGMENT_TAG = sys.argv[1] if len(sys.argv) > 1 else "cruise"

def run_once():
    vehicle  = TR_vehicle_setup(redesign_rotors=False)
    configs  = TR_configs_setup(vehicle)
    analyses = TR_analyses_setup(configs)
    mission_full = TR_mission_setup(analyses)

    segment = None
    for _, seg in mission_full.segments.items():
        if seg.tag == SEGMENT_TAG:
            segment = seg
            break

    single_mission = RCAIDE.Framework.Mission.Sequential_Segments()
    single_mission.append_segment(segment)
    missions = missions_setup(single_mission)

    ti = time.time()
    results = missions.base_mission.evaluate()
    elapsed = time.time() - ti
    seg_result = list(results.segments.values())[0]
    print(f"[{SEGMENT_TAG}] wall time: {elapsed:.1f}s, converged={bool(seg_result.converged)}", flush=True)
    return elapsed

if __name__ == '__main__':
    env_vars = ['OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'NUMEXPR_NUM_THREADS']
    print("Thread-limiting env vars:", {v: os.environ.get(v, '<unset>') for v in env_vars}, flush=True)
    run_once()
