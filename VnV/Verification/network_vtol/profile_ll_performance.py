# profile_ll_performance.py
#
# Time (cProfile) + memory (tracemalloc) profile of one isolated mission segment,
# to find which part of the Lifting_Line_Theory free_wake pipeline dominates cost.
# Reuses TR_*_setup from vtol_aircraft_test.py so segment definitions (bounds,
# guesses, solver settings) are the single source of truth -- same pattern as
# generate_segment_wakes.py.
#
# Usage:
#   python profile_ll_performance.py [segment_tag]
#   (default segment_tag: "cruise" -- converges reliably, moderate control-point
#   count, exercises the full free_wake relaxation loop without the multi-hour
#   risk of a transition/near-hover segment)
#
# Outputs:
#   - profile_<tag>.prof  (raw cProfile stats, open with snakeviz/pstats for a
#     full interactive view: `snakeviz profile_<tag>.prof`)
#   - printed top-30 by cumulative time, top-30 by internal (self) time, and a
#     view filtered to just RCAIDE's own Lifting_Line_Theory files
#   - printed top-30 net memory allocations (by source line) and top-20 by file

import os
import sys
import time
import cProfile
import pstats
import tracemalloc

import RCAIDE

base_dir = os.path.dirname(os.path.abspath(__file__))
vehicles_path = os.path.abspath(os.path.join(base_dir, "..", "..", "Vehicles"))
if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)
sys.path.insert(0, base_dir)

from vtol_aircraft_test import TR_vehicle_setup, TR_configs_setup, TR_analyses_setup, TR_mission_setup, missions_setup

SEGMENT_TAG = sys.argv[1] if len(sys.argv) > 1 else "cruise"


def build_single_segment_mission():
    vehicle  = TR_vehicle_setup(redesign_rotors=False)
    configs  = TR_configs_setup(vehicle)
    analyses = TR_analyses_setup(configs)
    mission_full = TR_mission_setup(analyses)

    segment = None
    for _, seg in mission_full.segments.items():
        if seg.tag == SEGMENT_TAG:
            segment = seg
            break
    if segment is None:
        available = [seg.tag for seg in mission_full.segments.values()]
        raise ValueError(f"segment tag '{SEGMENT_TAG}' not found. Available: {available}")

    single_mission = RCAIDE.Framework.Mission.Sequential_Segments()
    single_mission.append_segment(segment)
    return missions_setup(single_mission)


def run_time_profile():
    print(f"\n=== cProfile (time) on segment '{SEGMENT_TAG}' ===", flush=True)
    missions = build_single_segment_mission()

    profiler = cProfile.Profile()
    ti = time.time()
    profiler.enable()
    results = missions.base_mission.evaluate()
    profiler.disable()
    elapsed = time.time() - ti

    seg_result = list(results.segments.values())[0]
    print(f"Wall time: {elapsed:.1f}s, converged={bool(seg_result.converged)}", flush=True)

    prof_path = os.path.join(base_dir, f"profile_{SEGMENT_TAG}.prof")
    profiler.dump_stats(prof_path)
    print(f"Saved raw profile -> {prof_path} (open with `snakeviz {prof_path}` for an interactive view)")

    stats = pstats.Stats(profiler)
    stats.sort_stats('cumulative')
    print("\n--- Top 30 by CUMULATIVE time (includes time in callees) ---")
    stats.print_stats(30)

    stats.sort_stats('tottime')
    print("\n--- Top 30 by INTERNAL (self) time (excludes callees -- the real hotspots) ---")
    stats.print_stats(30)

    print("\n--- Filtered to RCAIDE's own Lifting_Line_Theory files, by internal time ---")
    stats.print_stats(r'Lifting_Line_Theory', 30)


def run_memory_profile():
    print(f"\n=== tracemalloc (memory) on segment '{SEGMENT_TAG}' ===", flush=True)
    missions = build_single_segment_mission()

    tracemalloc.start()
    snapshot_before = tracemalloc.take_snapshot()
    missions.base_mission.evaluate()
    snapshot_after = tracemalloc.take_snapshot()
    current, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    print(f"Current traced: {current/1e6:.1f} MB, Peak traced: {peak/1e6:.1f} MB", flush=True)

    top_by_line = snapshot_after.compare_to(snapshot_before, 'lineno')
    print("\n--- Top 30 net allocations by source line ---")
    for stat in top_by_line[:30]:
        print(stat)

    top_by_file = snapshot_after.compare_to(snapshot_before, 'filename')
    print("\n--- Top 20 net allocations by file ---")
    for stat in top_by_file[:20]:
        print(stat)


if __name__ == '__main__':
    run_time_profile()
    run_memory_profile()
