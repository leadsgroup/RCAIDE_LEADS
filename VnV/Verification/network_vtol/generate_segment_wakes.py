# generate_segment_wakes.py
#
# Runs each of the 14 Tiltrotor mission segments (from TR_mission_setup) as its OWN
# isolated single-segment mission under the vehicle's currently-configured fidelity
# (set free_wake=True / fidelity='Lifting_Line_Theory' in Tiltrotor_EVTOL.py before
# running this for real), then saves the converged rotor.blades.wake.nodes_body for
# every rotor to VnV/Vehicles/wake_geometries/<segment_tag>.pkl.
#
# Why per-segment, not sliced out of the full-mission run: the corrected battery
# config changes mass/CG, so the wake shape per flight condition needs regenerating.
# Running each segment in isolation (rather than reusing the full-mission run) gives
# a clean, independently-reproducible reference wake per flight condition, not one
# entangled with whatever the previous segment in the chain happened to converge to.
#
# Does NOT modify vtol_aircraft_test.py's TR_mission_setup at all -- it builds the
# full mission exactly as-is (segment definitions are the single source of truth),
# then slices one segment back out into its own fresh Sequential_Segments mission.
# This avoids re-deriving/duplicating any of the carefully-tuned per-segment
# bounds/guesses/solver settings.

import os
import sys
import pickle
import time

import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots import plot_3d_vehicle

base_dir = os.path.dirname(os.path.abspath(__file__))
vehicles_path = os.path.abspath(os.path.join(base_dir, "..", "..", "Vehicles"))
if vehicles_path not in sys.path:
    sys.path.insert(0, vehicles_path)

sys.path.insert(0, base_dir)
from vtol_aircraft_test import TR_vehicle_setup, TR_configs_setup, TR_analyses_setup, TR_mission_setup, missions_setup

# Segment tag -> config (analyses/vehicle) tag it runs on, read directly off the
# `segment.analyses.extend(analyses.<config>)` calls in TR_mission_setup.
TAG_TO_CONFIG = {
    "Vertical_Climb":          "vertical_flight",
    "departure_transition_1":  "transition_setting_1",
    "departure_transition_2":  "transition_setting_2",
    "Departure_Pattern_Curve": "transition_setting_2",
    "climb_1":                 "cruise",
    "climb_2":                 "cruise",
    "cruise":                  "cruise",
    "descent_1":               "cruise",
    "descent_2":               "descent_cruise",
    "descent_3":               "cruise",
    "Approach_Pattern_Curve":  "transition_setting_2",
    "arriving_transition_1":   "low_speed_transition",
    "arriving_transition_2":   "low_speed_transition",
    "Vertical_Descent":        "vertical_flight",
}


def extract_wakes(vehicle_config, debug=False):
    """Pull rotor.blades.wake.nodes_body off every propulsor's rotor in a config."""
    wakes = {}
    for network in vehicle_config.networks.values():
        for prop_tag, propulsor in network.propulsors.items():
            rotor = getattr(propulsor, 'rotor', None)
            if rotor is None:
                if debug:
                    print(f"  [debug] {prop_tag}: no .rotor attribute")
                continue
            has_blades = 'blades' in rotor
            wake_obj   = rotor.blades.get('wake', None) if has_blades else None
            nodes_body = wake_obj.get('nodes_body', None) if wake_obj is not None else None
            if debug:
                print(f"  [debug] {prop_tag}: id(rotor)={id(rotor)}, fidelity={rotor.get('fidelity', '?')}, "
                      f"has_blades={has_blades}, wake_obj={'present' if wake_obj is not None else 'None'}, "
                      f"nodes_body={'shape='+str(nodes_body.shape) if nodes_body is not None else 'None'}")
            if nodes_body is not None:
                wakes[prop_tag] = nodes_body.copy()
    return wakes


def main(segments_to_run=None, pop_up_plot=False, wake_control_point=0, wake_tube_radius=0.03):
    vehicle  = TR_vehicle_setup(redesign_rotors=False)
    configs  = TR_configs_setup(vehicle)
    analyses = TR_analyses_setup(configs)

    mission_full = TR_mission_setup(analyses)

    out_dir = os.path.join(vehicles_path, 'wake_geometries')
    os.makedirs(out_dir, exist_ok=True)

    tags = segments_to_run if segments_to_run is not None else list(TAG_TO_CONFIG.keys())

    for key, segment in mission_full.segments.items():
        tag = segment.tag
        if tag not in tags:
            continue

        print(f"\n=== Running segment '{tag}' in isolation ===", flush=True)
        ti = time.time()

        single_mission = RCAIDE.Framework.Mission.Sequential_Segments()
        single_mission.append_segment(segment)
        missions = missions_setup(single_mission)

        results = missions.base_mission.evaluate()
        elapsed = (time.time() - ti) / 60

        seg_result   = results.segments[key]
        converged    = bool(seg_result.converged)
        config_name  = TAG_TO_CONFIG[tag]

        # IMPORTANT: hp_decomposition deep-copies the segment (and its .analyses.vehicle,
        # rotor objects included) once per piece -- configs[config_name] is the ORIGINAL,
        # never-mutated object for any segment that got hp-decomposed. The real converged
        # rotor lives on the solved segment's own .analyses.vehicle (which, for
        # non-decomposed segments, is the same object as configs[config_name] anyway,
        # since .extend() just copies the reference -- so this path is correct either way).
        solved_vehicle = getattr(seg_result.analyses, 'vehicle', None) or configs[config_name]
        wakes          = extract_wakes(solved_vehicle, debug=True)

        payload = {
            'segment_tag':  tag,
            'config_name':  config_name,
            'converged':    converged,
            'elapsed_min':  elapsed,
            'wake_inputs':  dict(configs[config_name].networks.electric.propulsors['front_port_propulsor'].rotor.wake_inputs),
            'wakes':        wakes,   # {propulsor_tag: nodes_body array, shape (ctrl_pts, N_wake+1, B, 3)}
        }

        out_path = os.path.join(out_dir, f"{tag}.pkl")
        with open(out_path, 'wb') as f:
            pickle.dump(payload, f)

        print(f"'{tag}': converged={converged}, elapsed={elapsed:.1f} min, saved -> {out_path}", flush=True)

        if pop_up_plot:
            # Interactive PyVista window -- rotate/zoom with the mouse, close it to
            # continue to the next segment (plotter.show() blocks until closed).
            # Uses solved_vehicle (see note above), not configs[config_name], so the
            # wake actually shows up for hp-decomposed segments too.
            plot_3d_vehicle(solved_vehicle,
                             plot_wake           = True,
                             wake_control_point   = wake_control_point,
                             wake_tube_radius     = wake_tube_radius,
                             show_figure          = True,
                             save_figure          = False)

    return


if __name__ == '__main__':
    # Quick smoke test on one cheap segment first -- pass tags on the command line to
    # run a subset, e.g.: python generate_segment_wakes.py cruise
    # Add --plot to pop up an interactive 3D wake view (mouse rotate/zoom) after each
    # segment, e.g.: python generate_segment_wakes.py Vertical_Climb --plot
    # Add --radius=<value> to override the default wake_tube_radius (0.03 m), e.g.:
    # python generate_segment_wakes.py Vertical_Climb --plot --radius=0.05
    args   = sys.argv[1:]
    pop_up = '--plot' in args
    radius = 0.03
    for a in args:
        if a.startswith('--radius='):
            radius = float(a.split('=', 1)[1])
    tags = [a for a in args if a != '--plot' and not a.startswith('--radius=')]

    if tags:
        main(segments_to_run=set(tags), pop_up_plot=pop_up, wake_tube_radius=radius)
    else:
        main(pop_up_plot=pop_up, wake_tube_radius=radius)
