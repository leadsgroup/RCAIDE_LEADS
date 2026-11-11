# VnV/Verification/powertrain/test_wake_model.py
#
# Created: Nov 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import os
import sys
import warnings
import numpy as np

import RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Blade_Element_Momentum_Theory_Helmholtz_Wake.wake_model as wake_model

# ----------------------------------------------------------------------------------------------------------------------
#  main
# ----------------------------------------------------------------------------------------------------------------------
def main():
    # The BEMT inflow angle is solved station by station. In the tiltrotor departure transition the inboard stations
    # have two roots and a stiff residual; solved together as one system (scipy fsolve, before Nov 2026) the solve
    # stalled there and returned an unconverged result, and some stations reached non-physical roots with reversed
    # inflow. These checks run the rotor inputs of every tiltrotor segment at the first iterate.
    vnv_folder = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
    sys.path.insert(0, os.path.join(vnv_folder, "Vehicles"))
    start_folder = os.getcwd()

    # the imported VnV script expects to run from its own folder
    os.chdir(os.path.join(vnv_folder, "Verification", "network_vtol"))
    sys.path.insert(0, os.getcwd())
    rotor_inputs = tiltrotor_rotor_inputs()
    os.chdir(start_folder)

    for tag, (rotor, wake_inputs) in rotor_inputs.items():
        check_inflow_converged(tag, rotor, wake_inputs)
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Checks
# ----------------------------------------------------------------------------------------------------------------------
def check_inflow_converged(tag, rotor, wake_inputs):
    PSI_start = np.ones((wake_inputs.ctrl_pts, wake_inputs.Nr))

    # an unconverged station is reported as a RuntimeWarning, so it is turned into an error here
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        PSI = wake_model.stationwise_newton(PSI_start, wake_inputs, rotor)

    R = np.abs(wake_model.residual(PSI, wake_inputs, rotor))
    print(f"  {tag}: max |residual| {R.max():.1e}, inflow angle from {PSI.min():.3f} to {PSI.max():.3f} rad")
    assert R.max() < 1e-6, f"{tag}: inflow angle not converged"
    assert PSI.min() >= -np.pi/2 and PSI.max() <= np.pi, f"{tag}: inflow angle outside [-pi/2, pi]"
    return

# ----------------------------------------------------------------------------------------------------------------------
#  Rotor inputs of the tiltrotor mission
# ----------------------------------------------------------------------------------------------------------------------
def tiltrotor_rotor_inputs():
    """The rotor and wake inputs of each tiltrotor segment, at the first iterate of the mission solve."""
    import vtol_aircraft_test as vtol
    vehicle  = vtol.TR_vehicle_setup(False)
    analyses = vtol.TR_analyses_setup(vtol.TR_configs_setup(vehicle))
    mission  = vtol.missions_setup(vtol.TR_mission_setup(analyses)).base_mission

    # record the first wake solve of each segment by wrapping the module's wake_convergence
    captured = {}
    wake_convergence = wake_model.wake_convergence
    def recording_wake_convergence(rotor, wake_inputs):
        captured.setdefault(current_tag[0], (rotor, wake_inputs))
        return wake_convergence(rotor, wake_inputs)
    current_tag = [None]
    wake_model.wake_convergence = recording_wake_convergence

    # initialize each segment from the previous one and iterate it once, as a mission solve starts
    try:
        mission.process.initialize(mission)
        previous = None
        for tag, segment in mission.segments.items():
            if previous is not None:
                segment.state.initials = previous
            current_tag[0] = tag
            segment.process.initialize.expand_state(segment)
            segment.process.initialize(segment)
            segment.process.iterate(segment)
            previous = segment.state
    finally:
        wake_model.wake_convergence = wake_convergence
    captured.pop(None, None)   # solves made while sizing the vehicle, before any segment
    return captured


if __name__ == '__main__':
    main()
