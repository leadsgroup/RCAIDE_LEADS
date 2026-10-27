# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/compute_rotor_to_rotor_interaction.py
#
# Created:  Oct 2026, M. Clarke, H. Hussien

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.compute_rotor_performance import compute_rotor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.lifting_line_performance import build_rotor_wake_inputs
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.evaluate_bound_vortex_circulation import evaluate_bound_vortex_circulation
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.compute_lifting_line_loads import compute_lifting_line_loads

# ----------------------------------------------------------------------------------------------------------------------
#  compute_rotor_to_rotor_interaction
# ----------------------------------------------------------------------------------------------------------------------
def compute_rotor_to_rotor_interaction(rotors, conditions, wing_on_rotor_coupling=False):
    """Computes the performance of coupled rotors with mutual wake interaction.

    Lifting_Line_Theory rotors are solved together with the real cross-rotor K_bound[i][j]/
    K_wake[i][j] Biot-Savart terms (see evaluate_bound_vortex_circulation), prescribed wake
    only -- free_wake's multi-rotor extension is not wired in yet. Any other-fidelity rotor in
    the list is still solved independently (BEMT/Actuator_Disk rotor-on-rotor coupling is not
    built yet).
    """
    ll_rotors = [r for r in rotors if r.fidelity == 'Lifting_Line_Theory']

    if ll_rotors:
        wake_inputs_list = [build_rotor_wake_inputs(r, conditions, wing_on_rotor_coupling) for r in ll_rotors]
        evaluate_bound_vortex_circulation(ll_rotors, wake_inputs_list, conditions)
        for r, wi in zip(ll_rotors, wake_inputs_list):
            compute_lifting_line_loads(r, wi, conditions)

    for r in rotors:
        if r.fidelity != 'Lifting_Line_Theory':
            compute_rotor_performance(r, conditions, wing_on_rotor_coupling)
    return
