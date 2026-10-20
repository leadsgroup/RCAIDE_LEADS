# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/compute_rotor_to_rotor_interaction.py
#
# Created:  Oct 2026, M. Clarke, H. Hussien

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.compute_rotor_performance import compute_rotor_performance
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.lifting_line_performance import prepare_wake_inputs
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.evaluate_coupled_bound_vortex_circulation import evaluate_coupled_bound_vortex_circulation
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.compute_lifting_line_loads import compute_lifting_line_loads

# ----------------------------------------------------------------------------------------------------------------------
#  compute_rotor_to_rotor_interaction
# ----------------------------------------------------------------------------------------------------------------------
def compute_rotor_to_rotor_interaction(rotors, conditions, wing_on_rotor_coupling=False):
    """Computes the performance of coupled rotors with mutual wake interaction.

    Lifting_Line_Theory rotors are solved TOGETHER via a Jacobi-coupled bound-circulation
    solve (evaluate_coupled_bound_vortex_circulation) with a PRESCRIBED wake only -- free-wake
    relaxation coupling across rotors is follow-up work, not implemented here yet. Rotors at
    any other fidelity are still solved independently, one at a time.

    Marks each rotor's conditions entry as already solved so the main per-propulsor loop in
    Network.evaluate() (which runs right after this, for every propulsor regardless of coupling)
    does not solve it a second time -- this call is the ONLY place rotor aero gets computed for
    anything passed in here.
    """
    ll_rotors = [r for r in rotors if r.fidelity == 'Lifting_Line_Theory']
    other_rotors = [r for r in rotors if r.fidelity != 'Lifting_Line_Theory']

    if ll_rotors:
        wake_inputs_list = [prepare_wake_inputs(rotor, conditions) for rotor in ll_rotors]
        evaluate_coupled_bound_vortex_circulation(ll_rotors, wake_inputs_list, conditions)
        for rotor, wake_inputs in zip(ll_rotors, wake_inputs_list):
            compute_lifting_line_loads(rotor, wake_inputs, conditions)
            conditions.energy.converters[rotor.tag].rotor_interaction_solved = True

    for rotor in other_rotors:
        compute_rotor_performance(rotor,conditions,wing_on_rotor_coupling)
        conditions.energy.converters[rotor.tag].rotor_interaction_solved = True
    return
