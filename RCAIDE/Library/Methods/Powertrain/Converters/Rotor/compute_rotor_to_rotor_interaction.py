# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/compute_rotor_to_rotor_interaction.py
# 
# Created:  Oct 2026, M. Clarke, H. Hussien

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.compute_rotor_performance import compute_rotor_performance

# ---------------------------------------------------------------------------------------------------------------------- 
#  compute_rotor_to_rotor_interaction
# ----------------------------------------------------------------------------------------------------------------------  
def compute_rotor_to_rotor_interaction(rotors, conditions, wing_on_rotor_coupling=False):
    """Computes the performance of coupled rotors with mutual wake interaction. Rotors are
    currently solved independently -- true coupled (Lifting_Line_Theory) multi-rotor induction
    is follow-up work, not implemented here yet.

    Marks each rotor's conditions entry as already solved so the main per-propulsor loop in
    Network.evaluate() (which runs right after this, for every propulsor regardless of coupling)
    does not solve it a second time -- this call is the ONLY place rotor aero gets computed for
    anything passed in here.
    """
    for rotor in rotors:
        compute_rotor_performance(rotor,conditions,wing_on_rotor_coupling)
        conditions.energy.converters[rotor.tag].rotor_interaction_solved = True
    return
