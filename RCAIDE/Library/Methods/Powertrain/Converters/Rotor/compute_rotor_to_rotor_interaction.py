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
    """Computes the performance of coupled rotors with mutual wake interaction. Rotors are currently solved independently."""
    for rotor in rotors:
        compute_rotor_performance(rotor,conditions,wing_on_rotor_coupling)
    return
