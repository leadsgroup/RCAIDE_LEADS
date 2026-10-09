# RCAIDE/Library/Methods/Powertrain/Converters/Rotor/compute_rotor_wake_interaction.py
# 
# Created:  Oct 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Library.Methods.Powertrain.Converters.Rotor.compute_rotor_performance import compute_rotor_performance

# ---------------------------------------------------------------------------------------------------------------------- 
#  compute_rotor_wake_interaction
# ----------------------------------------------------------------------------------------------------------------------  
def compute_rotor_wake_interaction(rotors, conditions):
    """Computes the performance of rotors with mutual wake interaction. Rotors are currently solved independently."""
    for rotor in rotors:
        compute_rotor_performance(rotor,conditions)
    return
