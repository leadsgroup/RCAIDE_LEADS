# RCAIDE/Methods/Energy/Propulsors/Electric_Rotor_Propulsor/__init__.py
# 

"""
Collection of methods for analyzing electric rotor propulsion systems. These methods handle the 
performance computation, state management, and solver integration for electric motor-driven rotors. 

See Also
--------
RCAIDE.Library.Methods.Powertrain.Propulsors.Electric_Ducted_Fan_Propulsor
RCAIDE.Library.Methods.Powertrain.Propulsors.ICE_Propulsor
"""

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

from .append_electric_rotor_conditions           import append_electric_rotor_conditions
from .append_electric_rotor_residual_and_unknown import append_electric_rotor_residual_and_unknown
from .compute_electric_rotor_performance         import compute_electric_rotor_performance
from .pack_electric_rotor_residuals              import pack_electric_rotor_residuals
from .unpack_electric_rotor_unknowns             import unpack_electric_rotor_unknowns
from .design_electric_rotor                      import design_electric_rotor