# RCAIDE/Methods/Powertrain/Sources/Batteries/Ragone/find_ragone_properties.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports 
from .size_module_from_energy_and_power  import size_module_from_energy_and_power
from .find_specific_power                import find_specific_power

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------  
def find_ragone_properties(specific_energy, battery, energy, power):
    """
    Determines battery mass based on specific energy, energy required, and power required.
    
    Parameters
    ----------
    specific_energy : float
        Specific energy value to use for the battery [J/kg]
    battery : Battery
        The battery component to be sized
    energy : float
        Required energy capacity [J]
    power : float
        Required power output [W]
    
    Returns
    -------
    mass : float
        Calculated mass of the battery [kg]
    
    Notes
    -----
    This function calculates the mass of a battery needed to meet both energy and power
    requirements, using a specific energy value and the corresponding specific power
    determined from a Ragone curve.
    
    The function performs the following steps:
      1. Calculates the specific power corresponding to the given specific energy using the Ragone curve correlation
      2. Sizes the battery module based on the energy and power requirements
    
    The Ragone curve describes the trade-off between specific energy and specific power
    in energy storage devices. By using this relationship, the function ensures that
    the battery is sized appropriately to meet both energy and power requirements.
    
    **Major Assumptions**
        * The Ragone curve accurately represents the energy-power trade-off
        * Linear scaling of energy and power with mass
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common.find_specific_power
    RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common.size_module_from_energy_and_power
    """
    
    find_specific_power(battery, specific_energy)
    size_module_from_energy_and_power(battery, energy, power)
    
    # can be used for a simple optimization
    return battery.mass_properties.mass 