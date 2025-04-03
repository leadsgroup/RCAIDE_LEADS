# RCAIDE/Methods/Powertrain/Sources/Batteries/Common/size_module_from_energy_and_power.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------  
def size_module_from_energy_and_power(battery, energy, power):
    """
    Calculates the battery mass, maximum energy, and maximum power based on energy and power requirements.
    
    Parameters
    ----------
    battery : BatteryModule
        The battery module with the following attributes:
            - cell.specific_energy : float
                Specific energy of the cell [J/kg]
            - cell.specific_power : float
                Specific power of the cell [W/kg]
            - mass_properties : MassProperties
                Object to store the calculated mass
    energy : float
        Required energy capacity [J]
    power : float
        Required power output [W]
    
    Returns
    -------
    None
        This function modifies the battery object in-place, setting the following attributes:
            - mass_properties.mass : float
                Calculated mass of the battery module [kg]
            - maximum_energy : float
                Maximum energy capacity of the battery module [J]
            - maximum_power : float
                Maximum power output of the battery module [W]
    
    Notes
    -----
    This function determines the battery mass needed to meet both energy and power
    requirements. It calculates two potential masses:
      1. Energy-limited mass: The mass required to meet the energy requirement
      2. Power-limited mass: The mass required to meet the power requirement
      
    The function then selects the larger of these two masses to ensure that both
    requirements are satisfied. The maximum energy and power capabilities are then
    calculated based on the selected mass and the specific energy/power characteristics.
    
    **Major Assumptions**
        * Constant specific energy and power values
        * Linear scaling of energy and power with mass
        * No temperature or state-of-charge effects on performance
    
    See Also
    --------
    RCAIDE.Library.Methods.Powertrain.Sources.Batteries.Common.size_module_from_mass
    """
    
    energy_mass = energy/battery.cell.specific_energy
    power_mass  = power/battery.cell.specific_power 
    mass        = np.maximum(energy_mass, power_mass)

    battery.mass_properties.mass   = mass
    battery.maximum_energy         = battery.cell.specific_energy*mass
    battery.maximum_power          = battery.cell.specific_power*mass
    
    return 