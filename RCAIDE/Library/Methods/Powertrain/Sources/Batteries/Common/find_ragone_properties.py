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
    """determines mass of a battery based on the specific energy, energy required, and power required,
    works by calling find_specific_power and size_module_from_energy_and_power
    Assumptions:
    None
    
    Inputs:
    energy            [J]
    power             [W]
    battery.
    
    battery.
      type
      specific_energy [J/kg]               
      specific_power  [W/kg]
      ragone.
        constant_1    [W/kg]
        constant_2    [J/kg]
        upper_bound   [J/kg]
        lower_bound   [J/kg]
                
    Outputs:
    battery.
      maximum_energy      [J]
      maximum_power       [W]
      specific_energy [J/kg]
      specific_power  [W/kg]
      mass_properties.
        mass           [kg]    
    
    
    """
    
    
    
    
    find_specific_power(battery, specific_energy)
    size_module_from_energy_and_power(battery, energy, power)
    
    #can used for a simple optimization
    return battery.mass_properties.mass 