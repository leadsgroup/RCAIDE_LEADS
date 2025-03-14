# RCAIDE/Library/Methods/Powertrain/Converters/Expansion_Nozzle/append_expansion_nozzle_conditions.py 
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
# append_expansion_nozzle_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_expansion_nozzle_conditions(expansion_nozzle,segment,energy_conditions):   
    energy_conditions.converters[expansion_nozzle.tag]                      = Conditions()
    energy_conditions.converters[expansion_nozzle.tag].inputs               = Conditions()
    energy_conditions.converters[expansion_nozzle.tag].outputs              = Conditions() 
    return 