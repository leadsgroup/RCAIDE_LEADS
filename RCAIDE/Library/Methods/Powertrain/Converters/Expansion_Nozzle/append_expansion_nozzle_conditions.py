# RCAIDE/Library/Methods/Powertrain/Converters/Expansion_Nozzle/append_expansion_nozzle_conditions.py 
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
# append_expansion_nozzle_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_expansion_nozzle_conditions(expansion_nozzle,segment,propulsor_conditions):   
    propulsor_conditions[expansion_nozzle.tag]                      = Conditions()
    propulsor_conditions[expansion_nozzle.tag].inputs               = Conditions()
    propulsor_conditions[expansion_nozzle.tag].outputs              = Conditions() 
    return 