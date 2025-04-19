# RCAIDE/Library/Methods/Powertrain/Converters/supersonic_nozzle/append_supersonic_nozzle_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
# append_supersonic_nozzle_conditions 
# ----------------------------------------------------------------------------------------------------------------------    
def append_supersonic_nozzle_conditions(supersonic_nozzle,segment,propulsor_conditions): 
    propulsor_conditions[supersonic_nozzle.tag]                     = Conditions()
    propulsor_conditions[supersonic_nozzle.tag].inputs              = Conditions()
    propulsor_conditions[supersonic_nozzle.tag].outputs             = Conditions()
    return 