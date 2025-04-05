# RCAIDE/Library/Methods/Powertrain/Converters/Ram/append_ram_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_ram_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_ram_conditions(ram,segment,energy_conditions): 
    energy_conditions.converters[ram.tag]                              = Conditions() 
    energy_conditions.converters[ram.tag].inputs                       = Conditions() 
    energy_conditions.converters[ram.tag].outputs                      = Conditions() 
    return 