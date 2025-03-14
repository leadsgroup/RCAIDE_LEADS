# RCAIDE/Library/Methods/Powertrain/Converters/compression_nozzle/append_compression_nozzle_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports 
# ----------------------------------------------------------------------------------------------------------------------   
from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
# append_compression_nozzle_conditions 
# ----------------------------------------------------------------------------------------------------------------------    
def append_compression_nozzle_conditions(compression_nozzle,segment,energy_conditions): 
    energy_conditions.converters[compression_nozzle.tag]                     = Conditions()
    energy_conditions.converters[compression_nozzle.tag].inputs              = Conditions()
    energy_conditions.converters[compression_nozzle.tag].outputs             = Conditions()
    return 