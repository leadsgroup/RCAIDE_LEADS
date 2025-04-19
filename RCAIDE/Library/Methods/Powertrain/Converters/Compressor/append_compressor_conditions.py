# RCAIDE/Library/Methods/Powertrain/Converters/compressor/append_compressor_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_compressor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_compressor_conditions(compressor,segment,propulsor_conditions): 
    propulsor_conditions[compressor.tag]         = Conditions()
    propulsor_conditions[compressor.tag].inputs  = Conditions()
    propulsor_conditions[compressor.tag].outputs = Conditions()
    return 