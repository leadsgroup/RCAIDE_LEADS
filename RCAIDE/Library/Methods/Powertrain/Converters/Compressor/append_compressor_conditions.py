# RCAIDE/Library/Methods/Powertrain/Converters/compressor/append_compressor_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_compressor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_compressor_conditions(compressor,segment,energy_conditions): 
    ones_row    = segment.state.ones_row 
    energy_conditions.converters[compressor.tag]                                   = Conditions()
    energy_conditions.converters[compressor.tag].inputs                            = Conditions()
    energy_conditions.converters[compressor.tag].outputs                           = Conditions()
    energy_conditions.converters[compressor.tag].outputs.external_shaft_work_done  = 0*ones_row(1)
    energy_conditions.converters[compressor.tag].outputs.external_electrical_power = 0*ones_row(1)
    return 