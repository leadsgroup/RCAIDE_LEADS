# RCAIDE/Library/Methods/Powertrain/Converters/Combustor/append_combustor_conditions.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_combustor_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_combustor_conditions(combustor,segment,energy_conditions):
    '''
    Appends initial conditions to combustor component
    '''
    ones_row    = segment.state.ones_row 
    energy_conditions.converters[combustor.tag]                           = Conditions() 
    energy_conditions.converters[combustor.tag].inputs                    = Conditions() 
    energy_conditions.converters[combustor.tag].inputs.nondim_mass_ratio  = ones_row(1)
    energy_conditions.converters[combustor.tag].outputs                   = Conditions()
    return 