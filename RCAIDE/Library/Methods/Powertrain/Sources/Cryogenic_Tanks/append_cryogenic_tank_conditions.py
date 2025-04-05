# RCAIDE/Methods/Powertrain/Sources/Cryogenic_Tanks/append_cryogenic_tank_conditions.py
# 
# 
# Created:  Jan 2025, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports  
from RCAIDE.Framework.Mission.Common     import   Conditions

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------  
def append_cryogenic_tank_conditions(cryogenic_tank,segment,bus):
    '''
    Appends conditions to cryogenic tank compoment
    '''
    ones_row    = segment.state.ones_row                 
    segment.state.conditions.energy[bus.tag][cryogenic_tank.tag]                 = Conditions()  
    segment.state.conditions.energy[bus.tag][cryogenic_tank.tag].mass_flow_rate  = ones_row(1)  
    segment.state.conditions.energy[bus.tag][cryogenic_tank.tag].mass            = ones_row(1)
    
    return 
