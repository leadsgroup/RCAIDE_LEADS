# RCAIDE/Library/Methods/Powertrain/Converters/Pump/append_pump_conditions.py
# 
# Created:  Jun 2024, M. Clarke  

from RCAIDE.Framework.Mission.Common     import   Conditions

# ---------------------------------------------------------------------------------------------------------------------- 
#  append_pump_conditions
# ----------------------------------------------------------------------------------------------------------------------    
def append_pump_conditions(pump, segment, energy_conditions):  
    """
    Initializes and appends empty pump conditions data structures to the segment state conditions.
    
    Parameters
    ----------
    pump : Pump 
        The pump component for which conditions are being initialized. 
    """ 
    ones_row    = segment.state.ones_row 
    energy_conditions.converters[pump.tag]                                 = Conditions()
    energy_conditions.converters[pump.tag].inputs                          = Conditions()
    energy_conditions.converters[pump.tag].outputs                         = Conditions() 
    
    return 
