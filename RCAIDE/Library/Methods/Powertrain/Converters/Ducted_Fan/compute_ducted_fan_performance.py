# RCAIDE/Library/Methods/Powertrain/Converters/Ducted_Fan/Performance/compute_ducted_fan_performance.py

# 
# Created:  Jan 2025, M. Clarke
# Modified: Jan 2025, M. Clarke, M. Guidotti    

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
 # RCAIDE imports 
import RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.Performance.Blade_Element_Momentum_Theory.BEMT_performance as BEMT_performance
import RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.Performance.Rankine_Froude_Momentum_Theory.RFMT_performance as RFMT_performance
  

# ---------------------------------------------------------------------------------------------------------------------- 
# compute_ducted_fan_performance
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_ducted_fan_performance(ducted_fan,ducted_fan_conditions,conditions):
    """
    Computes ducted fan performance characteristics using either Blade Element Momentum Theory (BEMT) 
    or Rankine-Froude Momentum Theory.

    Parameters
    ----------
    propulsor : Converter
        Ducted fan propulsor component containing the ducted fan
    state : Conditions
        Mission segment state conditions
    center_of_gravity : array_like, optional
        Aircraft center of gravity coordinates [[x, y, z]], defaults to [[0.0, 0.0, 0.0]]
 

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Ducted_Fan
    RCAIDE.Library.Methods.Powertrain.Converters.Ducted_Fan.design_ducted_fan
    """
  
    
    if ducted_fan.fidelity == 'Blade_Element_Momentum_Theory': 

        outputs = BEMT_performance(ducted_fan,ducted_fan_conditions,conditions)
                      
    elif ducted_fan.fidelity == 'Rankine_Froude_Momentum_Theory': 

        outputs = RFMT_performance(ducted_fan,ducted_fan_conditions,conditions)
    
    conditions.energy.converters[ducted_fan.tag] = outputs   
    
    return  
