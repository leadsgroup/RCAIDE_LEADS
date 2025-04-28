# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/supersonic_miscellaneous_drag_aircraft.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
  
from RCAIDE.Framework.Core                    import Data  

# package imports
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Supersonic Miscellaneous Drag Total
# ----------------------------------------------------------------------------------------------------------------------   
def miscellaneous_drag(state,settings,geometry):
    """Computes the miscellaneous drag associated with an aircraft

    Assumptions:
    Basic fit

    Source:
    http://aerodesign.stanford.edu/aircraftdesign/aircraftdesign.html (Stanford AA241 A/B Course Notes)

    Args:
    configuration.trim_drag_correction_factor  [Unitless]
    geometry.nacelle.diameter                  [m]
    geometry.reference_area                    [m^2]
    geometry.wings['main_wing'].aspect_ratio   [Unitless]
    state.conditions.freestream.mach_number    [Unitless] (actual values are not used)

    Returns:
    total_miscellaneous_drag                   [Unitless] 
    """ 

    conditions     = state.conditions  
    S_ref          = geometry.reference_area
    Mach           = conditions.freestream.mach_number 
   
    supersonic_mach_flag = Mach >=1.0 
    swet_tot       = 0.
    for wing in geometry.wings:
        swet_tot += wing.areas.wetted 
    for fuselage in geometry.fuselages:
        swet_tot += fuselage.areas.wetted
    for boom in geometry.booms:
        swet_tot += boom.areas.wetted
    for network in geometry.networks: 
        for propulsor in network.propulsors:  
            if 'nacelle' in propulsor:
                swet_tot += propulsor.nacelle.areas.wetted
                        
    # total subsonic miscellaneous drag 
    total_miscellaneous_drag      =  3 * (0.40* (0.0184 + 0.000469 * swet_tot - 1.13*10**-7 * swet_tot ** 2)) / S_ref *np.ones_like(Mach) 
    
    supersonic_miscellaneous_drag =  0.006 /S_ref *np.ones_like(Mach)
    total_miscellaneous_drag[supersonic_mach_flag] = supersonic_miscellaneous_drag[supersonic_mach_flag] 
        
    # Store results 
    conditions.aerodynamics.coefficients.drag.miscellaneous = Data( 
        total            = total_miscellaneous_drag,)
    return  
    
