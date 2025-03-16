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
   
    if np.all((Mach<=1.0) == True): 
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
                            
        # total miscellaneous drag 
        miscellaneous_drag =  (0.40* (0.0184 + 0.000469 * swet_tot - 1.13*10**-7 * swet_tot ** 2)) / S_ref
        total_miscellaneous_drag = miscellaneous_drag *np.ones_like(Mach)    
    else:

        # Initialize drag
        total_nacelle_base_drag   = 0.0  
        nacelle_base_drag_results = Data() 
    
        # Estimating nacelle drag 
        for network in  geometry.networks: 
            for propulsor in network.propulsors:  
                if 'nacelle' in propulsor: 
                    nacelle_base_drag = 0.5/12. * np.pi * propulsor.nacelle.diameter * 0.2/S_ref 
                    nacelle_base_drag_results[propulsor.nacelle.tag] = nacelle_base_drag * np.ones_like(Mach)   
                    total_nacelle_base_drag += nacelle_base_drag     

        #   Fuselage upsweep drag 
        fuselage_upsweep_drag = 0.006 /S_ref 
         
        # total miscellaneous drag 
        miscellaneous_drag = total_nacelle_base_drag + fuselage_upsweep_drag
        total_miscellaneous_drag = miscellaneous_drag *np.ones_like(Mach)
        
    # Store results 
    conditions.aerodynamics.coefficients.drag.miscellaneous = Data( 
        total            = total_miscellaneous_drag  ,
    ) 
    return  
    
