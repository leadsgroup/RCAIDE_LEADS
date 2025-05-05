# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/total_drag.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  Wave Drag 
# ---------------------------------------------------------------------------------------------------------------------- 
def total_drag(state,settings,geometry):
    """ Computes the total drag of an aircraft.

    Assumptions:
        None

    Source:
        None 

    Args:
        settings.
          drag_coefficient_increment                          (float): drag_coefficient_increment [Unitless] 
        state.conditions.aerodynamics.coefficients.drag.
          trim_corrected_drag                         (numpy.ndarray): trim corrected drag        [Unitless]
          spoiler_drag                                (numpy.ndarray): spoiler drag               [Unitless]
        geometry                                               (dict): aircraft data structure    [-]


    Returns:
        None 
    """

    # unpack inputs  
    trim_correction_factor     = settings.trim_drag_correction_factor    
    drag_coefficient_increment = settings.drag_coefficient_increment  
    drag                       = state.conditions.aerodynamics.coefficients.drag

    # various drag components
    parasite_total        = drag.parasite.total            
    induced_total         = drag.induced.total            
    compressibility_total = drag.compressible.total     
    miscellaneous_drag    = drag.miscellaneous.total
    cooling_drag          = drag.cooling.total 
    spoiler_drag          = drag.spoiler.total 
    #flap_drag             = drag.flap.total # Yet to be implemented
    #slat_drag             = drag.slat.total # Yet to be implemented

    # untrimmed drag 
    untrimmed_drag  =  parasite_total + induced_total  + compressibility_total + miscellaneous_drag + cooling_drag + spoiler_drag # + flap_drag + slat_drag
    
    # trim correction
    corrected_aircraft_total_trim_drag = trim_correction_factor * untrimmed_drag   
     
    # total drag 
    aircraft_total_drag =  corrected_aircraft_total_trim_drag   + drag_coefficient_increment

    # Store to results 
    drag.total           = aircraft_total_drag
    
    return  