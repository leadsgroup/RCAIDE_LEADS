# RCAIDE/Library/Methods/Aerodynamics/Common/Lift/fuselage_correction.py
#  
# Created: Mar 2024 M. Carke

# ----------------------------------------------------------------------------------------------------------------------
#  Fuselage Correction
# ----------------------------------------------------------------------------------------------------------------------
def fuselage_correction(state,settings,geometry):  
    """Corrects aircraft lift based on fuselage effects

    Assumptions:
        None

    Source:
        adg.stanford.edu (Stanford AA241 A/B Course Notes)

    Args:
        settings.fuselage_lift_correction (float): fuselage lift correction [unitless]
        state.conditions.
          freestream.mach_number          (numpy.ndarray): mach number      [unitless]
          aerodynamics.angles.alpha       (numpy.ndarray): angle of attack  [radians]
          aerodynamics.coefficients.lift  (numpy.ndarray): lift coefficient [unitless]

    Returns:
        aircraft_total_lift               (numpy.ndarray): lift coefficient [unitless]   
    """        
    # unpack
    invs_lift       = state.conditions.aerodynamics.coefficients.lift.inviscid.total
        
    if len(geometry.fuselages) > 0:  
        # total lift, assuming one fuselage 
        aircraft_total_lift = invs_lift * settings.fuselage_lift_correction   
    
        state.conditions.aerodynamics.coefficients.lift.total = aircraft_total_lift
    else:
        state.conditions.aerodynamics.coefficients.lift.total = invs_lift

    return 