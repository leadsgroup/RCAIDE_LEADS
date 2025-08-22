# RCAIDE/Library/Methods/Aerodynamics/Common/Lift/fuselage_correction.py
#  
# Created: Mar 2024 M. Carke

# ----------------------------------------------------------------------------------------------------------------------
#  Fuselage Correction
# ----------------------------------------------------------------------------------------------------------------------
def fuselage_correction(state,settings,geometry):  
    """
    Corrects aircraft lift coefficient based on fuselage interference effects.

    Parameters
    ----------
    state : Data
        Flight conditions and aerodynamic state containing:
            - conditions.aerodynamics.coefficients.lift.inviscid.total : float
                Inviscid lift coefficient [unitless]
    settings : dict
        Aerodynamic analysis settings containing:
            - fuselage_lift_correction : float
                Fuselage lift correction factor [unitless]
    geometry : Data
        Aircraft geometry containing:
            - fuselages : list
                List of fuselage objects (may be empty)

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.lift.total

    Notes
    -----
    This function applies a fuselage lift correction factor to account for
    interference effects between the fuselage and wing. The correction
    modifies the inviscid lift coefficient.
    
    **Major Assumptions**
        * Single fuselage configuration (first fuselage used if multiple exist)
        * Fuselage lift correction factor is user-defined
        * Inviscid lift coefficient represents wing-only contribution
        * Fuselage effects are multiplicative rather than additive
        * No fuselage results in no correction applied
    
    **Theory**

    The total aircraft lift coefficient is calculated as:

    :math:`C_{L,total} = C_{L,inviscid} \\cdot f_{fuselage}`

    where:
        - :math:`C_{L,inviscid}` is the inviscid lift coefficient (wing only)
        - :math:`f_{fuselage}` is the fuselage lift correction factor

    **Definitions**

    'Fuselage Correction'
        Factor accounting for the influence of fuselage on total aircraft lift.
    
    'Interference Effects'
        Aerodynamic interactions between different aircraft components that modify individual component performance.

    References
    ----------
    [1] Stanford AA241 Course Notes. adg.stanford.edu

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Common.Lift.compute_flap_lift
    RCAIDE.Library.Methods.Aerodynamics.Common.Lift.compute_slat_lift
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