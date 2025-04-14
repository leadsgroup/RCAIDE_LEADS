# RCAIDE/Library/Methods/Weights/Correlation_Buildups/BWB/compute_aft_centerbody_weight.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units

# ---------------------------------------------------------------------------------------------------------------------- 
# Aft Centerbody Weight 
# ----------------------------------------------------------------------------------------------------------------------
def compute_aft_centerbody_weight(no_of_engines, aft_centerbody_area, aft_centerbody_taper, TOGW):
    """
    Computes the structural weight of the aft section of a BWB centerbody using regression-based methods.

    Parameters
    ----------
    no_of_engines : int
        Number of engines mounted on aft centerbody
    aft_centerbody_area : float
        Planform area of aft centerbody section [m²]
        Typically measured behind 70% chord
    aft_centerbody_taper : float
        Taper ratio of aft centerbody section
        Excludes chord taken up by pressurized cabin
    TOGW : float
        Takeoff gross weight of aircraft [kg]

    Returns
    -------
    W_aft : float
        Estimated structural weight of BWB aft centerbody [kg]

    Notes
    -----
    Uses regression equations derived from FEA studies to estimate the structural
    weight required to handle engine loads and aerodynamic forces.

    **Major Assumptions**
        * Engines are mounted on the aft centerbody
        * Aft section is unpressurized
        * Linear relationship with planform area
        * Engine count affects weight through simple scaling
        * Structure sized primarily by engine and aerodynamic loads

    **Theory**
    Weight is computed using:
    .. math::
        W_{aft} = (1 + 0.05n_e)\\cdot 0.53\\cdot S_{aft}\\cdot W^{0.2}\\cdot(\\lambda + 0.5)

    where:
        * n_e = number of engines
        * S_aft = aft centerbody area
        * W = takeoff weight
        * λ = taper ratio

    References
    ----------
    [1] Bradley, K. R., "A Sizing Methodology for the Conceptual Design of 
        Blended-Wing-Body Transports," NASA/CR-2004-213016, 2004.

    See Also
    --------
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.BWB.FLOPS.compute_cabin_weight
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.BWB.FLOPS.compute_operating_empty_weight
    """         
    # convert to imperial units and shorten variable names 
    S_aft  = aft_centerbody_area  / Units.feet ** 2.0
    l_aft  = aft_centerbody_taper
    W      = TOGW/ Units.pounds
    
    W_aft = (1.0 + 0.05*no_of_engines) * 0.53 * S_aft * (W**0.2) * (l_aft + 0.5)
    
    # convert back to base units
    W_aft = W_aft * Units.pounds
    
    return W_aft