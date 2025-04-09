# RCAIDE/Library/Methods/Weights/Correlation_Buildups/BWB/compute_cabin_weight.py
# 
# Created: Sep 2024, M. Clarke 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from RCAIDE.Framework.Core import Units

# ---------------------------------------------------------------------------------------------------------------------- 
#  Cabin Weight 
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_cabin_weight(cabin_area, TOGW):
    """
    Computes the structural weight of the pressurized cabin section of a BWB aircraft using 
    regression-based methods derived from FEA studies.

    Parameters
    ----------
    cabin_area : float
        Planform area of pressurized passenger cabin [m²]
    TOGW : float
        Takeoff gross weight of aircraft [kg]

    Returns
    -------
    W_cabin : float
        Estimated structural weight of BWB pressurized cabin section [kg]

    Notes
    -----
    Uses regression equations derived from detailed FEA studies by Bradley to estimate the structural
    weight required for the non-circular pressure vessel of a BWB cabin [1].

    **Major Assumptions**
        * Pressurized sandwich composite structure
        * Ultimate cabin pressure differential of 18.6 psi
        * Critical load case is 2.5g maneuver at maximum TOGW
        * Uniform pressure distribution
        * Non-buckling design
        * Structural efficiency similar to baseline FEA model

    **Theory**
    Weight is computed using:
    .. math::
        W_{cabin} = 5.698865 \\cdot 0.316422 \\cdot W^{0.166552} \\cdot S_{cab}^{1.061158}

    where:
        * W = takeoff weight
        * S_cab = cabin planform area

    The equation coefficients account for:
        * Non-circular pressure vessel penalties
        * Composite material properties
        * Combined pressure and flight loads
        * Minimum gauge requirements

    References
    ----------
    [1] Bradley, K. R., "A Sizing Methodology for the Conceptual Design of Blended-Wing-Body Transports," NASA/CR-2004-213016, 2004.
    See Also
    --------
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.BWB.FLOPS.compute_aft_centerbody_weight
    RCAIDE.Library.Methods.Mass_Properties.Weight_Buildups.Conventional.BWB.FLOPS.compute_operating_empty_weight
    """       
    
    # convert to imperial units
    S_cab    = cabin_area / Units.feet ** 2.0
    W        = TOGW       / Units.pounds
    
    W_cabin = 5.698865 * 0.316422 * (W ** 0.166552) * S_cab ** 1.061158
    
    # convert to SI units
    W_cabin = W_cabin * Units.pounds
    
    return W_cabin