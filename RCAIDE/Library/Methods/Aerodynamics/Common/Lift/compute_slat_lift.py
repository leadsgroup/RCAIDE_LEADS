
# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
from   RCAIDE import  * 
from   RCAIDE.Framework.Core import Units
import numpy as np

# ----------------------------------------------------------------------
#  compute_slat_lift
# ----------------------------------------------------------------------

def compute_slat_lift(slat_angle,sweep_angle):
    """
    Computes the increase in lift coefficient due to leading edge slat deployment.

    Parameters
    ----------
    slat_angle : float
        Slat deflection angle [radians]
    sweep_angle : float
        Wing leading edge sweep angle [radians]

    Returns
    -------
    dcl_slat : float
        Lift coefficient increase due to slat deployment [unitless]

    Notes
    -----
    This function calculates the lift augmentation due to leading edge slat
    deployment using the Stanford AA241 methodology. The calculation accounts
    for slat deflection angle and wing sweep effects.
    
    **Major Assumptions**
        * Stanford AA241 methodology is valid for typical slat configurations
        * Sweep corrections follow cosine relationships
        * Slat angle effects follow cosine squared relationship
        * No interference effects between slats and other high-lift devices
    
    **Theory**

    The lift coefficient increment due to slats follows the AA241 method:

    :math:`\\Delta C_{L,slat} = \\frac{\\delta_{slat}}{23°} \\cdot \\cos^{1.4}(\\Lambda) \\cdot \\cos^2(\\delta_{slat})`

    where:
        - :math:`\\delta_{slat}` is the slat deflection angle in degrees
        - :math:`\\Lambda` is the wing leading edge sweep angle in radians

    **Definitions**

    'Slat'
        High-lift device mounted on the leading edge of a wing to delay stall and increase lift coefficient.
    
    References
    ----------
    [1] Stanford AA241 Course Notes. adg.stanford.edu

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Common.Lift.compute_flap_lift
    """

    # unpack
    sa = slat_angle  / Units.deg
    sw = sweep_angle

    # AA241 Method from adg.stanford.edu
    dcl_slat = (sa/23.)*(np.cos(sw))**1.4 * np.cos(sa * Units.deg)**2

    #returning dcl_slat
    return dcl_slat
 