 # compute_flap_lift.py
#
# Created:  Dec 2013, A. Varyar
# Modified: Feb 2014, T. Orra
#           Jan 2016, E. Botero         

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
from   RCAIDE import  * 
from   RCAIDE.Framework.Core import Units
import numpy  as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  compute_flap_lift
# ---------------------------------------------------------------------------------------------------------------------- 
def compute_flap_lift(t_c,flap_type,flap_chord,flap_angle,sweep,wing_Sref,wing_affected_area):
    """
    Computes the increase in lift coefficient due to trailing edge flap deployment.

    Parameters
    ----------
    t_c : float
        Wing thickness-to-chord ratio [unitless]
    flap_type : str, optional
        Type of flap ('single_slotted', 'triple_slotted', or None)
    flap_chord : float
        Flap chord as fraction of wing chord [unitless]
    flap_angle : float
        Flap deflection angle [radians]
    sweep : float
        Wing sweep angle [radians]
    wing_Sref : float
        Wing reference area [m²]
    wing_affected_area : float
        Wing area affected by flaps [m²]

    Returns
    -------
    dcl_max_flaps : float
        Lift coefficient increase due to flap deployment [unitless]

    Notes
    -----
    This function calculates the lift augmentation due to trailing edge flap
    deployment using empirical correlations. The calculation accounts for
    wing geometry, flap characteristics, and aerodynamic corrections.
    
    **Major Assumptions**
        * Empirical correlations are valid for typical transport aircraft
        * Flap effects are linear with affected area ratio
        * Sweep corrections follow cosine relationships
        * Flap type corrections are multiplicative factors
        * Chord and deflection corrections are independent
    
    **Theory**

    The basic increase in lift coefficient is calculated using a polynomial fit:

    :math:`\\Delta C_{L,max,ref} = -4 \\times 10^{-5}(t/c)^4 + 0.0014(t/c)^3 - 0.0093(t/c)^2 + 0.0436(t/c) + 0.9734`

    Flap type corrections are applied:
        - Single slotted: :math:`\\Delta C_{L,max} = 0.93 \\cdot \\Delta C_{L,max,ref}`
        - Triple slotted: :math:`\\Delta C_{L,max} = 1.08 \\cdot \\Delta C_{L,max,ref}`
        - No flap: :math:`\\Delta C_{L,max} = 0`

    The chord correction factor is:
    :math:`K_c = 0.0395 \\cdot f_c + 0.0057`

    where :math:`f_c` is the flap chord fraction in percent.

    The deflection correction factor is:
    :math:`K_d = -1.7857 \\times 10^{-4} \\theta^2 + 2.9214 \\times 10^{-2} \\theta - 1.4000 \\times 10^{-2}`

    where :math:`\\theta` is the flap deflection angle in degrees.

    The sweep correction factor is:
    :math:`K_{sw} = (1 - 0.08 \\cos^2(\\Lambda)) \\cdot \\cos^{0.75}(\\Lambda)`

    where :math:`\\Lambda` is the wing sweep angle.

    The final lift coefficient increment is:
    :math:`\\Delta C_{L,flaps} = K_c \\cdot K_d \\cdot K_{sw} \\cdot \\Delta C_{L,max,ref} \\cdot \\frac{S_{wf}}{S_{ref}}`

    where :math:`S_{wf}` is the wing area affected by flaps.
    
    **Definitions**

    'Flap'
        High-lift device mounted on the trailing edge of a wing to increase lift coefficient.
    
    'Affected Area'
        Portion of the wing surface influenced by flap deployment.

    References
    ----------
    Unknown

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Common.Lift.compute_slat_lift
    """          

    #unpack
    tc_r  = t_c
    fc    = flap_chord * 100.
    fa    = flap_angle / Units.deg
    Swf   = wing_affected_area
    sweep = sweep

    # Basic increase in CL due to flap
    dmax_ref= -4E-05*tc_r**4 + 0.0014*tc_r**3 - 0.0093*tc_r**2 + 0.0436*tc_r + 0.9734

    # Corrections for flap type
    if flap_type == None:
        dmax_ref = 0.
    elif flap_type.upper() == 'single_slotted'.upper():
        dmax_ref = dmax_ref * 0.93
    elif flap_type.upper() == 'triple_slotted'.upper():
        dmax_ref = dmax_ref * 1.08

    # Chord correction
    Kc =  0.0395*fc    + 0.0057

    # Deflection correction
    Kd = -1.7857E-04*fa**2 + 2.9214E-02*fa - 1.4000E-02

    # Sweep correction
    Ksw = (1 - 0.08 * (np.cos(sweep))**2) * (np.cos(sweep)) ** 0.75

    # Applying corrections
    dmax_flaps = Kc * Kd * Ksw * dmax_ref

    # Final CL increment due to flap
    dcl_max_flaps = dmax_flaps  *  Swf / wing_Sref

    return dcl_max_flaps