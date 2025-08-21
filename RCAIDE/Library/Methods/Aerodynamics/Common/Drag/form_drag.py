# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/form_drag.py 
# 
# Created:  Jul 2025, M. Clarke 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from  RCAIDE.Framework.Core import  Units
from RCAIDE.Library.Methods.Utilities         import Cubic_Spline_Blender

# ---------------------------------------------------------------------------------------------------------------------- 
#  Form Drag 
# ----------------------------------------------------------------------------------------------------------------------   
def form_drag(state,settings,geometry):
    """
    Computes the form drag coefficient associated with aircraft geometry and angle of attack.

    Parameters
    ----------
    state : Data
        Flight conditions and aerodynamic state containing:
            - conditions.freestream.mach_number : float
                Freestream Mach number [unitless]
            - conditions.aerodynamics.angles.alpha : float
                Angle of attack [radians]
    settings : dict
        Aerodynamic analysis settings containing:
            - supersonic.begin_drag_rise_mach_number : float
                Mach number at which drag rise begins [unitless]
            - supersonic.end_drag_rise_mach_number : float
                Mach number at which drag rise ends [unitless]
    geometry : Data
        Aircraft geometry containing:
            - reference_area : float
                Reference area for drag coefficient calculation [m²]
            - wings : list
                List of wing objects containing:
                    - segments : dict
                        Dictionary of wing segments with:
                            - areas.reference : float
                                Reference area of the segment [m²]
                    - areas.reference : float
                        Reference area of the wing [m²]

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.form.total

    Notes
    -----
    This function calculates the form drag coefficient based on angle of attack and Mach number
    effects. The calculation accounts for separation drag on wing segments and applies Mach
    number corrections for compressibility effects. The form drag is blended between subsonic
    and supersonic regimes using a cubic spline.
    
    **Major Assumptions**
        * Form drag is primarily due to wing geometry and angle of attack
        * Separation drag follows polynomial correlation with angle of attack
        * Mach number correction is valid for typical transport aircraft
        * Vertical tail contributions are negligible
        * Cubic spline blending smooths transition between flight regimes
    
    **Theory**

    The Mach number correction factor is:

    :math:`M_{correction} = 2.9788 M^3 - 6.4381 M^2 + 4.4967 M`

    where :math:`M` is the freestream Mach number.

    The separation drag coefficient follows a polynomial correlation:

    :math:`C_{D,sep} = (111.41 \\alpha^4 - 16.569 \\alpha^3 + 1.8 \\alpha^2 - 0.0242 \\alpha + 0.0015) \\cdot M_{correction}`

    where :math:`\\alpha` is the angle of attack in radians.

    For wings with segments, the total form drag is:

    :math:`C_{D,form,wing} = \\sum_{i=1}^{n-1} C_{D,sep,i} \\cdot S_{ref,i}`

    where :math:`S_{ref,i}` is the reference area of segment :math:`i`.

    For wings without segments:

    :math:`C_{D,form,wing} = C_{D,sep} \\cdot S_{ref,wing}`

    The total form drag coefficient is:

    :math:`C_{D,form} = \\frac{\\sum C_{D,form,wing}}{S_{ref}} \\cdot h_{00}(M)`

    where :math:`h_{00}(M)` is the cubic spline blending function.
    
    **Definitions**

    'Form Drag'
        Drag component caused by pressure differences due to flow separation and body shape.
    
    'Separation Drag'
        Additional drag caused by boundary layer separation from the surface.
    
    'Cubic Spline Blending'
        Smooth transition function between subsonic and supersonic aerodynamic regimes.

    References
    ----------
    [1] Empirical correlation for separation drag based on angle of attack comes from NASA CRM model
    [2] Unknown

    See Also
    --------
    RCAIDE.Library.Components.Wings.Vertical_Tail
    RCAIDE.Library.Methods.Utilities.Cubic_Spline_Blender
    """

    conditions       = state.conditions   
    Mach             = conditions.freestream.mach_number
    low_mach_cutoff  = settings.supersonic.begin_drag_rise_mach_number 
    high_mach_cutoff = settings.supersonic.end_drag_rise_mach_number 

    sub_spline = Cubic_Spline_Blender(low_mach_cutoff, high_mach_cutoff)  
    sub_h00    = lambda M:sub_spline.compute(M)
    
    alpha     = conditions.aerodynamics.angles.alpha 
    CD_form   = 0
    for wing in geometry.wings:
        M_correction = 2.9788*(Mach **3) - 6.4381*(Mach **2) + 4.4967*(Mach)  
        if type(wing) !=  RCAIDE.Library.Components.Wings.Vertical_Tail():
            CD_form_wing =  0 
            if len(wing.segments) > 0:
                CD_sep    = 0
                for i in  range((len(wing.segments) -1)):
                    segs          = list(wing.segments.keys())
                    segment       = wing.segments[segs[i]]
                    CD_sep        = (111.41* (alpha**4) - 16.569* (alpha**3) + 1.8* (alpha**2) - 0.0242*alpha + 0.0015)*M_correction
                    CD_form_wing  += CD_sep   * segment.areas.reference  
            else:
                CD_sep  = (111.41* (alpha**4) - 16.569* (alpha**3) + 1.8* (alpha**2) - 0.0242*alpha + 0.0015)*M_correction
                CD_form_wing = CD_sep * wing.areas.reference  
        
            CD_form += CD_form_wing /geometry.reference_area 
        
    state.conditions.aerodynamics.coefficients.drag.form.total = CD_form * sub_h00(Mach)
    return