# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/form_drag.py 
# 
# Created:  Jul 2025, M. Clarke 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Utilities         import Cubic_Spline_Blender
import  numpy as  np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Form Drag 
# ----------------------------------------------------------------------------------------------------------------------   
def form_drag(state,settings,geometry):
    """
    Computes the form drag coefficient associated with aircraft geometry and lift coefficient.

    Parameters
    ----------
    state : Data
        Flight conditions and aerodynamic state containing:
            - conditions.freestream.mach_number : float
                Freestream Mach number [unitless]
            - conditions.aerodynamics.coefficients.lift.total : float
                Vehicle lift coefficient, as predicted by this same VLM pipeline [unitless]
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
    This function calculates the form drag coefficient based on vehicle lift coefficient and
    Mach number effects. The calculation accounts for separation drag on wing segments and
    applies Mach number corrections for compressibility effects. The form drag is blended
    between subsonic and supersonic regimes using a cubic spline.

    **Major Assumptions**
        * Form drag is primarily due to wing geometry and lift coefficient
        * Separation drag follows a lookup table against CL, not angle of attack -- CL is the
          physically appropriate variable since separation onset tracks proximity to CL_max,
          and alpha-to-CL mapping is itself aircraft-dependent (AR, sweep, lift-curve slope),
          so an alpha-keyed correlation from one aircraft does not transfer to another
        * The lookup table is keyed on CL as predicted by RCAIDE's own VLM (not the true/
          experimental CL) since that is what is actually available to query at runtime --
          VLM has no stall model, so its CL keeps climbing linearly with alpha past the real
          aircraft's stall
        * Mach number correction is valid for typical transport aircraft
        * Vertical tail contributions are negligible
        * Blended Wing Body centerbody contributions are excluded -- this correlation is
          derived from a conventional swept wing (NASA CRM) and is not validated for a
          blended lifting-body centerbody's flow behavior
        * Cubic spline blending smooths transition between flight regimes
        * No aspect-ratio-based scaling is applied to other aircraft -- the correlation is
          only backed by a single reference aircraft, so it is used as-is rather than
          extrapolated via a fitted trend with no real supporting data

    **Theory**

    The Mach number correction factor is:

    :math:`M_{correction} = 2.9788 M^3 - 6.4381 M^2 + 4.4967 M`

    where :math:`M` is the freestream Mach number.

    The separation drag coefficient is read from a lookup table against CL:

    :math:`C_{D,sep} = \\text{interp}(C_L, \\, C_{L,table}, \\, C_{D,sep,table})`

    The table was derived by running RCAIDE's own VLM pipeline for the NASA CRM at the alpha/
    Mach/Reynolds conditions of its wind tunnel campaign, then taking, at each point:

    :math:`C_{D,form,needed} = C_{D,experiment} - (C_{D,total,RCAIDE} - C_{D,form,RCAIDE})`

    i.e. the residual between the wind-tunnel-measured total drag and RCAIDE's own current
    parasite + induced + compressibility + miscellaneous + cooling + trim buildup (everything
    *except* this form drag term), paired with RCAIDE's own VLM-predicted CL at that same
    condition. This keeps the table self-consistent with the rest of the drag buildup as it
    exists today, rather than reusing constants fit against a possibly-since-changed pipeline.

    Since the code below still re-applies the per-segment area weighting and Mach-spline
    scaling documented below, the stored table values are :math:`C_{D,form,needed}` divided
    by that same scaling as measured for CRM (a constant ~1.2293 over this Mach range), so
    that re-applying the scaling reproduces the intended total.

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
    [1] NASA CRM wind tunnel data (Mach ~0.85, Re ~5e7) and RCAIDE VLM validation, as embedded
        in RCAIDE_LEADS/VnV/Validation/aerodynamic_and_stability/test_CRM_aerodynamics.py

    See Also
    --------
    RCAIDE.Library.Components.Wings.Vertical_Tail
    RCAIDE.Library.Components.Wings.Blended_Wing_Body
    RCAIDE.Library.Methods.Utilities.Cubic_Spline_Blender
    """

    conditions       = state.conditions
    Mach             = conditions.freestream.mach_number
    CL               = conditions.aerodynamics.coefficients.lift.total
    high_mach_cutoff = settings.supersonic.end_drag_rise_mach_number
    low_mach_cutoff  = settings.supersonic.begin_drag_rise_mach_number
    CD_form          = np.zeros_like(Mach)

    # supersonic smoothing
    sup_spline = Cubic_Spline_Blender(low_mach_cutoff,high_mach_cutoff)
    sup_h00    = lambda M:sup_spline.compute(M)

    # CD_sep(CL) lookup table -- derived from NASA CRM wind tunnel data minus RCAIDE's own
    # current non-form drag buildup, at RCAIDE's own VLM-predicted CL (see Theory above)
    CD_sep_CL   = np.array([-0.26154624,-0.12047385,-0.05288686, 0.01460536, 0.08416023, 0.15828647,
                              0.21823000, 0.28480546, 0.35359477, 0.39051263, 0.42537388, 0.45681209,
                              0.48938546, 0.52689736, 0.55758345, 0.59050904, 0.62721824, 0.66113404,
                              0.69993884, 0.72890280, 0.76026821, 0.79556372, 0.82726587, 0.88702770,
                              0.96596717, 1.03323402, 1.08964067, 1.14958867, 1.20849892, 1.28677187,
                              1.34415649, 1.39433542, 1.49393924])
    CD_sep_data = np.array([ 0.00123614,-0.00187464,-0.00183041,-0.00121211,-0.00012013, 0.00064925,
                              0.00072570, 0.00098505, 0.00151466, 0.00191563, 0.00172489, 0.00140788,
                              0.00129481, 0.00146245, 0.00181780, 0.00258879, 0.00378745, 0.00508128,
                              0.00647820, 0.00795086, 0.00935953, 0.01079466, 0.01154840, 0.01134654,
                              0.01136504, 0.01303240, 0.01495724, 0.01845308, 0.02204719, 0.02544638,
                              0.02673925, 0.02979446, 0.03389191])


    for wing in geometry.wings:
        is_bwb = isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body)
        if not wing.vertical and not is_bwb:
            CD_form_wing = 0
            CD_sep       = np.interp(CL, CD_sep_CL, CD_sep_data)
            segs = list(wing.segments.keys())
            for i in range(len(wing.segments) - 1):
                CD_form_wing += CD_sep * wing.segments[segs[i]].areas.reference
            CD_form += CD_form_wing * sup_h00(Mach) / geometry.reference_area

    state.conditions.aerodynamics.coefficients.drag.form.total = CD_form
    return