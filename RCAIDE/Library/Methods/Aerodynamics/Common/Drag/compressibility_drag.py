# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/compressibility_drag.py
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Utilities         import Cubic_Spline_Blender 

# package imports
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Compressibility Drag Total
# ----------------------------------------------------------------------------------------------------------------------  
def compressibility_drag(state,settings,geometry):
    """
    Computes compressibility drag coefficient for full aircraft including volume drag effects.

    Parameters
    ----------
    state : Data
        Flight conditions and aerodynamic state containing:
            - conditions.freestream.mach_number : float
                Freestream Mach number [unitless]
            - conditions.aerodynamics.coefficients.lift.total : float
                Total lift coefficient [unitless]
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
                  - sweeps.leading_edge : float
                      Leading edge sweep angle [radians]
                  - thickness_to_chord : float
                      Thickness-to-chord ratio [unitless]

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.compressible.total

    Notes
    -----
    This function calculates the compressibility drag coefficient using empirical correlations
    based on wing geometry and flight conditions. The calculation accounts for the critical
    Mach number and the drag rise characteristics of swept wings.
    
    **Major Assumptions**
        * Compressibility effects are primarily due to wing geometry
        * Critical Mach number correlation is valid for typical transport aircraft
        * Cubic spline blending smooths transition between subsonic and supersonic regimes
    
    **Theory**

    The critical Mach number is calculated using a regression fit from AA241:

    :math:`M_{cc} \\cos(\\Lambda) = 0.922 - 1.154(t/c) - 0.305 C_L + 0.333(t/c)^2 + 0.467(t/c)C_L + 0.087 C_L^2`

    where:
      - :math:`M_{cc}` is the critical Mach number
      - :math:`\\Lambda` is the leading edge sweep angle [radians]
      - :math:`t/c` is the thickness-to-chord ratio corrected for sweep
      - :math:`C_L` is the lift coefficient corrected for sweep

    The corrected thickness-to-chord ratio is:

    :math:`(t/c)_{eff} = \\frac{t/c}{\\cos(\\Lambda)}`

    The corrected lift coefficient is:

    :math:`C_{L,eff} = \\frac{C_L}{\\cos^2(\\Lambda)}`

    The divergence ratio is:

    :math:`M/M_{cc} = \\frac{M}{M_{cc}}`

    The compressibility drag coefficient follows Shevell's correlation:

    :math:`\\Delta C_{D,comp} = 0.0019 \\left(\\frac{M}{M_{cc}}\\right)^{14.641} \\cos^3(\\Lambda)`
    
    **Definitions**

    'Compressibility Drag'
        Additional drag caused by compressibility effects as the aircraft approaches and exceeds the critical Mach number.
    
    'Critical Mach Number'
        The freestream Mach number at which the local flow over some part of the aircraft first reaches sonic velocity.
    
    'Drag Rise'
        Rapid increase in drag coefficient as the aircraft approaches and exceeds the critical Mach number.

    References
    ----------
    [1] Stanford AA241 Lecture Notes
    [2] Shevell, R. S. (1989). "Fundamentals of Flight." Prentice Hall.

    See Also
    --------
    RCAIDE.Library.Components.Wings.Main_Wing
    RCAIDE.Library.Components.Wings.Blended_Wing_Body
    RCAIDE.Library.Methods.Utilities.Cubic_Spline_Blender
    """

    # Unpack
    conditions       = state.conditions
    Mach             = conditions.freestream.mach_number  
    low_mach_cutoff  = settings.supersonic.begin_drag_rise_mach_number 
    high_mach_cutoff = settings.supersonic.end_drag_rise_mach_number
   
    sub_spline = Cubic_Spline_Blender(low_mach_cutoff, high_mach_cutoff)  
    sub_h00    = lambda M:sub_spline.compute(M)   

    cd_compressibility  = np.zeros_like(Mach)
    cl                  = conditions.aerodynamics.coefficients.lift.total
    for wing in  geometry.wings:
        if isinstance(wing,RCAIDE.Library.Components.Wings.Main_Wing) or isinstance(wing,RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            sweep_w   = wing.sweeps.leading_edge 
    
            # Get effective CLift_wings and sweep
            tc = wing.thickness_to_chord / np.cos(sweep_w)
            cl = conditions.aerodynamics.coefficients.lift.total/ (np.cos(sweep_w) ** 2)
    
            # Compressibility drag based on regressed fits from AA241 
            mcc_cos_ws = 0.922321524499352  - 1.153885166170620*tc  - 0.304541067183461*cl    \
                           + 0.332881324404729*tc*tc  + 0.467317361111105*tc*cl   + 0.087490431201549*cl*cl
    
            # Crest-critical Mach number, corrected for wing sweep
            Mcc = mcc_cos_ws/ np.cos(sweep_w)      
    
            # Divergence ratio
            mo_Mach = Mach/Mcc
    
            # Compressibility correlation, Shevell
            dcdc_cos3g = 0.0019*mo_Mach**14.641
    
            # Compressibility drag  
            cd_compressibility = dcdc_cos3g * (np.cos(sweep_w)**3)  
 
    cd_comp  = cd_compressibility*(sub_h00(Mach))   
   
    # store results  
    conditions.aerodynamics.coefficients.drag.compressible.total = cd_comp

    return  