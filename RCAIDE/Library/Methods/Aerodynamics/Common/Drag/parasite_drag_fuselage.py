# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/subsonic_parasite_drag_fuselage.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE import
import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressible_turbulent_flat_plate import compressible_turbulent_flat_plate
from RCAIDE.Library.Methods.Utilities         import Cubic_Spline_Blender   

# python imports 
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#   Parasite Drag Fuselage
# ---------------------------------------------------------------------------------------------------------------------- 
def parasite_drag_fuselage(state,settings,fuselage):
    """
    Computes the parasite drag coefficient for a fuselage or boom accounting for compressibility effects.

    Parameters
    ----------
    state : Data
        Flight conditions containing:
            - conditions.freestream.mach_number : float
                Freestream Mach number [unitless]
            - conditions.freestream.temperature : float
                Freestream static temperature [K]
            - conditions.freestream.reynolds_number : float
                Freestream Reynolds number per unit length [unitless/m]
    settings : dict
        Aerodynamic analysis settings containing:
            - fuselage_parasite_drag_form_factor : float
                Form factor for fuselage parasite drag [unitless]
            - supersonic.fuselage_parasite_drag_begin_blend_mach : float
                Mach number at which supersonic blending begins [unitless]
            - supersonic.fuselage_parasite_drag_end_blend_mach : float
                Mach number at which supersonic blending ends [unitless]
    fuselage : Data
        Fuselage geometry containing:
            - tag : str
                Unique identifier for the fuselage
            - areas.front_projected : float
                Front projected area [m²]
            - areas.wetted : float
                Wetted area [m²]
            - lengths.total : float
                Total length of fuselage [m]
            - effective_diameter : float
                Effective diameter of fuselage [m]

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.parasite[fuselage.tag]

    Notes
    -----
    This function calculates the parasite drag coefficient for a fuselage or boom using
    compressible turbulent flat plate theory with form factor corrections. The calculation
    accounts for compressibility effects through Mach number-dependent form factors and
    uses cubic spline blending for the transonic regime.
    
    **Major Assumptions**
        * Fully turbulent boundary layer over the entire fuselage
        * Cylindrical body approximation for form factor calculations
        * Compressible turbulent flat plate skin friction correlation
        * Cubic spline blending smooths transition between subsonic and supersonic regimes
        * Form factor accounts for pressure drag due to body shape
    
    **Theory**

    The fuselage Reynolds number is:

    :math:`Re_{fus} = Re \\cdot l_{fus}`

    where :math:`Re` is the freestream Reynolds number per unit length and :math:`l_{fus}` is the fuselage length.

    The skin friction coefficient is calculated using compressible turbulent flat plate theory:

    :math:`C_f = f(Re_{fus}, M, T)`

    The diameter-to-length ratio is:

    :math:`d/l = \\frac{d_{fus}}{l_{fus}}`

    For subsonic flow (M ≤ 1.0), the form factor parameters are:

    :math:`D = \\sqrt{1 - (1-M^2)(d/l)^2}` for M < 0.95

    :math:`D = \\sqrt{1 - (d/l)^2}` for M ≥ 0.95

    :math:`a = \\frac{2(1-M^2)(d/l)^2(\\text{arctanh}(D)-D)}{D^3}` for M < 0.95

    :math:`a = \\frac{2(d/l)^2(\\text{arctanh}(D)-D)}{D^3}` for M ≥ 0.95

    The maximum velocity perturbation is:

    :math:`\\frac{\\Delta u_{max}}{u_{\\infty}} = \\frac{a}{(2-a)\\sqrt{1-M^2}}` for M < 0.95

    :math:`\\frac{\\Delta u_{max}}{u_{\\infty}} = \\frac{a}{2-a}` for M ≥ 0.95

    The form factor is:

    :math:`k_{fus} = (1 + FF \\cdot \\frac{\\Delta u_{max}}{u_{\\infty}})^2`

    where :math:`FF` is the user-specified form factor.

    For supersonic flow, the form factor is calculated using cubic spline blending between
    subsonic and supersonic correlations.

    The parasite drag coefficient is:

    :math:`C_{D,parasite} = k_{fus} \\cdot C_f \\cdot \\frac{S_{wet}}{S_{ref}}`
    
    **Definitions**

    'Parasite Drag'
        Drag component caused by viscous effects and pressure forces on the aircraft surface.
    
    'Form Factor'
        Multiplier accounting for the increase in drag due to body shape compared to a flat plate.
    
    'Compressibility Effects'
        Changes in aerodynamic characteristics due to compressible flow effects at high Mach numbers.

    References
    ----------
    [1] Stanford AA241 Course Notes

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressible_turbulent_flat_plate
    RCAIDE.Library.Methods.Utilities.Cubic_Spline_Blender
    """
     
    # unpack inputs   
    Sref          = fuselage.areas.front_projected
    Swet          = fuselage.areas.wetted 
    l_fus         = fuselage.lengths.total
    d_fus         = fuselage.effective_diameter   
    form_factor   = settings.fuselage_parasite_drag_form_factor 
    low_cutoff    = settings.supersonic.fuselage_parasite_drag_begin_blend_mach
    high_cutoff   = settings.supersonic.fuselage_parasite_drag_end_blend_mach  
    Mach          = state.conditions.freestream.mach_number
    T             = state.conditions.freestream.temperature    
    Re            = state.conditions.freestream.reynolds_number 

    # Reynolds number
    Re_fus = Re*(l_fus)
    
    # skin friction coefficient
    cf_fus, k_comp, k_reyn = compressible_turbulent_flat_plate(Re_fus,Mach,T)       
    d_d = float(d_fus)/float(l_fus) 
 
    if np.all((Mach<=1.0) == True): 
        # compute form factor for cylindrical bodies 
        D             = np.zeros_like(Mach)    
        D[Mach < 0.95]  = np.sqrt(1 - (1-Mach[Mach < 0.95]**2) * d_d**2)
        D[Mach >= 0.95] = np.sqrt(1 - d_d**2)
    
        a             = np.zeros_like(Mach)    
        a[Mach < 0.95]  = 2 * (1-Mach[Mach < 0.95]**2) * (d_d**2) *(np.arctanh(D[Mach < 0.95])-D[Mach < 0.95]) / (D[Mach < 0.95]**3)
        a[Mach >= 0.95] = 2  * (d_d**2) *(np.arctanh(D[Mach >= 0.95])-D[Mach >= 0.95]) / (D[Mach >= 0.95]**3)
    
        du_max_u               = np.zeros_like(Mach)    
        du_max_u[Mach < 0.95]  = a[Mach < 0.95] / ( (2-a[Mach < 0.95]) * (1-Mach[Mach < 0.95]**2)**0.5 ) 
        du_max_u[Mach >= 0.95] = a[Mach >= 0.95] / ( (2-a[Mach >= 0.95]) )
        
        k_fus                  = (1 + form_factor*du_max_u)**2 
        fuselage_parasite_drag = k_fus * cf_fus * Swet / Sref
        
    else: 
        
        # supersonic condition  
        D_low        = np.zeros_like(Mach)
        a_low        = np.zeros_like(Mach)
        du_max_u_low = np.zeros_like(Mach)
        
        D_high        = np.zeros_like(Mach)
        a_high        = np.zeros_like(Mach)
        du_max_u_high = np.zeros_like(Mach) 
        k_fus         = np.zeros_like(Mach)
        
        low_inds      = Mach < high_cutoff
        high_inds     = Mach > low_cutoff
        
        D_low[low_inds]        = np.sqrt(1 - (1-Mach[low_inds]**2) * d_d**2)
        a_low[low_inds]        = 2 * (1-Mach[low_inds]**2) * (d_d**2) *(np.arctanh(D_low[low_inds])-D_low[low_inds]) / (D_low[low_inds]**3)
        du_max_u_low[low_inds] = a_low[low_inds] / ( (2-a_low[low_inds]) * (1-Mach[low_inds]**2)**0.5 )
        
        D_high[high_inds]        = np.sqrt(1 - d_d**2)
        a_high[high_inds]        = 2  * (d_d**2) *(np.arctanh(D_high[high_inds])-D_high[high_inds]) / (D_high[high_inds]**3)
        du_max_u_high[high_inds] = a_high[high_inds] / ( (2-a_high[high_inds]) )
        
        spline = Cubic_Spline_Blender(low_cutoff,high_cutoff)
        h00    = lambda M:spline.compute(M)
        
        du_max_u = du_max_u_low*(h00(Mach)) + du_max_u_high*(1-h00(Mach))    
        
        k_fus = (1 + form_factor*du_max_u)**2
    
        fuselage_parasite_drag = k_fus * cf_fus * Swet / Sref
         
    # Store data 
    state.conditions.aerodynamics.coefficients.drag.parasite[fuselage.tag] = Data(
        wetted_area               = Swet   , 
        reference_area            = Sref   , 
        total                     = fuselage_parasite_drag ,
        skin_friction             = cf_fus ,
        compressibility_factor    = k_comp ,
        reynolds_factor           = k_reyn , 
        form_factor               = k_fus  ,
    )    
        
    return  