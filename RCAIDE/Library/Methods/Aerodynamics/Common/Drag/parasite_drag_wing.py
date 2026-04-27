# RCAIDE/Methods/Aerodynamics/Common/Drag/parasite_drag_wing.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
from RCAIDE.Framework.Core import Data  
from RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressible_mixed_flat_plate import compressible_mixed_flat_plate
from RCAIDE.Library.Methods.Utilities         import Cubic_Spline_Blender  
 
# package imports
import numpy as np  

# ----------------------------------------------------------------------------------------------------------------------  
#   Parasite Drag Wing 
# ----------------------------------------------------------------------------------------------------------------------   
def parasite_drag_wing(state,settings,geometry):
    """
    Computes the parasite drag coefficient for wings accounting for segments and compressibility effects.

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
            - wing_parasite_drag_form_factor : float
                Form factor for wing parasite drag [unitless]
    geometry : Data
        Wing geometry containing:
            - tag : str
                Unique identifier for the wing
            - thickness_to_chord : float
                Thickness-to-chord ratio [unitless]
            - areas.reference : float
                Reference area of the wing [m²]
            - areas.wetted : float
                Wetted area of the wing [m²]
            - transition_x_upper : float
                Upper surface transition point as fraction of chord [unitless]
            - transition_x_lower : float
                Lower surface transition point as fraction of chord [unitless]
            - segments : dict, optional
                Dictionary of wing segments containing:
                    - thickness_to_chord : float
                        Thickness-to-chord ratio of segment [unitless]
                    - chords.mean_aerodynamic : float
                        Mean aerodynamic chord of segment [m]
                    - areas.reference : float
                        Reference area of segment [m²]
                    - areas.wetted : float
                        Wetted area of segment [m²]
                    - sweeps.leading_edge : float
                        Leading edge sweep angle of segment [radians]
            - chords.mean_aerodynamic : float
                Mean aerodynamic chord of wing [m]
            - sweeps.leading_edge : float
                Leading edge sweep angle of wing [radians]

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.parasite[wing.tag]

    Notes
    -----
    This function calculates the parasite drag coefficient for wings using compressible
    mixed flat plate theory with form factor corrections. The calculation handles both
    segmented and non-segmented wings, accounting for different transition points on
    upper and lower surfaces.
    
    **Major Assumptions**
        * Mixed laminar-turbulent boundary layer with specified transition points
        * Compressible flat plate skin friction correlation
        * Form factor accounts for airfoil thickness and sweep effects
        * Cubic spline blending smooths transition between subsonic and supersonic regimes
        * Segmented wings are analyzed segment by segment and area-weighted
    
    **Theory**

    For segmented wings, the total parasite drag is area-weighted:

    :math:`C_{D,parasite} = \\frac{\\sum_{i=1}^{n-1} C_{D,i} \\cdot S_{ref,i}}{S_{ref,total}}`

    where each segment's drag coefficient is calculated using the compute_parasite_drag function.

    For non-segmented wings, the drag is calculated directly:

    :math:`C_{D,parasite} = \\frac{C_{f,upper} + C_{f,lower}}{2} \\cdot k_w \\cdot \\frac{S_{wet}}{S_{ref}}`

    where:
        - :math:`C_{f,upper}` and :math:`C_{f,lower}` are the skin friction coefficients
        - :math:`k_w` is the form factor
        - :math:`S_{wet}` and :math:`S_{ref}` are the wetted and reference areas
    
    **Definitions**
    
    'Form Factor'
        Multiplier accounting for the increase in drag due to airfoil shape compared to a flat plate.

    References
    ----------
    [1] Stanford AA241 Course Notes. http://aerodesign.stanford.edu/aircraftdesign/aircraftdesign.html

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compute_parasite_drag
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressible_mixed_flat_plate
    """
    
    # unpack inputs
    C             = settings.wing_parasite_drag_form_factor  
    freestream    = state.conditions.freestream 
    Mc            = freestream.mach_number
    Tc            = freestream.temperature    
    re            = freestream.reynolds_number 
    wing          = geometry 
    Sref          = wing.areas.reference   
    num_segments  = len(wing.segments.keys())

    wing_parasite_drag = 0.0     
    
    # if wing has segments, compute and sum parasite drag of each segment 
    xtu       = wing.transition_x_upper
    xtl       = wing.transition_x_lower     
     
    seg_tags                     = list(wing.segments.keys())
    total_segment_parasite_drag  = 0 
    total_segment_k_w            = 0 
    total_segment_cf_w_u         = 0
    total_segment_cf_w_l         = 0 
    total_segment_k_comp_u       = 0
    total_segment_k_comp_l       = 0
    total_k_reyn_u               = 0          
    total_k_reyn_l               = 0
    
    for i,segment in enumerate(wing.segments): 
        if i == num_segments-1:               
            continue 
        avg_t_c_s     = (wing.segments[seg_tags[i]].thickness_to_chord + wing.segments[seg_tags[i+1]].thickness_to_chord)/2              
        mac_seg       = segment.chords.mean_aerodynamic
        Sref_seg      = segment.areas.reference
        Swet_seg      = segment.areas.wetted
        sweep_seg     = segment.sweeps.leading_edge  

        # compute parasite drag coef., form factor, skin friction coef., compressibility factor and reynolds number for segments
        segment_parasite_drag , segment_k_w, segment_cf_w_u, segment_cf_w_l, segment_k_comp_u, segment_k_comp_l, k_reyn_u ,k_reyn_l = compute_parasite_drag(re,mac_seg,Mc,Tc,xtu,xtl,sweep_seg,avg_t_c_s,Sref_seg,Swet_seg,C)
        
        total_segment_parasite_drag  += segment_parasite_drag*Sref_seg   
        total_segment_k_w            += segment_k_w*Sref_seg 
        total_segment_cf_w_u         += segment_cf_w_u*Sref_seg 
        total_segment_cf_w_l         += segment_cf_w_l*Sref_seg 
        total_segment_k_comp_u       += segment_k_comp_u*Sref_seg 
        total_segment_k_comp_l       += segment_k_comp_l*Sref_seg 
        total_k_reyn_u               += k_reyn_u*Sref_seg                 
        total_k_reyn_l               += k_reyn_l*Sref_seg  
            
    wing_parasite_drag = total_segment_parasite_drag  / Sref
    k_w                = total_segment_k_w / Sref
    cf_w_u             = total_segment_cf_w_u  / Sref
    cf_w_l             = total_segment_cf_w_l / Sref
    k_comp_u           = total_segment_k_comp_u  / Sref
    k_comp_l           = total_segment_k_comp_l  / Sref
    k_reyn_u           = total_k_reyn_u  / Sref
    k_reyn_l           = total_k_reyn_l  / Sref
        

    # dump data to conditions
    wing_result = Data(
        wetted_area               = wing.areas.wetted,
        reference_area            = Sref   , 
        total                     = wing_parasite_drag ,
        skin_friction             = (cf_w_u+cf_w_l)/2.   ,
        compressibility_factor    = (k_comp_u+k_comp_l)/2 ,
        reynolds_factor           = (k_reyn_u+k_reyn_l)/2 , 
        form_factor               = k_w    ,
    )
    
    state.conditions.aerodynamics.coefficients.drag.parasite[wing.tag] = wing_result

    return  
 
def compute_parasite_drag(re,mac_w,Mc,Tc,xtu,xtl,sweep_w,t_c_w,Sref,Swet,C):
    """
    Computes the parasite drag coefficient for a wing section using compressible mixed flat plate theory.

    Parameters
    ----------
    re : float
        Freestream Reynolds number per unit length [unitless/m]
    mac_w : float
        Mean aerodynamic chord of wing section [m]
    Mc : float
        Freestream Mach number [unitless]
    Tc : float
        Freestream static temperature [K]
    xtu : float
        Upper surface transition point as fraction of chord [unitless]
    xtl : float
        Lower surface transition point as fraction of chord [unitless]
    sweep_w : float
        Leading edge sweep angle [radians]
    t_c_w : float
        Thickness-to-chord ratio [unitless]
    Sref : float
        Reference area of wing section [m²]
    Swet : float
        Wetted area of wing section [m²]
    C : float
        Form factor coefficient [unitless]

    Returns
    -------
    wing_parasite_drag : float
        Parasite drag coefficient [unitless]
    k_w : float
        Form factor [unitless]
    cf_w_u : float
        Upper surface skin friction coefficient [unitless]
    cf_w_l : float
        Lower surface skin friction coefficient [unitless]
    k_comp_u : float
        Upper surface compressibility factor [unitless]
    k_comp_l : float
        Lower surface compressibility factor [unitless]
    k_reyn_u : float
        Upper surface Reynolds number factor [unitless]
    k_reyn_l : float
        Lower surface Reynolds number factor [unitless]

    Notes
    -----
    This function calculates the parasite drag coefficient for a wing section using
    compressible mixed flat plate theory with separate analysis of upper and lower
    surfaces. The form factor accounts for airfoil thickness and sweep effects.
    
    **Major Assumptions**
        * Mixed laminar-turbulent boundary layer with different transition points
        * Compressible flat plate skin friction correlation
        * Form factor accounts for airfoil thickness and sweep effects
        * Cubic spline blending for transonic regime
        * Upper and lower surfaces are analyzed separately
    
    **Theory**

    The wing Reynolds number is:

    :math:`Re_w = Re \\cdot MAC`

    where :math:`Re` is the freestream Reynolds number per unit length and :math:`MAC` is the mean aerodynamic chord.

    The skin friction coefficients are calculated separately for upper and lower surfaces:

    :math:`C_{f,upper} = f(Re_w, M, T, x_{tu})`

    :math:`C_{f,lower} = f(Re_w, M, T, x_{tl})`

    The form factor for subsonic flow (M ≤ 1.0) is:

    :math:`k_w = 1 + \\frac{2C(t/c)\\cos^2(\\Lambda)}{\\beta} + \\frac{C^2\\cos^2(\\Lambda)(t/c)^2(1+5\\cos^2(\\Lambda))}{2\\beta^2}`

    where:
        - :math:`\\beta = \\sqrt{1-(M\\cos(\\Lambda))^2}` is the Prandtl-Glauert factor
        - :math:`\\Lambda` is the leading edge sweep angle
        - :math:`t/c` is the thickness-to-chord ratio

    For transonic flow, the form factor is blended using a cubic spline:

    :math:`k_w = k_w \\cdot h_{00}(M) + 1 \\cdot (1-h_{00}(M))`

    The parasite drag coefficient is:

    :math:`C_{D,parasite} = \\frac{C_{f,upper} + C_{f,lower}}{2} \\cdot k_w \\cdot \\frac{S_{wet}}{S_{ref}}`
    
    **Definitions**

    'Form Factor'
        Multiplier accounting for the increase in drag due to airfoil shape compared to a flat plate.
    
    'Mixed Boundary Layer'
        Boundary layer that transitions from laminar to turbulent flow at specified points.

    References
    ----------
    [1] Stanford AA241 Course Notes. adg.stanford.edu

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressible_mixed_flat_plate
    RCAIDE.Library.Methods.Utilities.Cubic_Spline_Blender
    """
   
    # reynolds number
    Re_w = re*mac_w  
    
    # skin friction  coefficient, upper
    cf_w_u, k_comp_u, k_reyn_u = compressible_mixed_flat_plate(Re_w,Mc,Tc,xtu)
    
    # skin friction  coefficient, lower
    cf_w_l, k_comp_l, k_reyn_l = compressible_mixed_flat_plate(Re_w,Mc,Tc,xtl) 
    
    # correction for airfoils
    cos_sweep = np.cos(sweep_w)
    cos2      = cos_sweep*cos_sweep
    
    ind = Mc <= 1.
    
    k_w = np.ones_like(Mc)
    beta   =  ( np.sqrt(1.-(Mc[ind]*cos_sweep)**2.) )
    k_w[ind] = 1. + ( 2.* C * (t_c_w * cos2) ) /beta \
                  + (( C**2) * cos2 * (t_c_w** 2) * (1. + 5.*(cos2)) ) / (2.* beta ** 2)             
    
    spline = Cubic_Spline_Blender(.95,1.0)
    h00 = lambda M:spline.compute(M)
    
    k_w = k_w*(h00(Mc)) + 1*(1-h00(Mc))   

    # find the final result 
    wing_parasite_drag = ( cf_w_u + cf_w_l)/2  * k_w  * (Swet / Sref)


    return wing_parasite_drag , k_w, cf_w_u, cf_w_l, k_comp_u, k_comp_l, k_reyn_u, k_reyn_l