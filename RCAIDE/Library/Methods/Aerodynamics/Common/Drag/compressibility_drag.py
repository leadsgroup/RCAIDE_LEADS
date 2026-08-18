# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/compressibility_drag.py
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Methods.Utilities         import Cubic_Spline_Blender  
from RCAIDE.Library.Components.Wings          import Main_Wing 

# package imports
import numpy as np
from scipy.interpolate import RegularGridInterpolator
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
    conditions           = state.conditions
    Mach                 = conditions.freestream.mach_number  
    sup_low_mach_cutoff  = settings.supersonic.begin_drag_rise_mach_number 
    sup_high_mach_cutoff = settings.supersonic.end_drag_rise_mach_number
    sub_low_mach_cutoff  = settings.subsonic.begin_transonic_rise_mach_number      
    sub_high_mach_cutoff = settings.subsonic.end_transonic_rise_rise_mach_number    
    
    # functions smoothing 
    sup_spline = Cubic_Spline_Blender(sup_low_mach_cutoff,sup_high_mach_cutoff) 
    sup_h00    = lambda M:sup_spline.compute(M)  
    sub_spline = Cubic_Spline_Blender(sub_low_mach_cutoff, sub_high_mach_cutoff)  
    sub_h00    = lambda M:sub_spline.compute(M)   

    subsonic_CDc          = subsonic_compressibility_drag(state,settings,geometry) 
    transonic_CDw_lift    = transonic_lift_wave_drag(conditions, settings, geometry) 
    supersonic_CDw_volume = supersonic_volume_wave_drag(conditions, settings, geometry) 
    supersonic_CDw_lift   = supersonic_lift_wave_drag(conditions, settings, geometry)
    
    # Apply smoothing functions
    sub_CDc          = subsonic_CDc *(sub_h00(Mach)) 
    trans_CDw_lift   = transonic_CDw_lift  * (1-sub_h00(Mach))  
    sup_CDw_lift     = supersonic_CDw_lift  * (1-sup_h00(Mach))
    sup_CDw_volume   = supersonic_CDw_volume * (1-sup_h00(Mach)) 
    
    # compute totals 
    total_CDc        = sup_CDw_lift +  trans_CDw_lift  +  sup_CDw_volume  +  sub_CDc
    total_CDw        = sup_CDw_lift  + sup_CDw_volume  + trans_CDw_lift   
    total_CDw_lift   = sup_CDw_lift  + trans_CDw_lift
    
    # store results  
    conditions.aerodynamics.coefficients.drag.compressible.total[:,0]           = total_CDc[:,0]   
    conditions.aerodynamics.coefficients.drag.compressible.wave.total[:,0]      = total_CDw[:,0]             
    conditions.aerodynamics.coefficients.drag.compressible.wave.volume[:,0]     = sup_CDw_volume[:,0]                
    conditions.aerodynamics.coefficients.drag.compressible.wave.lift[:,0]       = total_CDw_lift[:,0]              

    return  

def subsonic_compressibility_drag(state,settings,geometry):
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
    conditions          = state.conditions
    Mach                = conditions.freestream.mach_number    
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
 
    cd_comp  = cd_compressibility 

    return  cd_comp


# ---------------------------------------------------------------------------------------------------------------------- 
#  transonic_wave_drag
# ----------------------------------------------------------------------------------------------------------------------
def transonic_lift_wave_drag(conditions, settings, geometry): 
    """
    Computes transonic lift wave drag coefficient using empirical correlations.

    Parameters
    ----------
    conditions : Data
        Flight conditions containing:
            - freestream.mach_number : float
                Freestream Mach number [unitless]
            - aerodynamics.coefficients.lift.total : float
                Total lift coefficient [unitless]
            - aerodynamics.coefficients.lift.spanwise : float, optional
                Spanwise lift distribution [unitless]
    settings : dict
        Analysis settings containing:
            - use_surrogate : bool
                Flag to use surrogate model
            - vortex_distribution : Data, optional
                Vortex distribution data containing:
                - chord_lengths : array
                    Chord lengths at spanwise stations [m]
                - leading_edge_sweeps : array
                    Leading edge sweep angles [radians]
                - chord_widths : array
                    Chord widths at spanwise stations [m]
    geometry : Data
        Aircraft geometry containing:
            - reference_area : float
                Reference area [m²]

    Returns
    -------
    CD_wave_transonic : float
        Transonic lift wave drag coefficient [unitless]

    Notes
    -----
    This function calculates the transonic lift wave drag using either a surrogate
    model or detailed spanwise analysis based on shock wave formation from Lock (1986). A generic
    airfoil (RAE 5225) is assumed for the Cp data.
    
    **Major Assumptions**
        * Surrogate model is valid for Mach 0.7-0.95 range
        * Shock wave formation depends on local pressure coefficient
        * Normalized curvature factor is constant (0.23)
        * Spanwise analysis accounts for sweep effects
    
    **Theory**

    For surrogate model:
    :math:`C_{D,wave} = f(C_L) \\cdot (12.5M - 8.75)`

    For detailed analysis:
    :math:`C_{D,wave} = \\sum_{i=1}^{n} C_{D,wave,i} \\cdot \\frac{S_i}{S_{ref}}`

    where each segment's wave drag is:
    :math:`C_{D,wave,i} = \\frac{\\cos^4(\\Lambda)}{\\kappa} \\cdot 0.243 \\cdot \\left(\\frac{1+0.2M\\cos(\\Lambda)}{M\\cos(\\Lambda)}\\right)^3 \\cdot (M_1^* - 1)^4 \\cdot \\frac{2-M_1^*}{M_1^*(1+0.2M_1^{*2})} \\cdot 0.5`

    The local Mach number before shock is:
    :math:`M_1^* = \\sqrt{\\frac{5+M^2\\cos^2(\\Lambda)}{(1+0.7M^2C_p)^{2/7}} - 5}`
    
    **Definitions**

    'Transonic Wave Drag'
        Wave drag occurring in the transonic regime (M ≈ 0.7-0.95).
    
    'Shock Wave'
        Discontinuity in flow properties caused by compressibility effects.
    
    'Pressure Coefficient'
        Dimensionless pressure difference normalized by dynamic pressure.

    References
    ----------
    [1] Lock, R. C. (1986). "The Prediction of the Drag of Aerofoils and Wings at High Subsonic Speeds." Aeronautical Journal.
    [2] Harris, C. D. (1990). "NASA Supercritical Airfoils." NASA TP 2969.

    """
    Mach              = conditions.freestream.mach_number  
    S_ref             = geometry.reference_area 
    CD_wave_transonic = np.zeros_like(Mach) 
    CL_y              = conditions.aerodynamics.coefficients.lift.spanwise *geometry.wings.main_wing.areas.projected/S_ref

    chords   = settings.vortex_distribution.chord_lengths
    delta    = settings.vortex_distribution.leading_edge_sweeps 
    dy       = settings.vortex_distribution.chord_widths
    
    CD_wave_transonic = np.zeros_like(Mach)
    c_kappa           = 0.23 # normalized curvature of the airfoil. This can eventually be calcualted using airfoil shape data. 

    # ------------------------------------------------------------------
    # Cp Data (as function of CL) from "The Prediciton of the Drag of Aerofoils and Wings at High Subsonic Speeds" by R.C. Lock, 1986, Aeronautical journal and NASA TP 2969, NASA Supercritical Airfoils by Charles D. Harris
    # ------------------------------------------------------------------
                
    Cp_shock =  0.9825 *(CL_y**2)  - 2.5132 *(CL_y) + 0.0279 # Cp vlaue right before the shock wave 
    # ------------------------------------------------------------------
    # Wave Drag Calculation
    # ------------------------------------------------------------------

    # Calculate the local mach number right before the shock wave
    M1_0_star = np.sqrt((5+Mach**2*np.cos(delta[0])**2)/(1+0.7*Mach**2*Cp_shock)**(2/7) - 5)
    
    # Calculate the wave drag coefficient
    constant = (np.cos(delta[0])**4)/c_kappa * 0.243*((1+0.2*Mach*np.cos(delta[0]))/(Mach*np.cos(delta[0])))**3 # note, delta is assumed to remain constant across all conditions. 
    CD_wave_segment = constant*(M1_0_star - 1)**4*(2-M1_0_star)/(M1_0_star*(1+0.2*M1_0_star**2)) * 0.5
    
    # Set the wave drag coefficient to 0 if there is no shock wave
    CD_wave_segment[Cp_shock > -0.6] = 0.0 # If the Cp > -0.6 then there is likely no shock wave

    S_segment        = chords[0]*dy[0]
    CD_wave_transonic = np.sum(CD_wave_segment*S_segment/S_ref, axis=1, keepdims=True) 
        
    CD_wave_transonic[Mach>1] = 0.0
    return CD_wave_transonic 
 
# ---------------------------------------------------------------------------------------------------------------------- 
# supersonic_lift_wave_drag
# ---------------------------------------------------------------------------------------------------------------------- 
def supersonic_lift_wave_drag(conditions,configuration,geometry):
    """
    Computes supersonic lift wave drag coefficient using JAXA methodology.

    Parameters
    ----------
    conditions : Data
        Flight conditions containing:
            - freestream.mach_number : float
                Freestream Mach number [unitless]
            - aerodynamics.coefficients.lift.total : float
                Total lift coefficient [unitless]
    configuration : dict
        Aircraft configuration settings
    geometry : Data
        Aircraft geometry containing:
            - wings : list
                List of wing objects containing:
                    - chords.root : float
                        Root chord length [m]
                    - spans.projected : float
                        Projected span [m]
                    - aspect_ratio : float
                        Aspect ratio [unitless]

    Returns
    -------
    cd_lift_wave : float
        Supersonic lift wave drag coefficient [unitless]

    Notes
    -----
    This function calculates the supersonic lift wave drag using the JAXA methodology
    based on wing geometry and lift coefficient.
    
    **Major Assumptions**
        * JAXA methodology is valid for supersonic speeds
        * Only main wing contributes to lift wave drag
        * Wing geometry parameters are sufficient for calculation
        * Empirical correlation is valid for typical supersonic aircraft
    
    **Theory**

    The supersonic lift wave drag coefficient is:

    :math:`C_{D,wave,lift} = C_L^2 \\cdot \\frac{\\beta^2}{\\pi} \\cdot p \\cdot \\frac{s}{l} \\cdot K_w`

    where:
        - :math:`\\beta = \\sqrt{M^2-1}` is the Prandtl-Glauert factor
        - :math:`p = \\frac{2}{AR} \\cdot \\frac{s}{l}` is the wing parameter
        - :math:`x = \\beta \\cdot \\frac{s}{l}` is the normalized span parameter
        - :math:`K_w` is the wave drag factor calculated from empirical correlation
    
    **Definitions**

    'Supersonic Wave Drag'
        Wave drag occurring at supersonic speeds (M > 1.0).
    
    'JAXA Methodology'
        Empirical method for calculating supersonic wave drag developed by JAXA.
    
    'Prandtl-Glauert Factor'
        Compressibility correction factor for supersonic flow.

    References
    ----------
    [1] Yoshida, K. "Supersonic drag reduction technology in the scaled supersonic experimental airplane project by JAXA."
    """
    # Initalize cd arrays 
    Mach         = conditions.freestream.mach_number
    cd_lift_wave = np.zeros_like(Mach)
    
    for wing in  geometry.wings:
        if isinstance(wing, Main_Wing):   
            # Lift coefficient  
            CL = conditions.aerodynamics.coefficients.lift.total 
            l  = wing.chords.root  
        
            # JAXA method
            s    = wing.spans.projected / 2
            AR   = wing.aspect_ratio
            p    = 2/AR*s/l
            beta = np.sqrt(Mach**2-1) 
            x    =  beta*s/l
        
            ret = np.zeros_like(x) 
            ret[x > 0.178] = 0.4935 - 0.2382*x[x > 0.178] + 1.6306*x[x > 0.178]**2 - 0.86*x[x > 0.178]**3 + 0.2232*x[x > 0.178]**4 - 0.0365*x[x > 0.178]**5 - 0.5            
            
            Kw           = (1+1/p)*ret/(2*beta**2*(s/l)**2) 
            cd_lift_wave = CL**2 * (beta**2/np.pi*p*(s/l)*Kw) 
    
    cd_lift_wave[cd_lift_wave<0] = 0.
    cd_lift_wave[np.isnan(cd_lift_wave)] = 0.  
    return cd_lift_wave

def supersonic_volume_wave_drag(conditions, settings, vehicle):
    """
    Computes supersonic volume wave drag coefficient based on aircraft geometry.

    Parameters
    ----------
    conditions : Data
        Flight conditions (not used in calculation)
    settings : dict
        Analysis settings (not used in calculation)
    vehicle : Data
        Aircraft geometry containing:
            - reference_area : float
                Reference area for drag coefficient [m²]
            - length : float
                Total aircraft length [m]
            - maximum_cross_sectional_area : float
                Maximum cross-sectional area [m²]
            - fuselages : list
                List of fuselage objects containing:
                    - width : float
                        Width of the fuselage [m]

    Returns
    -------
    CD_wave_vol : float
        Supersonic volume wave drag coefficient [unitless]

    Notes
    -----
    This function calculates the supersonic volume wave drag based on aircraft
    geometry using empirical correlations from Wright Laboratory procedures.
    
    **Major Assumptions**
        * Volume wave drag is independent of Mach number
        * Aircraft volume can be approximated by cylindrical geometry
        * Maximum fuselage width represents the characteristic dimension
        * Empirical correlation is valid for typical supersonic aircraft
    
    **Theory**

    The volume wave drag coefficient is:

    :math:`C_{D,wave,volume} = C_{D,wave,volume,frontal} \\cdot \\frac{A_{max}}{S_{ref}}`

    where the frontal area wave drag is:
    :math:`C_{D,wave,volume,frontal} = 24 \\cdot \\frac{V}{L^3}`

    The aircraft volume is approximated as:
    :math:`V = \\frac{3}{16} \\pi^2 R_{max}^2 L`

    where:
        - :math:`R_{max}` is the maximum fuselage radius
        - :math:`L` is the total aircraft length
        - :math:`A_{max}` is the maximum cross-sectional area
        - :math:`S_{ref}` is the reference area
    
    **Definitions**

    'Volume Wave Drag'
        Wave drag caused by aircraft volume in supersonic flow.
    
    'Frontal Area Wave Drag'
        Volume wave drag normalized by frontal area.
    
    'Characteristic Length'
        Representative length scale for volume wave drag calculation.

    References
    ----------
    [1] Sieron, T. R., et al. (1993). "Procedures and design data for the formulation of aircraft configurations." WRIGHT LAB WRIGHT-PATTERSON AFB OH.
    """
   
    S_ref = vehicle.reference_area
    L     = vehicle.length
    Amax  = vehicle.maximum_cross_sectional_area
     
    # compute volume drag referenced to frontal area
    Rmax = 0
    for fuselage in vehicle.fuselages:
        Rmax =  np.maximum(Rmax, fuselage.width)
    volume      =  (3 / 16) * (np.pi ** 2) * (Rmax ** 2)  *  L 
    CD_wave_vol_frontal = 24 *volume /(L **3)
    
    CD_wave_vol =  CD_wave_vol_frontal * ( Amax / S_ref) *  np.ones_like(conditions.freestream.mach_number) 
    return CD_wave_vol
 


