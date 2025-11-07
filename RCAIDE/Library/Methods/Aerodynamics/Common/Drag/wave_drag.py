# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/compressibility_drag_total.py
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core                    import Data
from RCAIDE.Library.Components.Wings          import Main_Wing
from RCAIDE.Library.Methods.Utilities         import Cubic_Spline_Blender

# package imports
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Compressibility Drag Total
# ----------------------------------------------------------------------------------------------------------------------  
def wave_drag(state,settings,geometry):
    """
    Computes wave drag coefficient for full aircraft including volume and lift components.

    Parameters
    ----------
    state : Data
        Flight conditions and aerodynamic state containing:
            - conditions.freestream.mach_number : float
                Freestream Mach number [unitless]
            - conditions.aerodynamics.coefficients.lift.total : float
                Total lift coefficient [unitless]
            - conditions.aerodynamics.coefficients.lift.inviscid.spanwise : float, optional
                Spanwise lift distribution [unitless]
    settings : dict
        Aerodynamic analysis settings containing:
            - supersonic.end_drag_rise_mach_number : float
                Mach number at which supersonic drag rise ends [unitless] 
            - use_surrogate : bool
                Flag to use surrogate model for transonic wave drag
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
                Reference area for drag coefficient calculation [m²]
            - length : float
                Total aircraft length [m]
            - maximum_cross_sectional_area : float
                Maximum cross-sectional area [m²]
            - fuselages : list
                List of fuselage objects containing:
                    - width : float
                        Width of the fuselage [m]
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
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.wave

    Notes
    -----
    This function calculates the wave drag coefficient including both volume and lift
    components. The calculation uses different methods for transonic and supersonic
    regimes, with cubic spline blending for smooth transitions.
    
    **Major Assumptions**
        * Wave drag is composed of volume and lift components
        * Cubic spline blending provides smooth transitions between regimes
        * Transonic wave drag is limited to Mach 0.7-0.95 range
        * Supersonic wave drag begins at Mach 1.0
        * Volume wave drag is independent of Mach number
    
    **Theory**

    The total wave drag coefficient is the sum of volume and lift components:

    :math:`C_{D,wave} = C_{D,wave,volume} + C_{D,wave,lift}`

    The volume wave drag is:

    :math:`C_{D,wave,volume} = C_{D,wave,volume,frontal} \\cdot (1 - h_{00}(M))`

    The lift wave drag combines transonic and supersonic contributions:

    :math:`C_{D,wave,lift} = C_{D,wave,transonic} \\cdot h_{00}(M) + C_{D,wave,supersonic} \\cdot (1 - h_{002}(M))`

    where :math:`h_{00}(M)` and :math:`h_{002}(M)` are cubic spline blending functions.
    
    **Definitions**

    'Wave Drag'
        Drag component caused by shock waves in compressible flow.
    
    'Volume Wave Drag'
        Wave drag due to aircraft volume and cross-sectional area.
    
    'Lift Wave Drag'
        Wave drag due to lift generation in compressible flow.

    References
    ----------
    [1] Lock, R. C. (1986). "The Prediction of the Drag of Aerofoils and Wings at High Subsonic Speeds." Aeronautical Journal.
    [2] Harris, C. D. (1990). "NASA Supercritical Airfoils." NASA TP 2969.
    [3] Yoshida, K. "Supersonic drag reduction technology in the scaled supersonic experimental airplane project by JAXA."
    [4] Sieron, T. R., et al. (1993). "Procedures and design data for the formulation of aircraft configurations." WRIGHT LAB WRIGHT-PATTERSON AFB OH.

    See Also
    --------
    transonic_lift_wave_drag
    supersonic_lift_wave_drag
    supersonic_volume_wave_drag
    RCAIDE.Library.Methods.Utilities.Cubic_Spline_Blender
    """
     
    # Unpack
    conditions       = state.conditions
    Mach             = conditions.freestream.mach_number  
    high_mach_cutoff = settings.supersonic.end_drag_rise_mach_number 
    low_mach_cutoff  = settings.supersonic.begin_drag_rise_mach_number 

    # supersonic smoothing 
    sup_spline = Cubic_Spline_Blender(low_mach_cutoff,high_mach_cutoff) 
    sup_h00    = lambda M:sup_spline.compute(M) 
    
    # Wave drag due to volume  
    CD_wave_volume = supersonic_volume_wave_drag(conditions, settings, geometry) *(1-sup_h00(Mach))
    
    # wave drag due to lift  
    transonic_CDw_lift           = transonic_lift_wave_drag(conditions, settings, geometry) *sup_h00(Mach)
    transonic_CDw_lift[Mach<0.7] = 0
    transonic_CDw_lift[Mach>high_mach_cutoff] = 0
    supersonic_CDw_lift          = supersonic_lift_wave_drag(conditions, settings, geometry) *(1-sup_h00(Mach))
    supersonic_CDw_lift[Mach<1]  = 0   
   
    # total wave drag  
    CD_wave_lift   = supersonic_CDw_lift + transonic_CDw_lift
    CD_wave        = CD_wave_lift + CD_wave_volume 

    # Save drag breakdown 
    conditions.aerodynamics.coefficients.drag.wave = Data(total  = CD_wave, 
                                                          lift   = CD_wave_lift,
                                                          volume = CD_wave_volume,)  
        
    return

# ---------------------------------------------------------------------------------------------------------------------- 
#  transonic_lift_wave_drag
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
            - aerodynamics.coefficients.lift.inviscid.spanwise : float, optional
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
    
    if settings.use_surrogate or (settings.vortex_distribution == None): 
        Cl                   = conditions.aerodynamics.coefficients.lift.total
        CD_wave_transonic    = np.array([-1.34E-03,2.35E-05,1.42E-03,1.95E-03,2.23E-03,2.56E-03,2.80E-03, 4.32E-03,4.89E-03,
                                         7.11E-03,1.48E-02, 2.31E-02,2.83E-02,3.42E-02,3.96E-02,3.76E-02,2.71E-02]) 
        CLs                  = np.array([-0.25836715,-0.05233014,0.08334449,0.21608904,0.35012533, 0.42120447,0.48458659,
                                         0.55214519,0.62108179,0.69313687,0.75284432,0.81921256,0.95662739,1.07911642,1.19681914,1.33118394,1.47947542      ])    
        CD_wave_transonic    = np.interp(Cl, CLs, CD_wave_transonic) 
    else: 
        chords   = settings.vortex_distribution.chord_lengths
        delta    = settings.vortex_distribution.leading_edge_sweeps 
        dy       = settings.vortex_distribution.chord_widths
        
        CD_wave_total = np.zeros_like(Mach)
        CL_y          = conditions.aerodynamics.coefficients.lift.inviscid.spanwise 
        c_kappa       = 0.23 # normalized curvature of the airfoil. This can eventually be calcualted using airfoil shape data. 
    
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

        S_segment = chords[0]*dy[0]
        CD_wave_total = np.sum(CD_wave_segment*S_segment/S_ref, axis=1, keepdims=True)

        CD_wave_transonic = CD_wave_total
        
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
    # initialize cd arrays 
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
 
















 
