# RCAIDE/Methods/Aerodynamics/Common/Lift/BET_calculations.py
# 
# 
# Created:  Jul 2023, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
 
from RCAIDE.Framework.Core import interp2d 

# package imports 
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_airfoil_aerodynamics
# ----------------------------------------------------------------------------------------------------------------------   
def compute_airfoil_aerodynamics(beta,c,r,R,B,Wa,Wt,a,nu,airfoils,airfoil_locations,ctrl_pts,Nr,Na,tc,use_2d_analysis):
    """
    Computes aerodynamic forces at sectional blade locations using Blade Element Theory. 

    Parameters
    ----------
    beta : float
        Blade twist distribution [radians]
    c : float
        Chord distribution [m]
    r : float
        Radius distribution [m]
    R : float
        Tip radius [m]
    B : int
        Number of rotor blades [unitless]
    Wa : float
        Axial velocity [m/s]
    Wt : float
        Tangential velocity [m/s]
    a : float
        Speed of sound [m/s]
    nu : float
        Kinematic viscosity [m²/s]
    airfoils : list
        List of airfoil objects containing polar data
    airfoil_locations : list
        List indicating which airfoil is used at each radial station
    ctrl_pts : int
        Number of control points [unitless]
    Nr : int
        Number of radial blade sections [unitless]
    Na : int
        Number of azimuthal blade stations [unitless]
    tc : float
        Thickness-to-chord ratio [unitless]
    use_2d_analysis : bool
        Flag for 2D disc vs. 1D single angle analysis

    Returns
    -------
    Cl : float
        Lift coefficients [unitless]
    Cdval : float
        Drag coefficients before scaling [unitless]
    alpha : float
        Section local angle of attack [radians]
    alpha_disc : float
        Angle of attack distribution across disc [radians]
    Ma : float
        Local Mach number [unitless]
    W : float
        Local velocity magnitude [m/s]
    Re : float
        Local Reynolds number [unitless]
    Re_disc : float
        Reynolds number distribution across disc [unitless]

    Notes
    -----
    This function computes aerodynamic forces at blade sections using either
    airfoil polar data or empirical approximations. Please note that the empirical correlations
    are rather coarse and should be used with caution. The calculation accounts
    for local Reynolds number, Mach number, and angle of attack effects.
    
    **Major Assumptions**
        * Airfoil polars are available if airfoils are specified
        * Empirical correlations are valid for unspecified airfoils
        * Compressibility effects follow Karman-Tsien correction
        * Stall behavior follows standard airfoil characteristics
    
    **Theory**

    The local angle of attack accoutning for sideslip and rotation is:
    :math:`\\alpha = \\beta - \\arctan\\left(\\frac{W_a}{W_t}\\right)`

    The local velocity magnitude is:
    :math:`W = \\sqrt{W_a^2 + W_t^2}`

    The local Mach number is:
    :math:`M = \\frac{W}{a}`

    The local Reynolds number is:
    :math:`Re = \\frac{Wc}{\\nu}`

    For specified airfoils, lift and drag coefficients are interpolated from polar data.
    For unspecified airfoils, empirical correlations are used:
    :math:`C_{L,max} = -0.0009(t/c)^3 + 0.0217(t/c)^2 - 0.0442(t/c) + 0.7005`

    Reynolds number correction:
    :math:`C_{L,max,Re} = C_{L,max,ref} \\left(\\frac{Re}{Re_{ref}}\\right)^{0.1}`

    Karman-Tsien compressibility correction:
    :math:`C_L = \\frac{C_L}{(1-M^2)^{0.5} + \\frac{M^2}{1+(1-M^2)^{0.5}}} \\cdot \\frac{C_L}{2}`
    
    **Definitions**

    'Blade Element Theory'
        Method for analyzing rotor aerodynamics by dividing blades into discrete sections.
    
    'Airfoil Polar'
        Relationship between lift and drag coefficients as a function of angle of attack.
    
    'Karman-Tsien Correction'
        Compressibility correction for airfoil characteristics at high subsonic speeds.

    References
    ----------
    [1] Unknown
    """
    alpha    = beta - np.arctan2(Wa,Wt)
    W        = (Wa*Wa + Wt*Wt)**0.5
    Ma       = W/a
    Re       = (W*c)/nu

    # If rotor airfoils are defined, use airfoil surrogate
    if len(airfoil_locations) != 0:
        a_loc = np.array(airfoil_locations)
        # Compute blade Cl and Cd distribution from the airfoil data 
        if use_2d_analysis:
            # return the 2D Cl and CDval of shape (ctrl_pts, Nr, Na)
            Cl         = np.zeros((ctrl_pts,Nr,Na))
            Cdval      = np.zeros((ctrl_pts,Nr,Na))
            
            for jj,airfoil in enumerate(airfoils):
                pd                   = airfoil.polars
                Cl_af                = interp2d(Re,alpha,pd.reynolds_numbers, pd.angle_of_attacks, pd.lift_coefficients) 
                Cdval_af             = interp2d(Re,alpha,pd.reynolds_numbers, pd.angle_of_attacks, pd.drag_coefficients)
                locs                 = np.where(np.array(a_loc) == jj )
                Cl[:,locs,:]         = Cl_af[:,locs,:]
                Cdval[:,locs,:]      = Cdval_af[:,locs,:]
            alpha_disc           = alpha
            Re_disc              = Re
        else:
            # return the 1D Cl and CDval of shape (ctrl_pts, Nr)
            Cl         = np.zeros((ctrl_pts,Nr))
            Cdval      = np.zeros((ctrl_pts,Nr))             

            for jj,airfoil in enumerate(airfoils):
                pd                   = airfoil.polars
                Cl_af                = interp2d(Re,alpha,pd.reynolds_numbers, pd.angle_of_attacks, pd.lift_coefficients)
                Cdval_af             = interp2d(Re,alpha,pd.reynolds_numbers, pd.angle_of_attacks, pd.drag_coefficients)
                locs                 = np.where(np.array(a_loc) == jj )
                Cl[:,locs]           = Cl_af[:,locs]
                Cdval[:,locs]        = Cdval_af[:,locs] 
            alpha_disc = np.tile(alpha[:,:, None], (1, 1, Na)) 
            Re_disc    = np.tile(Re[:,:, None], (1, 1, Na))  

    else:
        # Estimate Cl max
        tc_1 = tc*100
        Cl_max_ref = -0.0009*tc_1**3 + 0.0217*tc_1**2 - 0.0442*tc_1 + 0.7005
        Cl_max_ref[Cl_max_ref<0.7] = 0.7
        Re_ref     = 9.*10**6
        Cl1maxp    = Cl_max_ref * ( Re / Re_ref ) **0.1

        # If not airfoil polar provided, use 2*pi as lift curve slope
        Cl = 2.*np.pi*alpha

        # By 90 deg, it's totally stalled.
        Cl[Cl>Cl1maxp]  = Cl1maxp[Cl>Cl1maxp] # This line of code is what changed the regression testing
        Cl[alpha>=np.pi/2] = 0.

        # Scale for Mach, this is Karmen_Tsien
        KT_cond = np.logical_and((Ma[:,:]<1.),(Cl>0))
        Cl[KT_cond] = Cl[KT_cond]/((1-Ma[KT_cond]*Ma[KT_cond])**0.5+((Ma[KT_cond]*Ma[KT_cond])/(1+(1-Ma[KT_cond]*Ma[KT_cond])**0.5))*Cl[KT_cond]/2)

        # If the blade segments are supersonic, don't scale
        Cl[Ma[:,:]>=1.] = Cl[Ma[:,:]>=1.]

        #This is an atrocious fit of DAE51 data at RE=50k for Cd
        Cdval = (0.108*(Cl*Cl*Cl*Cl)-0.2612*(Cl*Cl*Cl)+0.181*(Cl*Cl)-0.0139*Cl+0.0278)*((50000./Re)**0.2)
        Cdval[alpha>=np.pi/2] = 2.
        
        alpha_disc = np.tile(alpha[:,:, None], (1, 1, Nr)) 
        Re_disc    = np.tile(Re[:,:, None], (1, 1, Nr))  

    # prevent zero Cl to keep Cd/Cl from breaking in BET
    Cl[Cl==0] = 1e-6

    return Cl, Cdval, alpha, alpha_disc,Ma,W,Re,Re_disc

# ----------------------------------------------------------------------------------------------------------------------
#  compute_inflow_and_tip_loss
# ----------------------------------------------------------------------------------------------------------------------    
def compute_inflow_and_tip_loss(r,R,Wa,Wt,B,et1=1,et2=1,et3=1):
    """
    Computes the inflow ratio and tip loss factor for rotor analysis.

    Parameters
    ----------
    r : float
        Radius distribution [m]
    R : float
        Tip radius [m]
    Wa : float
        Axial velocity [m/s]
    Wt : float
        Tangential velocity [m/s]
    B : int
        Number of rotor blades [unitless]
    et1 : float, optional
        Tuning parameter for tip loss function [unitless]
    et2 : float, optional
        Tuning parameter for tip loss function [unitless]
    et3 : float, optional
        Tuning parameter for tip loss function [unitless]

    Returns
    -------
    lamdaw : float
        Inflow ratio [unitless]
    Ftip : float
        Tip loss factor [unitless]
    piece : float
        Intermediate calculation result needed for residual computation [unitless]

    Notes
    -----
    This function computes the inflow ratio and tip loss factor using
    empirical correlations. The tip loss factor accounts for the reduction
    in lift near the blade tips due to three-dimensional effects.
    
    **Major Assumptions**
        * Empirical tip loss correlation is valid for typical rotor configurations
        * Inflow ratio is small and positive
        * Tip loss follows exponential decay function
        * Tuning parameters allow for correlation adjustment
        * Blade tip effects are independent of blade number
    
    **Theory**

    The inflow ratio is:
    :math:`\\lambda_w = \\frac{r W_a}{R W_t}`

    where negative values are limited to a small positive number to prevent numerical issues.

    The tip loss factor follows an exponential decay:
    :math:`F_{tip} = \\frac{2}{\\pi} \\arccos(e^{-f_{tip}})`

    where the tip factor is:
    :math:`f_{tip} = \\frac{B}{2} \\frac{(R/r)^{\\eta_1} - 1)^{\\eta_2}}{\\lambda_w^{\\eta_3}}`

    and :math:`\\eta_1`, :math:`\\eta_2`, :math:`\\eta_3` are tuning parameters.
    
    **Definitions**

    'Inflow Ratio'
        Ratio of axial to tangential velocity at a given radial station.
    
    'Tip Loss Factor'
        Factor accounting for reduction in lift near blade tips due to 3D effects.
    
    'Blade Element Theory'
        Method for analyzing rotor aerodynamics by dividing blades into discrete sections.

    References
    ----------
    [1] Unknown
    """
    lamdaw             = r*Wa/(R*Wt)
    lamdaw[lamdaw<=0.] = 1e-12

    tipfactor = B/2.0*(  (R/r)**et1 - 1  )**et2/lamdaw**et3 

    piece = np.exp(-tipfactor)
    Ftip  = 2.*np.arccos(piece)/np.pi  

    return lamdaw, Ftip, piece