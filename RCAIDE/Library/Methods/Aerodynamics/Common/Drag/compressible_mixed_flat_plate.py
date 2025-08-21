

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# package imports
import numpy as np


# ----------------------------------------------------------------------
#  Compressible Mixed Flat Plate
# ----------------------------------------------------------------------
def compressible_mixed_flat_plate(Re,Ma,Tc,xt):
    """
    Computes the compressible skin friction coefficient for a flat plate with mixed laminar-turbulent boundary layer.

    Parameters
    ----------
    Re : float
        Reynolds number based on chord length [unitless]
    Ma : float
        Freestream Mach number [unitless]
    Tc : float
        Freestream static temperature [K]
    xt : float
        Turbulent transition point as a proportion of chord length [unitless]
            - 0.0 indicates fully turbulent flow
            - 1.0 indicates fully laminar flow
            - Values between 0.0 and 1.0 indicate mixed flow

    Returns
    -------
    cf_comp : float
        Compressible skin friction coefficient [unitless]
    k_comp : float
        Compressibility correction factor [unitless]
    k_reyn : float
        Reynolds number correction factor [unitless]

    Notes
    -----
    This function calculates the skin friction coefficient for a flat plate accounting for
    compressibility effects, mixed laminar-turbulent boundary layers, and surface roughness
    typical of transport aircraft. The method combines laminar and turbulent correlations
    with compressibility corrections based on temperature effects.
    
    **Major Assumptions**
        * Reynolds number between 10^5 and 10^9
        * Turbulent transition point between 0 and 1
        * Surface roughness typical of transport aircraft
        * Adiabatic wall conditions
        * Perfect gas behavior
    
    **Theory**

    The effective transition length is calculated as:

    :math:`\\theta = \\frac{0.671 x_t}{\\sqrt{Re_{x_t}}}`

    :math:`x_{eff} = (27.78 \\theta Re^{0.2})^{1.25}`

    where :math:`Re_{x_t} = Re \\cdot x_t` is the Reynolds number at transition.

    The effective Reynolds number for turbulent flow is:

    :math:`Re_{eff} = Re(1 - x_t + x_{eff})`

    The laminar skin friction coefficient is:

    :math:`C_{f,lam} = \\frac{1.328}{\\sqrt{Re_{x_t}}}`

    The turbulent skin friction coefficient is interpolated from empirical data
    accounting for typical transport aircraft surface roughness.

    The mixed flow skin friction coefficient is:

    :math:`C_{f,mixed} = C_{f,lam} x_t + C_{f,turb}(1 - x_t + x_{eff}) - C_{f,start} x_{eff}`

    The compressibility correction accounts for temperature effects:

    :math:`T_w = T_c(1 + 0.178 M^2)`

    :math:`T_d = T_c(1 + 0.035 M^2 + 0.45(\\frac{T_w}{T_c} - 1))`

    :math:`k_{comp} = \\frac{T_c}{T_d}`

    The Reynolds number correction is:

    :math:`Re_d = Re(\\frac{T_d}{T_c})^{1.5}(\\frac{T_d + 216}{T_c + 216})`

    :math:`k_{reyn} = (\\frac{Re}{Re_d})^{0.2}`

    The final compressible skin friction coefficient is:

    :math:`C_{f,comp} = C_{f,mixed} \\cdot k_{comp} \\cdot k_{reyn}`
    
    **Definitions**

    'Mixed Boundary Layer'
        Boundary layer that transitions from laminar to turbulent flow at some point along the surface.
    
    'Skin Friction Coefficient'
        Dimensionless measure of the shear stress at the wall normalized by dynamic pressure.
    
    'Compressibility Correction'
        Factor accounting for the effects of compressibility on boundary layer properties.

    References
    ----------
    [1] Stanford AA241 Course Notes. adg.stanford.edu
    [2] Shevell, R. S. (1989). "Fundamentals of Flight." Prentice Hall.

    """
    
    if xt < 0.0 or xt > 1.0:
        raise ValueError("Turbulent transition must be between 0 and 1")

    Rex = Re*xt
    Rex[Rex==0.0] = 0.0001

    theta = 0.671*xt/(Rex**0.5)
    xeff  = (27.78*theta*Re**0.2)**1.25
    Rext  = Re*(1-xt+xeff)
    
    #cf_turb  = 0.455/(np.log10(Rext)**2.58) # Schlichting empircal formula for turbulent flat plate. note that this correlation is meant for smooth surfaces.

    # Turbulent flat plate correlation from Shevell's Fundamentals of Flight. Accounts for typical roughness of a transport aircraft
    y = np.array([0.01273, 0.0090077, 0.0058721317, 0.00716019, 0.0049416, 0.003744045, 0.00316321, 0.002506763, 0.002064, 0.0016604122])
    x = np.array([10000, 115300, 675100, 281300, 1492600, 6040200, 14920400, 58584300, 193593000, 897594100])
    cf_turb = np.interp(Rext, x, y) # Data is from Figure 11.2 of Shevell's Fundamentals of Flight.
    cf_lam   = 1.328/(Rex**0.5)
    
    if xt > 0.0:
        cf_start = np.interp(Re*xeff, x, y)  
    else:
        cf_start = 0.0
    
    cf_inc = cf_lam*xt + cf_turb*(1-xt+xeff) - cf_start*xeff
    
    # compressibility correction
    Tw = Tc * (1. + 0.178*Ma*Ma)
    Td = Tc * (1. + 0.035*Ma*Ma + 0.45*(Tw/Tc - 1.))
    k_comp = (Tc/Td) 
    
    # reynolds correction
    Rd_w   = Re * (Td/Tc)**1.5 * ( (Td+216.) / (Tc+216.) )
    k_reyn = (Re/Rd_w)**0.2
    
    # apply corrections
    cf_comp = cf_inc * k_comp * k_reyn
    
    return cf_comp, k_comp, k_reyn