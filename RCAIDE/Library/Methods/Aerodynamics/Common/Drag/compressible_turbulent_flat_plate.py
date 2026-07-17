

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------
#  Compressible Turbulent Flat Plate
# ----------------------------------------------------------------------
def compressible_turbulent_flat_plate(Re,Ma,Tc):
    """
    Computes the compressible skin friction coefficient for a fully turbulent flat plate.

    Parameters
    ----------
    Re : float
        Reynolds number based on chord length [unitless]
    Ma : float
        Freestream Mach number [unitless]
    Tc : float
        Freestream static temperature [K]

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
    This function calculates the skin friction coefficient for a flat plate with fully
    turbulent boundary layer, accounting for compressibility effects through temperature
    corrections. 
    
    **Major Assumptions**
        * Reynolds number between 10^5 and 10^9
        * Fully turbulent boundary layer from leading edge
        * Smooth surface conditions
        * Adiabatic wall conditions
        * Perfect gas behavior
    
    **Theory**

    The incompressible turbulent skin friction coefficient follows Schlichting's correlation:

    :math:`C_{f,inc} = \\frac{0.455}{(\\log_{10}(Re))^{2.58}}`

    The compressibility correction accounts for temperature effects:

    :math:`T_w = T_c(1 + 0.178 M^2)`

    :math:`T_d = T_c(1 + 0.035 M^2 + 0.45(\\frac{T_w}{T_c} - 1))`

    :math:`k_{comp} = \\frac{T_c}{T_d}`

    where:
    - :math:`T_w` is the adiabatic wall temperature [K]
    - :math:`T_d` is the reference temperature for viscosity [K]
    - :math:`T_c` is the freestream static temperature [K]
    - :math:`M` is the freestream Mach number

    The Reynolds number correction accounts for temperature-dependent viscosity:

    :math:`Re_d = Re(\\frac{T_d}{T_c})^{1.5}(\\frac{T_d + 216}{T_c + 216})`

    :math:`k_{reyn} = (\\frac{Re}{Re_d})^{0.2}`

    The final compressible skin friction coefficient is:

    :math:`C_{f,comp} = C_{f,inc} \\cdot k_{comp} \\cdot k_{reyn}`
    
    **Definitions**

    'Turbulent Boundary Layer'
        Boundary layer characterized by chaotic, irregular fluid motion and high mixing rates.
    
    'Skin Friction Coefficient'
        Dimensionless measure of the shear stress at the wall normalized by dynamic pressure.
    
    'Compressibility Correction'
        Factor accounting for the effects of compressibility on boundary layer temperature and viscosity.

    References
    ----------
    [1] Stanford AA241 Course Notes. adg.stanford.edu

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressible_mixed_flat_plate
    """

    # incompressible skin friction coefficient
    cf_inc = 0.455/(np.log10(Re))**2.58
    
    # compressibility correction
    Tw = Tc * (1. + 0.178*Ma**2.)
    Td = Tc * (1. + 0.035*Ma**2. + 0.45*(Tw/Tc - 1.))
    k_comp = (Tc/Td) 
    
    # reynolds correction
    Rd_w   = Re * (Td/Tc)**1.5 * ( (Td+216.) / (Tc+216.) )
    k_reyn = (Re/Rd_w)**0.2
    
    # apply corrections
    cf_comp = cf_inc * k_comp * k_reyn
    
    return cf_comp, k_comp, k_reyn
