# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/parasite_drag_nacelle.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
  
from RCAIDE.Framework.Core                    import Data  
from RCAIDE.Library.Methods.Utilities         import Cubic_Spline_Blender   
from RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressible_turbulent_flat_plate import compressible_turbulent_flat_plate

# package imports
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Supersonic Parasite Drag Nacekke 
# ---------------------------------------------------------------------------------------------------------------------- 
def parasite_drag_nacelle(state,settings,geometry):
    """
    Computes the parasite drag coefficient for all nacelles in the aircraft.

    Parameters
    ----------
    state : Data
        Flight conditions and aerodynamic state
    settings : dict
        Aerodynamic analysis settings and parameters
    geometry : Data
        Aircraft geometry containing:
            - networks : list
                List of propulsion networks containing propulsors
                    - propulsors : list
                        List of propulsor objects with nacelle attributes
                            - nacelle : Nacelle, optional
                                Nacelle object to be analyzed

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.parasite[nacelle.tag]

    Notes
    -----
    This function iterates through all propulsion networks and propulsors to identify
    nacelles and compute their parasite drag coefficients using the nacelle_drag helper
    function.
    
    **Major Assumptions**
        * All nacelles follow the same drag calculation methodology
        * Nacelle drag is independent of other aircraft components
        * Each nacelle has a unique tag for result storage
    """
     
    # Estimating nacelle drag 
    for network in  geometry.networks: 
        for propulsor in network.propulsors:   
            if propulsor.nacelle != None:
                nacelle_drag(state,settings,propulsor.nacelle)
    return     
# ---------------------------------------------------------------------------------------------------------------------- 
#  Nacelle Drag 
# ---------------------------------------------------------------------------------------------------------------------- 
def nacelle_drag(state,settings, nacelle):
    """
    Computes the parasite drag coefficient for a single nacelle accounting for compressibility effects.

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
            - supersonic.begin_drag_rise_mach_number : float
                Mach number at which drag rise begins [unitless]
            - supersonic.end_drag_rise_mach_number : float
                Mach number at which drag rise ends [unitless]
    nacelle : Data
        Nacelle geometry containing:
            - tag : str
                Unique identifier for the nacelle
            - diameter : float
                Diameter of the nacelle [m]
            - length : float
                Length of the nacelle [m]
            - areas.wetted : float
                Wetted area of the nacelle [m²]

    Returns
    -------
    None
        Results are stored in state.conditions.aerodynamics.coefficients.drag.parasite[nacelle.tag]

    Notes
    -----
    This function calculates the parasite drag coefficient for a nacelle using compressible
    turbulent flat plate theory with form factor corrections. The calculation accounts for
    compressibility effects and uses cubic spline blending for the transonic regime.
    
    **Major Assumptions**
        * Fully turbulent boundary layer over the entire nacelle
        * Nacelle is treated as a cylindrical body of revolution, using the same
          Mach-dependent max-velocity-increment form factor method as the fuselage
          (see parasite_drag_fuselage) -- Raymer's nacelle correlation (1 + 0.35/(l/d))
          has no Mach dependence and left nacelle parasite drag flat across a subsonic
          speed sweep, unlike the wing and fuselage terms
        * Compressible turbulent flat plate skin friction correlation
        * Cubic spline blending smooths transition between subsonic and supersonic regimes
    
    **Theory**

    The nacelle Reynolds number is:

    :math:`Re_{nac} = Re \cdot l_{nac}`

    where :math:`Re` is the freestream Reynolds number per unit length and :math:`l_{nac}` is the nacelle length.

    The skin friction coefficient is calculated using compressible turbulent flat plate theory:

    :math:`C_f = f(Re_{nac}, M, T)`

    The reference area is:

    :math:`S_{ref} = \pi \cdot d_{nac} \cdot l_{nac}`

    where :math:`d_{nac}` is the nacelle diameter.

    The form factor follows the same cylindrical-body, Mach-dependent max-velocity-increment
    method used for the fuselage, with :math:`d/l` computed from the nacelle's own diameter
    and length:

    :math:`k_{nac} = (1 + FF \cdot \frac{\Delta u_{max}}{u_{\infty}})^2`

    The parasite drag coefficient is:

    :math:`C_{D,parasite} = k_{nac} \cdot C_f \cdot \frac{S_{wet}}{S_{ref}}`
    
    **Definitions**

    'Nacelle Drag'
        Parasite drag component caused by the nacelle's aerodynamic shape and surface friction.
    
    'Form Factor'
        Multiplier accounting for the increase in drag due to nacelle shape compared to a flat plate.

    References
    ----------
    [1] Stanford AA241 Course Notes

    See Also
    --------
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.compressible_turbulent_flat_plate
    RCAIDE.Library.Methods.Aerodynamics.Common.Drag.parasite_drag_fuselage
    RCAIDE.Library.Methods.Utilities.Cubic_Spline_Blender
    """

    # unpack inputs
    conditions       = state.conditions
    freestream       = conditions.freestream
    Mach             = freestream.mach_number
    T                = freestream.temperature     
    Re               = freestream.reynolds_number
    form_factor      = settings.nacelle_parasite_drag_form_factor
    low_mach_cutoff  = settings.supersonic.begin_drag_rise_mach_number
    high_mach_cutoff = settings.supersonic.end_drag_rise_mach_number 
    Sref             = np.pi * nacelle.diameter * nacelle.length 
    Swet             = nacelle.areas.wetted
    
    # Reynolds number
    Re_prop = Re*nacelle.length
    
    # Skin friction coefficient
    cf_prop, k_comp, k_reyn = compressible_turbulent_flat_plate(Re_prop,Mach,T) 
    
    # cylindrical-body form factor (same method as parasite_drag_fuselage, sized to the nacelle)
    d_d = nacelle.diameter/nacelle.length

    if np.all((Mach<=1.0) == True):
        # subsonic condition
        D              = np.zeros_like(Mach)
        D[Mach < 0.95]  = np.sqrt(1 - (1-Mach[Mach < 0.95]**2) * d_d**2)
        D[Mach >= 0.95] = np.sqrt(1 - d_d**2)

        a              = np.zeros_like(Mach)
        a[Mach < 0.95]  = 2 * (1-Mach[Mach < 0.95]**2) * (d_d**2) *(np.arctanh(D[Mach < 0.95])-D[Mach < 0.95]) / (D[Mach < 0.95]**3)
        a[Mach >= 0.95] = 2  * (d_d**2) *(np.arctanh(D[Mach >= 0.95])-D[Mach >= 0.95]) / (D[Mach >= 0.95]**3)

        du_max_u               = np.zeros_like(Mach)
        du_max_u[Mach < 0.95]  = a[Mach < 0.95] / ( (2-a[Mach < 0.95]) * (1-Mach[Mach < 0.95]**2)**0.5 )
        du_max_u[Mach >= 0.95] = a[Mach >= 0.95] / ( (2-a[Mach >= 0.95]) )

        k_nac         = (1 + form_factor*du_max_u)**2
        parasite_drag = k_nac * cf_prop * Swet / Sref
    else:

        # supersonic condition
        D_low        = np.zeros_like(Mach)
        a_low        = np.zeros_like(Mach)
        du_max_u_low = np.zeros_like(Mach)

        # "low" (subsonic) formula relies on arctanh(D), which is only defined for D<1,
        # i.e. Mach<1 -- it must never be evaluated past that regardless of high_mach_cutoff.
        # It decays continuously to 0 as Mach->1, so leaving it at 0 beyond that is exact, not an approximation.
        low_inds  = Mach < 1.0

        D_low[low_inds]        = np.sqrt(1 - (1-Mach[low_inds]**2) * d_d**2)
        a_low[low_inds]        = 2 * (1-Mach[low_inds]**2) * (d_d**2) *(np.arctanh(D_low[low_inds])-D_low[low_inds]) / (D_low[low_inds]**3)
        du_max_u_low[low_inds] = a_low[low_inds] / ( (2-a_low[low_inds]) * (1-Mach[low_inds]**2)**0.5 )

        # "high" (frozen) formula has no Mach dependence, so it is valid everywhere -- no masking needed
        D_high        = np.sqrt(1 - d_d**2) * np.ones_like(Mach)
        a_high        = 2  * (d_d**2) *(np.arctanh(D_high)-D_high) / (D_high**3)
        du_max_u_high = a_high / (2-a_high)

        trans_spline = Cubic_Spline_Blender(low_mach_cutoff,high_mach_cutoff)
        h00 = lambda M:trans_spline.compute(M)

        du_max_u = du_max_u_low*(h00(Mach)) + du_max_u_high*(1-h00(Mach))

        k_nac = (1 + form_factor*du_max_u)**2

        # find the final result    
        parasite_drag = k_nac * cf_prop * Swet / Sref        
    
    # store results
    results = Data(
        wetted_area               = Swet    , 
        reference_area            = Sref    , 
        total                     = parasite_drag ,
        skin_friction             = cf_prop ,
        compressibility_factor    = k_comp  ,
        reynolds_factor           = k_reyn  , 
        form_factor               = k_nac  ,
    )
    state.conditions.aerodynamics.coefficients.drag.parasite[nacelle.tag] = results    
    
    return