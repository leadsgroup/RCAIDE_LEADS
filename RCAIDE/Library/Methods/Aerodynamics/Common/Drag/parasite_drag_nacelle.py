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
        * Raymer's form factor correlation is valid for nacelle geometry
        * Compressible turbulent flat plate skin friction correlation
        * Cubic spline blending smooths transition between subsonic and supersonic regimes
        * Nacelle shape can be approximated as a cylindrical body
    
    **Theory**

    The nacelle Reynolds number is:

    :math:`Re_{nac} = Re \\cdot l_{nac}`

    where :math:`Re` is the freestream Reynolds number per unit length and :math:`l_{nac}` is the nacelle length.

    The skin friction coefficient is calculated using compressible turbulent flat plate theory:

    :math:`C_f = f(Re_{nac}, M, T)`

    The reference area is:

    :math:`S_{ref} = \\pi \\cdot d_{nac} \\cdot l_{nac}`

    where :math:`d_{nac}` is the nacelle diameter.

    The form factor follows Raymer's correlation:

    :math:`FF = 1 + \\frac{0.35}{l_{nac}/d_{nac}}`

    For subsonic flow (M ≤ 1.0), the parasite drag coefficient is:

    :math:`C_{D,parasite} = FF \\cdot C_f \\cdot \\frac{S_{wet}}{S_{ref}}`

    For supersonic flow, the form factor is blended using a cubic spline:

    :math:`FF_{eff} = FF \\cdot h_{00}(M) + 1 \\cdot (1-h_{00}(M))`

    where :math:`h_{00}(M)` is the cubic spline blending function.

    The final parasite drag coefficient is:

    :math:`C_{D,parasite} = FF_{eff} \\cdot C_f \\cdot \\frac{S_{wet}}{S_{ref}}`
    
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
    RCAIDE.Library.Methods.Utilities.Cubic_Spline_Blender
    """

    # unpack inputs
    conditions       = state.conditions
    freestream       = conditions.freestream
    Mach             = freestream.mach_number
    T                = freestream.temperature     
    Re               = freestream.reynolds_number
    low_mach_cutoff  = settings.supersonic.begin_drag_rise_mach_number
    high_mach_cutoff = settings.supersonic.end_drag_rise_mach_number 
    Sref             = np.pi * nacelle.diameter * nacelle.length 
    Swet             = nacelle.areas.wetted
    
    # Reynolds number
    Re_prop = Re*nacelle.length
    
    # Skin friction coefficient
    cf_prop, k_comp, k_reyn = compressible_turbulent_flat_plate(Re_prop,Mach,T) 
    
    # Form factor according to Raymer equation
    form_factor  = 1 + 0.35 / ( nacelle.length/nacelle.diameter)   
         
    if np.all((Mach<=1.0) == True): 
        # subsonic condition 
        parasite_drag = form_factor * cf_prop * Swet / Sref 
    else:

        # supersonic condition 
        k_prop_sup = 1.
        
        trans_spline = Cubic_Spline_Blender(low_mach_cutoff,high_mach_cutoff)
        h00 = lambda M:trans_spline.compute(M)
        
        form_factor = form_factor*(h00(Mach)) + k_prop_sup*(1-h00(Mach))
             
        # find the final result    
        parasite_drag = form_factor * cf_prop * Swet / Sref        
    
    # store results
    results = Data(
        wetted_area               = Swet    , 
        reference_area            = Sref    , 
        total                     = parasite_drag ,
        skin_friction             = cf_prop ,
        compressibility_factor    = k_comp  ,
        reynolds_factor           = k_reyn  , 
        form_factor               = form_factor  ,
    )
    state.conditions.aerodynamics.coefficients.drag.parasite[nacelle.tag] = results    
    
    return