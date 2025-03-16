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
    """Computes the parasite drag due to the nacelle

    Assumptions:
        None 

    Source:
        None 

    Args: 
        state.conditions.freestream.mach_number      (numpy.ndarray): mach_number      [Unitless]
        state.conditions.freestream.temperature      (numpy.ndarray): temperature      [K]
        state.conditions.freestream.reynolds_number  (numpy.ndarray): Reynolds number  [Unitless]
        settings                                              (dict): analyses settings [-]
        geometry                                              (dict): aircraft geometry [-]

    Returns:
        None 
    """
     
    # Estimating nacelle drag 
    for network in  geometry.networks: 
        for propulsor in network.propulsors:  
            if 'nacelle' in propulsor: 
                nacelle_drag(state,settings,propulsor.nacelle)
    return     
# ---------------------------------------------------------------------------------------------------------------------- 
#  Nacelle Drag 
# ---------------------------------------------------------------------------------------------------------------------- 
def nacelle_drag(state,settings, nacelle):
    """helperr fuction to computes the parasite drag due to the nacelle

    Assumptions:
        None 

    Source:
        Stanford AA241 A/B Course Notes

    Args: 
        state.conditions.freestream.mach_number      (numpy.ndarray): mach_number      [Unitless]
        state.conditions.freestream.temperature      (numpy.ndarray): temperature      [K]
        state.conditions.freestream.reynolds_number  (numpy.ndarray): Reynolds number  [Unitless]
        settings                                              (dict): analyses settings [-] 

    Returns:
        None 
    """

    # unpack inputs
    conditions       = state.conditions
    freestream       = conditions.freestream
    Mach             = freestream.mach_number
    T                = freestream.temperature     
    Re               = freestream.reynolds_number
    low_mach_cutoff  = settings.supersonic.begin_drag_rise_mach_number
    high_mach_cutoff = settings.supersonic.end_drag_rise_mach_number 
    Sref             = nacelle.diameter**2 / 4 * np.pi
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