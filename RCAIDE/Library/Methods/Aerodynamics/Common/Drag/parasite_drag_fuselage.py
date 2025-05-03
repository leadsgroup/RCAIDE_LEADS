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
    """Computes the parasite drag due the a fuselage (or boom)

    Assumptions:
        None

    Source:
        Stanford AA241 A/B Course Notes

    Args:
        state.conditions.freestream.
          mach_number                                (numpy.ndarray): mach_number     [Unitless]
          temperature                                (numpy.ndarray): temperature     [K]
          reynolds_number                            (numpy.ndarray): Reynolds number [Unitless]
        settings.fuselage_parasite_drag_form_factor  (numpy.ndarray): form factor     [Unitless]
        geometry.fuselage.       
          areas.front_projected                              (float): projected front area [m^2]
          areas.wetted                                       (float): wetted area          [m^2]
          lengths.total                                      (float): length               [m]
          effective_diameter                                 (float): effective diamater   [m]

    Returns:
        None 
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
    
        du_max_u             = np.zeros_like(Mach)    
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
    results = Data(
        wetted_area               = Swet   , 
        reference_area            = Sref   , 
        total                     = fuselage_parasite_drag ,
        skin_friction             = cf_fus ,
        compressibility_factor    = k_comp ,
        reynolds_factor           = k_reyn , 
        form_factor               = k_fus  ,
    ) 
    state.conditions.aerodynamics.coefficients.drag.parasite[fuselage.tag] = results        
        
    return  