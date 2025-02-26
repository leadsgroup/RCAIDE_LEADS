# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/compressibility_drag_total.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jul 2024, RCAIDE Team 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
   
from RCAIDE.Library.Components.Wings          import Main_Wing
from RCAIDE.Library.Methods.Utilities         import Cubic_Spline_Blender  
from .wave_drag                               import wave_drag
from .drag_divergence                         import drag_divergence 
from .supersonic_wave_drag_volume_raymer      import supersonic_wave_drag_volume_raymer
from .supersonic_wave_drag_volume_sears_haack import supersonic_wave_drag_volume_sears_haack

# package imports
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Compressibility Drag Total
# ----------------------------------------------------------------------------------------------------------------------  
def compressibility_drag(state,settings,geometry):
    """Computes compressibility drag for full aircraft including volume drag

    Assumptions:
    None

    Source:
    None

    Args:   
    settings.
      begin_drag_rise_mach_number                                    [Unitless]
      end_drag_rise_mach_number                                      [Unitless]
      peak_mach_number                                               [Unitless]
      transonic_drag_multiplier                                      [Unitless]
      volume_wave_drag_scaling                                       [Unitless]
    state.conditions.aerodynamics.lift_breakdown.compressible_wings  [Unitless]
    state.conditions.freestream.mach_number                          [Unitless]
    geometry.maximum_cross_sectional_area                            [m^2] (used in subfunctions)
    geometry.total_length                                            [m]   (used in subfunctions)
    geometry.reference_area                                          [m^2]
    geometry.wings                             

    Returns:
    total_compressibility_drag                                       [Unitless]

    Properties Used:
    None
    """     

    # Unpack
    conditions  = state.conditions
    Mach        = conditions.freestream.mach_number 
    Cl          = conditions.aerodynamics.coefficients.lift.total  
      
    # supersonic 
    low_mach_cutoff  = settings.supersonic.begin_drag_rise_mach_number
    high_mach_cutoff = settings.supersonic.end_drag_rise_mach_number
    peak_mach        = settings.supersonic.peak_mach_number
    peak_factor      = settings.supersonic.transonic_drag_multiplier
    scaling_factor   = settings.supersonic.volume_wave_drag_scaling
    
    if settings.supersonic.wave_drag_type == 'Raymer':
        wave_drag_volume = supersonic_wave_drag_volume_raymer
    elif settings.supersonic.wave_drag_type == 'Sears-Haack':
        wave_drag_volume = supersonic_wave_drag_volume_sears_haack
    else:
        raise NotImplementedError    
    
    if settings.supersonic.cross_sectional_area_calculation_type != 'Fixed':
        raise NotImplementedError

    low_cutoff_volume_total  = np.zeros_like(Mach)
    high_cutoff_volume_total = np.zeros_like(Mach)     
    low_cutoff_volume_total  = drag_divergence(low_mach_cutoff*np.ones_like(Mach), geometry,Cl) 
    high_cutoff_volume_total = wave_drag_volume(geometry,low_mach_cutoff*np.ones_like(Mach),scaling_factor)
    
    peak_volume_total = high_cutoff_volume_total*peak_factor
     
    # subsonic side smoothing function
    a1 = (low_cutoff_volume_total-peak_volume_total)/(low_mach_cutoff-peak_mach)/(low_mach_cutoff-peak_mach)
    
    # supersonic side smoothing function
    a2 = (high_cutoff_volume_total-peak_volume_total)/(high_mach_cutoff-peak_mach)/(high_mach_cutoff-peak_mach) 
    
    # Shorten cubic Hermite spline
    sub_spline = Cubic_Spline_Blender(low_mach_cutoff, peak_mach-(peak_mach-low_mach_cutoff)*3/4)
    sup_spline = Cubic_Spline_Blender(peak_mach,high_mach_cutoff)
    sub_h00    = lambda M:sub_spline.compute(M)
    sup_h00    = lambda M:sup_spline.compute(M) 
    
    low_inds = Mach[:,0]<peak_mach
    hi_inds  = Mach[:,0]>=peak_mach

    cd_c_v_base                  = np.zeros_like(Mach) 
    cd_c_v_base[low_inds]        = drag_divergence(Mach[low_inds], geometry,Cl[low_inds]) 
    cd_c_l_base                  = lift_wave_drag(conditions, settings, geometry)
    cd_c_v_base[Mach>=peak_mach] = wave_drag_volume(geometry, Mach[Mach>=peak_mach], scaling_factor) 
    
    cd_c_v = np.zeros_like(Mach)
    cd_c_v[low_inds] = cd_c_v_base[low_inds]*(sub_h00(Mach[low_inds])) + transonic_drag_function(Mach[low_inds],a1[low_inds], peak_mach, peak_volume_total[low_inds])*(1-sub_h00(Mach[low_inds]))
    cd_c_v[hi_inds]  = transonic_drag_function(Mach[hi_inds],a2[hi_inds], peak_mach, peak_volume_total[hi_inds])*(sup_h00(Mach[hi_inds])) + cd_c_v_base[hi_inds]*(1-sup_h00(Mach[hi_inds]))

    if peak_mach<1.01:
        print('Warning: a peak Mach number of less than 1.01 will cause a small discontinuity in lift wave drag')
    cd_c_l = cd_c_l_base*(1-sup_h00(Mach))
    
    cd_c = cd_c_v + cd_c_l 
    
    # Save drag breakdown 
    drag = conditions.aerodynamics.coefficients.drag.compressible.total   = cd_c
        
    return  

# ---------------------------------------------------------------------------------------------------------------------- 
#  transonic_drag_function
# ---------------------------------------------------------------------------------------------------------------------- 
def transonic_drag_function(Mach,a_vertex, peak_mach, peak_volume_total):
    """  parabolic approximation of drag rise in the transonic region
    
    Assumptions:
    Basic fit

    Source:
    None 

    Args:
    Mach
    a_vertex
    peak_mach
    peak_volume_total

    Returns:
    transonic_drag
    """ 
    transonic_drag = a_vertex*(Mach-peak_mach)*(Mach-peak_mach)+peak_volume_total
    transonic_drag = transonic_drag.reshape(np.shape(Mach))
    return transonic_drag

# ---------------------------------------------------------------------------------------------------------------------- 
# lift_wave_drag
# ---------------------------------------------------------------------------------------------------------------------- 
def lift_wave_drag(conditions,configuration,geometry):
    """Determine lift wave drag for supersonic speeds

    Assumptions:
    Basic fit

    Source:
    http://aerodesign.stanford.edu/aircraftdesign/aircraftdesign.html (Stanford AA241 A/B Course Notes)

    Args:
    conditions.freestream.mach_number [-]
    configuration                     (passed to another function)
    wing.areas.reference              [m^2]
    Sref_main                         [m^2] Main reference area

    Returns:
    cd_c_l                            [-] Wave drag CD due to lift 
    """ 
    for wing in  geometry.wings:
        if isinstance(wing, Main_Wing):  
            # Unpack Mach number
            Mach       = conditions.freestream.mach_number
        
            # Initalize cd arrays
            cd_c_l = np.zeros_like(Mach) 
        
            # Calculate wing values at all Mach numbers
            cd_lift_wave = wave_drag(conditions,wing)
        
            # Pack supersonic results into correct elements
            cd_c_l[Mach >= 1.01] = cd_lift_wave[0:len(Mach[Mach >= 1.01]),0] 

    return cd_c_l

