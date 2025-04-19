# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/drag_divergence.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
   
from RCAIDE.Library.Components.Wings          import Main_Wing 

# package imports
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
# drag_divergence
# ----------------------------------------------------------------------------------------------------------------------
def drag_divergence(Mach,geometry, Cl):
    """Use drag divergence Mach number to determine drag for subsonic speeds

    Assumptions:
    Basic fit, subsonic

    Source:
    http://aerodesign.stanford.edu/aircraftdesign/aircraftdesign.html (Stanford AA241 A/B Course Notes)
    Concorde data can be found in "Supersonic drag reduction technology in the scaled supersonic 
    experimental airplane project by JAXA" by Kenji Yoshida

    Args:
    wing.
      thickness_to_chord    [-]     
      sweeps.quarter_chord  [radians]
      high_mach             [Boolean]
      areas.reference       [m^2]

    Returns:
    cd_c                    [-]
    Machc                   [-]
    MDiv                    [-]
  
    """

    t_c_w     = 0
    sweep_w   = 0
    high_mach = False 
    for wing in  geometry.wings:
        if isinstance(wing, Main_Wing):  
            t_c_w     = wing.thickness_to_chord
            sweep_w   = wing.sweeps.quarter_chord
            high_mach =  wing.high_mach
        
    # Check if the wing is designed for high subsonic cruise
    # If so use arbitrary divergence point as correlation will not work
    if high_mach == True: 
        # Divergence Mach number, fit to Concorde data 
        Mcc    =  0.95 * np.ones_like(Mach)  
    else:  
        # Get effective CLift_wings and sweep 
        tc = t_c_w / np.cos(sweep_w)
        cl = Cl/ (np.cos(sweep_w) ** 2)

        # Compressibility drag based on regressed fits from AA241
    
        mcc_cos_ws = 0.922321524499352       \
                   - 1.153885166170620*tc    \
                   - 0.304541067183461*cl    \
                   + 0.332881324404729*tc*tc \
                   + 0.467317361111105*tc*cl \
                   + 0.087490431201549*cl*cl
         
        # Crest-critical Mach number, corrected for wing sweep
        Mcc = mcc_cos_ws/ np.cos(sweep_w)      

    # Divergence ratio
    mo_Mach = Mach/Mcc

    # Compressibility correlation, Shevell
    dcdc_cos3g = 0.0019*mo_Mach**14.641

    # Compressibility drag 
    # Sweep correlation cannot be used if the wing has a high Mach design
    if high_mach is True:
        cd_c = dcdc_cos3g
    else:
        cd_c = dcdc_cos3g * (np.cos(sweep_w)**3)
          
    return cd_c 
