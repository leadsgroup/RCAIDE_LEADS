# RCAIDE/Library/Methods/Aerodynamics/Common/Drag/supersonic_wave_drag_volume_raymer.py
# (c) Copyright 2023 Aerospace Research Community LLC
# 
# Created:  Jun 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from RCAIDE.Framework.Core                    import    Units 
from RCAIDE.Library.Components.Wings          import Main_Wing
from RCAIDE.Library.Methods.Utilities         import Cubic_Spline_Blender   
from RCAIDE.Library.Methods.Geometry.Planform.convert_sweep import convert_sweep 

# package imports
import numpy as np

# ---------------------------------------------------------------------------------------------------------------------- 
#  Supersonic Wave Drag Due to Volume - Raymer Method 
# ---------------------------------------------------------------------------------------------------------------------- 
def supersonic_wave_drag_volume_raymer(vehicle,mach,scaling_factor):
    """Computes the volume drag

    Assumptions:
    Basic fit

    Source:
    D. Raymer, Aircraft Design: A Conceptual Approach, Fifth Ed. pg. 448-449

    Args:
    vehicle.
      wings.main_wing.sweeps.leading_edge [rad]
      total_length                        [m]
      maximum_cross_sectional_area        [m^2]
      reference_area                      [m^2]
      
    Returns:
    vehicle_wave_drag                     [Unitless] 
    """
    L =  0
    for wing in vehicle.wings:
        if isinstance(wing,Main_Wing):
            main_wing = wing
            L  =  wing.chords.root
    
    main_wing = vehicle.wings.main_wing
    # estimation of leading edge sweep if not defined 
    if main_wing.sweeps.leading_edge == None:                           
        main_wing.sweeps.leading_edge  = convert_sweep(main_wing,old_ref_chord_fraction = 0.25 ,new_ref_chord_fraction = 0.0) 
     
    for fuselage in vehicle.fuselages:
        L =  np.maximum(L, fuselage.lengths.total)
        
    LE_sweep = main_wing.sweeps.leading_edge / Units.deg
    Ae       = vehicle.maximum_cross_sectional_area
    S        = vehicle.reference_area
    
    # Compute sears-hack D/q
    Dq_SH = 9*np.pi/2*(Ae/L)*(Ae/L)
    
    spline = Cubic_Spline_Blender(1.2,1.3)
    h00    = lambda M:spline.compute(M)    
    
    # Compute full vehicle D/q
    Dq_vehicle           = np.zeros_like(mach)
    Dq_vehicle_simpified = np.zeros_like(mach)
    
    Dq_vehicle[mach>=1.2] = scaling_factor*(1-0.2*(mach[mach>=1.2]-1.2)**0.57*(1-np.pi*LE_sweep**.77/100))*Dq_SH
    Dq_vehicle_simpified  = scaling_factor*Dq_SH
    
    Dq_vehicle = Dq_vehicle_simpified*h00(mach) + Dq_vehicle*(1-h00(mach))
    
    CD_c_vehicle = Dq_vehicle/S
    
    return CD_c_vehicle