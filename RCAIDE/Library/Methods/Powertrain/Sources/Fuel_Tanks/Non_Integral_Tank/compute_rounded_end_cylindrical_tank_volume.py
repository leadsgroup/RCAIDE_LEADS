# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/Non_Integral_Tank/compute_rounded_end_cylindrical_tank_volume.py
# 
# 
# Created: Aug 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import  RCAIDE
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_non_dimensional_moment_of_inertia import compute_rounded_end_cylinder_non_dimensional_moi
from RCAIDE.Library.Methods.Geometry.Planform.convert_sweep import convert_sweep_segments  
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry,  compute_naca_4series 

#Python Imports 
import numpy as np
from scipy.interpolate import interp1d
from shapely.geometry import Polygon, Point
from copy import  deepcopy
import shapely
import os

# ----------------------------------------------------------------------------------------------------------------------
#  Methods to compute volume of non integrak tanks
# ----------------------------------------------------------------------------------------------------------------------  
def compute_rounded_end_cylindrical_tank_volume(fuel_tank):  
    """
    Computes the volume for a hollow rounded-end cylinder. 
    """ 
    
    # unpack (lengths.external is total tip-to-tip including hemispherical caps)
    L_total = fuel_tank.lengths.external
    D       = fuel_tank.diameters.external
    t       = fuel_tank.wall_thickness

    # outer dimensions: cylinder length = total - diameter (two hemispheres)
    R_o    = D / 2
    L_cyl  = L_total - D

    # inner dimensions
    R_i      = R_o - t
    L_cyl_i  = L_cyl - 2 * t

    # outer volume (cylinder + sphere)
    V_o_cyl = np.pi * R_o**2 * L_cyl
    V_o_sph = 4 / 3 * np.pi * R_o**3

    # inner volume (cylinder + sphere)
    V_i_cyl = np.pi * R_i**2 * L_cyl_i
    V_i_sph = 4 / 3 * np.pi * R_i**3

    fuel_tank.volume_properties.net_volume           = V_i_cyl + V_i_sph
    fuel_tank.volume_properties.gross_volume         = V_o_cyl + V_o_sph
    fuel_tank.fuel.mass_properties.center_of_gravity = [[L_total / 2, 0, 0]]
    fuel_tank.mass_properties.center_of_gravity      = [[L_total / 2, 0, 0]]
    fuel_tank.fuel.origin                            = fuel_tank.origin

    fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor = compute_rounded_end_cylinder_non_dimensional_moi(R_i, L_cyl_i)

    return
    