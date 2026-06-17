# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/Non_Integral_Tank/compute_rounded_end_cylindrical_tank_volume.py
# 
# 
# Created: Aug 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import  RCAIDE 
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
    
    # unpack 
    L = fuel_tank.lengths.external
    D = fuel_tank.diameters.external 
    t = fuel_tank.wall_thickness
     
    # compute tank dimensions 
    L_o = L
    R_o = D / 2
    R_i = R_o -  t
    L_i = L_o - 2 * t # There are two different conventions in this script. One where L is from hemisphere tip to hemisphere tip the other where it is from cylinder end to cylinder end. 
         
    # volume of external tank
    V_i_cyl = (np.pi * R_i ** 2 * L_i )  
    V_i_sph = ( 4 / 3 * np.pi * R_i ** 3)   
    
    # volume of interal walls 
    V_o_cyl = (np.pi * R_o ** 2 * L_o )  
    V_o_sph = ( 4 / 3 * np.pi * R_o ** 3)   
   
    # store the center of gravity and origin of the fuel mass for use in mass properties calculations
    fuel_tank.volume_properties.net_volume            = V_i_cyl + V_i_sph
    fuel_tank.volume_properties.gross_volume          = V_o_cyl + V_o_sph
    fuel_tank.fuel.mass_properties.center_of_gravity  =  [[(L_o + D)/2, 0, 0]]
    fuel_tank.mass_properties.center_of_gravity       =  [[(L_o + D)/2, 0, 0]]
    fuel_tank.fuel.origin                             = fuel_tank.origin

    # non-dimensional moment of inertia tensor for fuel (solid rounded-end cylinder)
    I_fuel_nd = np.zeros((3, 3))
    V_fuel    = V_i_cyl + V_i_sph
    if V_fuel > 0:
        f_cyl = V_i_cyl / V_fuel
        f_sph = V_i_sph / V_fuel
        d_h   = L_i / 2 + (3 / 8) * R_i
        I_fuel_nd[0][0] = 0.5 * f_cyl * R_i**2 + 2 / 5 * f_sph * R_i**2
        I_fuel_nd[1][1] = f_cyl * (R_i**2 / 4 + L_i**2 / 12) + 2 / 5 * f_sph * R_i**2 + f_sph * d_h**2
        I_fuel_nd[2][2] = I_fuel_nd[1][1]
    fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor = I_fuel_nd

    return
    