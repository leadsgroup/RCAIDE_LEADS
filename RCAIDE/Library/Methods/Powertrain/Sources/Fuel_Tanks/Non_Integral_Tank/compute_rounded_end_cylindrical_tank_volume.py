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
    tank_volume_i     = V_i_cyl + V_i_sph
    
    # volume of interal walls 
    V_o_cyl = (np.pi * R_o ** 2 * L_o )  
    V_o_sph = ( 4 / 3 * np.pi * R_o ** 3)  
    tank_volume_o     = V_o_cyl + V_o_sph    
 
    fuel_tank.volume_properties.net_volume         = tank_volume_i
    fuel_tank.volume_properties.gross_volume       = tank_volume_o

    if fuel_tank.fuel.mass_properties.mass != 0:
        actual_fuel_volume = fuel_tank.fuel.mass_properties.mass /  fuel_tank.fuel.density  
        if actual_fuel_volume > fuel_tank.volume_properties.net_volume + 1e-8 :
            print('Warning:Specified fuel mass greater than mass of fuel capable of being stored in fuel tank') 
    else:
        fuel_tank.fuel.mass_properties.mass         = tank_volume_i *  fuel_tank.fuel.density 
        fuel_tank.fuel.volume_properties.net_volume = tank_volume_i 
    
    fuel_tank.fuel.mass_properties.center_of_gravity  =  [[(L_o + D)/2, 0, 0]] 
    fuel_tank.mass_properties.center_of_gravity       =  [[(L_o + D)/2, 0, 0]]
    fuel_tank.fuel.origin                             = fuel_tank.origin    
    return 
    