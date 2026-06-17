# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/Non_Integral_Tank/compute_prismatic_tank_volume.py
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
def compute_prismatic_tank_volume(fuel_tank):
    """
    Computes the volume of a Prismatic non-integral fuel tanks.
 
    Parameters
    ----------
    fuel_tank : Fuel_Tank
        Fuel tank object containing tank specifications
            - fuel : Fuel
                Fuel properties including density 
            - length : float
                Length of the tank (computed) 
            - width : float
                Width of the tank (computed) 
            - height : float
                Height of the tank (computed)  

    Returns
    -------
    volume : float
        Internal volume of the non-integral fuel tank 
    """
    
    l = fuel_tank.lengths.external
    w = fuel_tank.widths.external
    h = fuel_tank.heights.external
    t = fuel_tank.wall_thickness
     
    inner_length = l - 2 * t 
    inner_width  = w - 2 * t  
    inner_height = h - 2 * t            

    tank_volume_o = l * w * h
    tank_volume_i = inner_length * inner_width *  inner_height
 
    fuel_tank.volume_properties.net_volume         = tank_volume_i
    fuel_tank.volume_properties.gross_volume       = tank_volume_o

    if fuel_tank.fuel.mass_properties.mass != 0:
        actual_fuel_volume = fuel_tank.fuel.mass_properties.mass /  fuel_tank.fuel.density  
        if actual_fuel_volume > fuel_tank.volume_properties.net_volume + 1e-8 :
            print('Warning: Specified fuel mass greater than mass of fuel capable of being stored in fuel tank') 
    else:
        fuel_tank.fuel.mass_properties.mass         = tank_volume_i *  fuel_tank.fuel.density 
        fuel_tank.fuel.volume_properties.net_volume = tank_volume_i 
    
    fuel_tank.fuel.mass_properties.center_of_gravity  =  [[fuel_tank.lengths.external /2, 0, 0]] 
    fuel_tank.mass_properties.center_of_gravity       =  [[fuel_tank.lengths.external /2, 0, 0]]
    fuel_tank.fuel.origin                             = fuel_tank.origin
         
    return
