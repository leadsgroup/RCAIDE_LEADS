# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/compute_integral_tank_volume.py
# 
# 
# Created: Aug 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import  RCAIDE  

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
def compute_rounded_end_cylindical_tank_volume(fuel_tank):  
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
         
    # volume of internal cavity
    V_i_cyl = (np.pi * R_i ** 2 * L_i)
    V_i_sph = (4 / 3 * np.pi * R_i ** 3)
    tank_volume_i = V_i_cyl + V_i_sph

    # volume of external shell
    V_o_cyl = (np.pi * R_o ** 2 * L_o)
    V_o_sph = (4 / 3 * np.pi * R_o ** 3)
    tank_volume_o = V_o_cyl + V_o_sph

    # Non-dimensional (I/mass) MOI computed from geometry alone; mass cancels out of the
    # underlying formulas, so this remains valid as the (changing) fuel mass burns down.
    I_local_tank = _non_dimensional_rounded_cylinder_moi(L_o, R_o, L_i, R_i, bwb_aft_tank=False)
    I_local_fuel = _non_dimensional_rounded_cylinder_moi(L_i, R_i, bwb_aft_tank=True)

    cg = [[(L_o + D) / 2, 0, 0]]

    # pack properties into fuel tank object
    fuel_tank.fuel.origin                                                     = fuel_tank.origin
    fuel_tank.mass_properties.center_of_gravity                              = cg
    fuel_tank.fuel.mass_properties.center_of_gravity                         = cg
    fuel_tank.mass_properties.moments_of_inertia.non_dimensional_tensor      = I_local_tank
    fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor = I_local_fuel
    fuel_tank.volume_properties.net_volume                                   = tank_volume_i
    fuel_tank.volume_properties.gross_volume                                 = tank_volume_o

    return 
