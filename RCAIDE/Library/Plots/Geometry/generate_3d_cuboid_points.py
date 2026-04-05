# RCAIDE/Library/Plots/Geometry/generate_3d_cuboid_points.py
#
# Created: Oct 2025, S Shekar
import numpy as np
from RCAIDE.Framework.Core import Data

def generate_3d_cuboid_points(component):
    """
    Generate VTK geometry for a cargo container.
    Expected columns in layout.object_coordinates (from 2D plot code):
        [.., 2:x, 3:y, 4:z?, 5:length, 6:width, 7:F_c, 8:B_c, 9:E_c, 10:seat, 11:Em_row, 12:Gal_Lav]
    """
    
    cuboid_points = np.zeros((4,4,3))
    
    l = component.length
    w = component.width
    h = component.height

    cuboid_points[0,:,0] = np.array([0, 0, 0, 0]) + component.origin[0][0]
    cuboid_points[0,:,1] = np.array([0, 0, 0, 0]) + component.origin[0][1]  
    cuboid_points[0,:,2] = np.array([0, 0, 0, 0]) + component.origin[0][2]  
 
    cuboid_points[1,:,0] = np.array([0, 0, 0, 0]) + component.origin[0][0]
    cuboid_points[1,:,1] = np.array([ -w/2, w/2, w/2, -w/2]) + component.origin[0][1]   
    cuboid_points[1,:,2] = np.array([-h/2, -h/2, h/2, h/2]) + component.origin[0][2]   

    cuboid_points[2,:,0] = np.array([l, l, l, l]) + component.origin[0][0]
    cuboid_points[2,:,1] =  np.array([ -w/2, w/2, w/2, -w/2])  + component.origin[0][1]  
    cuboid_points[2,:,2] = np.array([-h/2, -h/2, h/2, h/2]) + + component.origin[0][2]     

    cuboid_points[3,:,0] = np.array([l, l, l, l]) + component.origin[0][0]
    cuboid_points[3,:,1] = np.array([0, 0, 0, 0]) + component.origin[0][1]  
    cuboid_points[3,:,2] = np.array([0, 0, 0, 0]) + component.origin[0][2]       
    
    G = Data()         
    G.PTS  = cuboid_points
    
  
    return G 


