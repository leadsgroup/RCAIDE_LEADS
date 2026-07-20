
# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/Integral_Tank/compute_fuselage_integral_tank_volume.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Aug 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import  RCAIDE 
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry,  compute_naca_4series 
from RCAIDE.Library.Methods.Geometry.Planform import compute_segment_meshes

# Python Imports 
import numpy as np
from scipy.interpolate import interp1d 
from shapely import Polygon
import shapely
import matplotlib.pyplot as plt
import trimesh
from copy import deepcopy

# ----------------------------------------------------------------------------------------------------------------------
#  METHOD
# ----------------------------------------------------------------------------------------------------------------------  
def compute_fuselage_integral_tank_volume(fuel_tank,fuselage):
    """
    Computes the fuel volume for an integral fuel tank within a fuselage structure.

    This function calculates the volume of fuel that can be stored in an integral tank
    located between two fuselage segments. The calculation assumes a truncated cone
    geometry between the inner and outer segments.

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        The fuel tank object containing fuel properties and mass characteristics
    fuselage : Fuselage
        The fuselage object containing segment geometry and positioning data

    Returns
    -------
    volume : float
        The calculated fuel volume in cubic meters

    Notes
    -----
    The function iterates through fuselage segments to find adjacent segments where
    the inner segment has a fuel tank. The volume calculation uses the truncated
    cone formula for the space between two elliptical cross-sections.

    **Major Assumptions**
        * Fuel tank spans exactly between two adjacent fuselage segments
        * Fuselage cross-sections are elliptical
        * Fuel density is uniform throughout the tank

    **Theory**

    The volume of a truncated cone is calculated using:

    :math:`V = \\frac{1}{3} \\left( A_1 + A_2 + \\sqrt{A_1 A_2} \\right) h`

    where:
        - :math:`A_1` is the area of the inner segment cross-section
        - :math:`A_2` is the area of the outer segment cross-section  
        - :math:`h` is the height (length) between segments

    **Definitions**

    'Integral Tank'
        A fuel tank that is built into the structure of the aircraft rather than being a separate container

    'Truncated Cone'
        A cone with the top cut off by a plane parallel to the base
    """
    total_fuel_mass  = 0
    tank_volume_o    = 0
    tank_volume_i    = 0
    origin_x         = 100
    origin_y         = 0
    origin_z         = 0

    if len(fuselage.segments) > 1:
        segment_tank_moment = np.array([0.0, 0.0, 0.0])
        seg_bounds = fuel_tank.segments_bounding_tank 

        # Collect all segment tags between start and end (inclusive)
        collect = False
        seg_tags = []
        for segment in fuselage.segments:
            if segment.tag == seg_bounds[0]:
                collect = True
            if collect:
                seg_tags.append(segment.tag)
            if segment.tag == seg_bounds[1]:
                break
        
        for i in range(len(seg_tags)-1):
            inner_segment = fuselage.segments[seg_tags[i]]
            outer_segment = fuselage.segments[seg_tags[i+1]]
        
            h        = fuselage.lengths.total * (outer_segment.percent_x_location  - inner_segment.percent_x_location) 
            # volume of truncated cylinder 
            A_1_o    = np.pi * inner_segment.height /2  *  inner_segment.width/2
            A_2_o    = np.pi * outer_segment.height/2   *  outer_segment.width/2
            volume_o = (1 /3) * ( A_1_o + A_2_o + np.sqrt(A_1_o*A_2_o)) *h

            A_1_i    = np.pi * inner_segment.height /2  *  inner_segment.width/2
            A_2_i    = np.pi * outer_segment.height/2   *  outer_segment.width/2 
            volume_i = (1 /3) * ( A_1_i + A_2_i + np.sqrt(A_1_i*A_2_i)) *h
                
            total_fuel_mass        += volume_i * fuel_tank.fuel.density  
            segment_cg             = np.array([[fuselage.lengths.total * (inner_segment.percent_x_location  + outer_segment.percent_x_location)/2 ,0, \
                                         (inner_segment.height  + outer_segment.height)/2]])
            segment_tank_moment    += segment_cg[0] * volume_i * fuel_tank.fuel.density  
            tank_volume_i          += volume_i
            tank_volume_o          += volume_o

            if fuselage.lengths.total * inner_segment.percent_x_location < origin_x: 
                origin_x = fuselage.lengths.total * inner_segment.percent_x_location 
                origin_y = inner_segment.percent_y_location *fuselage.lengths.total    
                origin_z = inner_segment.percent_z_location *fuselage.lengths.total                    
            
        fuel_tank.fuel.mass_properties.center_of_gravity  = [list(segment_tank_moment / total_fuel_mass) ]
        fuel_tank.volume_properties.net_volume       = tank_volume_i
        fuel_tank.volume_properties.gross_volume     = tank_volume_o
     
        # update orign of tank 
        fuel_tank.origin      = [[origin_x, origin_y, origin_z]]             
        fuel_tank.fuel.origin = [[origin_x, origin_y, origin_z]]  

        # Compute the non-dimensional MOI (I/mass) of fuel using unit density
        collect    = False
        seg_tags   = []
        seg_bounds = fuel_tank.segments_bounding_tank

        for segment in fuselage.segments:
            if segment.tag == seg_bounds[0]:
                collect = True
            if collect:
                seg_tags.append(segment.tag)
            if segment.tag == seg_bounds[1]:
                break

        tessellation = 20
        num_fus_segs = len(seg_tags)
        fuselage_points = np.zeros((num_fus_segs * tessellation, 3))

        for i_seg, segment in enumerate(seg_tags):
            segment   = fuselage.segments[segment]
            segment_0 = fuselage.segments[seg_tags[0]]
            a         = segment.width / 2
            b         = segment.height / 2
            n         = segment.curvature
            theta     = np.linspace(0, 2 * np.pi, tessellation)
            fus_ypts  = (abs(np.cos(theta))**(2 / n)) * a * ((np.cos(theta) > 0) * 1 - (np.cos(theta) < 0) * 1)
            fus_zpts  = (abs(np.sin(theta))**(2 / n)) * b * ((np.sin(theta) > 0) * 1 - (np.sin(theta) < 0) * 1)

            start_idx = i_seg * tessellation
            end_idx   = (i_seg + 1) * tessellation
            fuselage_points[start_idx:end_idx, 0] = (segment.percent_x_location - segment_0.percent_x_location) * fuselage.lengths.total
            fuselage_points[start_idx:end_idx, 1] = fus_ypts + segment.percent_y_location * fuselage.lengths.total + fuselage.origin[0][1]
            fuselage_points[start_idx:end_idx, 2] = fus_zpts + segment.percent_z_location * fuselage.lengths.total + fuselage.origin[0][2]

        # Convex hull → watertight volume mesh
        solid_segment = trimesh.convex.convex_hull(fuselage_points)

        # Rotate to match the RCAIDE aircraft axes convention
        R = trimesh.transformations.rotation_matrix(np.deg2rad(90), [1, 0, 0], [0, 0, 0])
        solid_segment.apply_transform(R)

        # Non-dimensional moment of inertia (I/mass) using unit density
        solid_segment.density = 1.0
        I_fuel_nd = solid_segment.moment_inertia / solid_segment.mass

        fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor = I_fuel_nd

    return 
