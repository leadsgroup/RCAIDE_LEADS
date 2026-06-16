
# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/compute_integral_tank_volume.py
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
def compute_wing_integral_tank_volume(fuel_tank,wing,n_points = 101,scale_factor = 0.85):
    """
    Computes the fuel volume for an integral fuel tank within a wing structure.

    This function calculates the volume of fuel that can be stored in an integral tank
    within the wing. It handles both single-segment and multi-segment wing configurations,
    updating the fuel tank's mass properties and center of gravity accordingly.

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        The fuel tank object containing fuel properties and mass characteristics
    wing : Wing
        The wing object containing segment geometry, airfoil data, and fuel tank specifications

    Returns
    -------
    volume : float
        The calculated fuel volume in cubic units

    Notes
    -----
    The function determines the fuel tank origin from the wing origin and calculates
    volume based on whether the wing has multiple segments or is a single segment.
    For multi-segment wings, it iterates through adjacent segments to find fuel tank
    locations and accumulates moments of inertia.

    **Major Assumptions**
        * Fuel tank geometry follows the wing's airfoil profile
        * Fuel density is uniform throughout the tank
        * Wing segments are properly connected and oriented

    **Theory**

    For multi-segment wings, the volume is calculated segment by segment using
    truncated prism geometry. The center of gravity is computed as a weighted
    average of segment centers.

    **Definitions**

    'Integral Wing Tank'
        A fuel tank built into the wing structure, typically within the wing box

    'Wing Box'
        The structural box formed by the front and rear spars of the wing
    """

    # get orgin of fuel tank
    fuel_tank.origin                  = wing.origin 
    fuel_tank.fuel.origin             = wing.origin  
    fuel_tank.fuel.xz_plane_symmetric = wing.xz_plane_symmetric
    fuel_tank.fuel.xy_plane_symmetric = wing.xy_plane_symmetric
    fuel_tank.fuel.yz_plane_symmetric = wing.yz_plane_symmetric 
    
    if len(wing.segments) < 2:
        raise AttributeError('compute_wing_integral_tank_volume requires a wing with at least two segments.')

    seg_bounds =  fuel_tank.segments_bounding_tank
    # Collect all segment tags between start and end (inclusive)
    collect = False
    seg_keys = []
    for segment in wing.segments:
        if segment.tag == seg_bounds[0]:
            collect = True
        if collect:
            seg_keys.append(segment.tag)
        if segment.tag == seg_bounds[1]:
            break

    if len(seg_keys) < 2:
        raise AttributeError(f'Fuel tank segments_bounding_tank {seg_bounds} did not resolve to at least two wing segments.')

    segment_meshes = []
    symm    = wing.xz_plane_symmetric

    for i in range(len(seg_keys)-1):
        # compute volume and assume unit density to get mass
        inner_segment = wing.segments[seg_keys[i]]
        outer_segment = wing.segments[seg_keys[i+1]] 

        # Compute segment span length
        L = (outer_segment.percent_span_location - inner_segment.percent_span_location) * wing.spans.projected/(symm + 1)
        spanwise_shift = inner_segment.percent_span_location * wing.spans.projected/2 
        
        airfoil_in = inner_segment.airfoil 
        if  airfoil_in !=  None:                 
            if type(airfoil_in) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                geometry_in = compute_naca_4series(airfoil_in.NACA_4_Series_code,n_points)
            elif type(airfoil_in) == RCAIDE.Library.Components.Airfoils.Airfoil: 
                geometry_in     = import_airfoil_geometry(airfoil_in.coordinate_file,n_points)
        else:
            geometry_in = compute_naca_4series('0012',n_points)

        airfoil_out = outer_segment.airfoil 
        if  airfoil_out !=  None:                 
            if type(airfoil_out) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                geometry_out = compute_naca_4series(airfoil_out.NACA_4_Series_code,n_points)
            elif type(airfoil_out) == RCAIDE.Library.Components.Airfoils.Airfoil: 
                geometry_out     = import_airfoil_geometry(airfoil_out.coordinate_file,n_points)
        else:
            geometry_out = compute_naca_4series('0012',n_points)
        
        
        start_distance_in = inner_segment.origin[0][0]+wing.segments[seg_keys[i]].root_chord_percent* wing.chords.root * (fuel_tank.segments_percent_chord_start[i])
        end_distance_in   = inner_segment.origin[0][0]+wing.segments[seg_keys[i]].root_chord_percent* wing.chords.root * (fuel_tank.segments_percent_chord_end[i])

        start_distance_out = outer_segment.origin[0][0]+wing.segments[seg_keys[i+1]].root_chord_percent* wing.chords.root * (fuel_tank.segments_percent_chord_start[i+1])
        end_distance_out   = outer_segment.origin[0][0]+wing.segments[seg_keys[i+1]].root_chord_percent* wing.chords.root * (fuel_tank.segments_percent_chord_end[i+1])
                            
        x_in  = np.array(geometry_in.x_coordinates)[:-1] * wing.chords.root *inner_segment.root_chord_percent + inner_segment.origin[0][0]
        y_in  = np.array(geometry_in.y_coordinates)[:-1] * wing.chords.root *inner_segment.root_chord_percent + inner_segment.origin[0][2]
        x_out = np.array(geometry_out.x_coordinates)[:-1] * wing.chords.root *outer_segment.root_chord_percent + outer_segment.origin[0][0]
        y_out = np.array(geometry_out.y_coordinates)[:-1] * wing.chords.root *outer_segment.root_chord_percent + outer_segment.origin[0][2]

        # ---------------- Inner segment ----------------
        mask_in = (x_in >= start_distance_in) & (x_in <= end_distance_in)

        x_in_capped = x_in[mask_in]
        y_in_capped = y_in[mask_in]
        
        # ---------------- Outer segment ----------------
        mask_out = (x_out >= start_distance_out) & (x_out <= end_distance_out)

        x_out_capped = x_out[mask_out]
        y_out_capped = y_out[mask_out]

        solid_segment =  compute_segment_meshes(x_in_capped,y_in_capped, x_out_capped, y_out_capped, L, spanwise_shift) 
        segment_meshes.append(solid_segment)
    
    combinde_mesh = trimesh.util.concatenate(segment_meshes)
    
    # Reflect across the YZ plane (mirror X)
    Ry = np.diag([1, -1, 1])   # reflection matrix

    # Compute centroid
    centroid = combinde_mesh.centroid

    # Create scaling transform about centroid
    T = trimesh.transformations.scale_matrix(
        scale_factor,
        origin=centroid
    )
    combinde_mesh.apply_transform(T)
    
    if wing.xz_plane_symmetric:
        # 1. copy the mesh
        combined_mesh_sym = deepcopy(combinde_mesh)

        # 2. apply the mirror transform
        combined_mesh_sym.vertices = (Ry @ combined_mesh_sym.vertices.T).T

        # 3. fix face orientation (reverse winding)
        combined_mesh_sym.faces = combined_mesh_sym.faces[:, ::-1]

        # 4. concatenate original + mirrored
        combined_mesh_full         = trimesh.util.concatenate([combinde_mesh, combined_mesh_sym]) 
    else:
        combined_mesh_full = combinde_mesh
    centroid = combined_mesh_full.centroid
    cg_x     = centroid[0]
    cg_y     = centroid[1]
    cg_z     = centroid[2]

    # Non-dimensional (per-unit-mass) MOI from geometry alone (trimesh assumes density = 1
    # by default, so moment_inertia/volume is density-independent)
    total_fuel_volume   = combined_mesh_full.volume
    I_non_dim            = combined_mesh_full.moment_inertia / total_fuel_volume

    # Bounding-box envelope of the usable fuel volume, used as the design envelope by
    # downstream conformal tank sizing (e.g. compute_cryogenic_tank_conformal_volume)
    extents = combined_mesh_full.extents

    # pack properties into fuel tank object
    fuel_tank.lengths.external                                                = extents[0]
    fuel_tank.widths.external                                                 = extents[1]
    fuel_tank.heights.external                                                = extents[2]
    fuel_tank.mass_properties.center_of_gravity                               = [[cg_x, cg_y, cg_z]]
    fuel_tank.mass_properties.moments_of_inertia.non_dimensional_tensor       = np.zeros((3, 3))
    fuel_tank.fuel.mass_properties.center_of_gravity                          = [[cg_x, cg_y, cg_z]]
    fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor  = I_non_dim
    fuel_tank.volume_properties.gross_volume                                  = total_fuel_volume
    fuel_tank.volume_properties.net_volume                                    = total_fuel_volume
    return
  