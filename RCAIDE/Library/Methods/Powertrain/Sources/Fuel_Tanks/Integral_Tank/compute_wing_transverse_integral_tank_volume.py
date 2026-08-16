
# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/Integral_Tank/compute_wing_transverse_integral_tank_volume.py
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
def compute_wing_transverse_integral_tank_volume(fuel_tank, wing,_):

    # Check if there are enough properties to accurately compute the maximum possible tank volume 
    if any(val is None for val in [
        fuel_tank.transverse_tank_chord_bounds[0],
        fuel_tank.transverse_tank_chord_bounds[1],
        fuel_tank.segments_bounding_tank ,
        fuel_tank.transverse_tank_segment_bound
        ]):
        raise ValueError("One or more required aft tank parameters are not set in 'fuel_tank'.")

    # ------------------------------------------------------
    # compute tank bounds
    # ------------------------------------------------------
    # root chord of refernce wing
    root_chord = wing.chords.root
    wing_span  = wing.spans.projected
    # where tank is located as a percentage of root chord
    tank_start_percent = fuel_tank.transverse_tank_chord_bounds[0]
    tank_end_percent   = fuel_tank.transverse_tank_chord_bounds[1]
    # create x coordinates where airfoils will be interpolated to find polygon of interest
    n = 2 # 2 gives us a straight tank
    # ------------------------------------------------------
    # loop through wing segments to get cooridates
    # ------------------------------------------------------
    num_tank_sections    = 0
    wing_segment_origins = np.empty((0, 3))
    segments             = wing.segments
        
    seg_tags = list(wing.segments.keys())
    index = seg_tags.index( fuel_tank.transverse_tank_segment_bound)
    seg_names = seg_tags[:index + 1]

    for _,tag in enumerate(seg_names):
        segment = wing.segments[tag]
        num_tank_sections += 1
        wing_segment_origins =  np.concatenate((wing_segment_origins, np.array(segment.origin)), axis=0)

    tank_end_percent_current = tank_end_percent

    # dimensionalized location of tank bounds
    tank_start_dimensional = tank_start_percent * root_chord
    tank_end_dimensional   = tank_end_percent_current * root_chord
    x_tank_bounds = np.linspace(tank_start_dimensional, tank_end_dimensional, n)
    # ------------------------------------------------------
    # loop through wing segments to get cooridates
    # ------------------------------------------------------
    # fig, ax = plt.subplots()
    polygon_points = []
    for seg_i in range(num_tank_sections):
        segment = segments[seg_names[seg_i]]
        if seg_i == 0:
            fuel_tank.wing_root_twist = segments[seg_names[seg_i]].twist
        if segment.airfoil != None:
            if type(segment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                geometry = compute_naca_4series(segment.airfoil.NACA_4_Series_code)
            elif type(segment.airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil:
                geometry = import_airfoil_geometry(segment.airfoil.coordinate_file)
        else:
            geometry = compute_naca_4series('0012')
        # Get segment chord
        segment_chord = segments[seg_names[seg_i]].root_chord_percent * root_chord
        # Get upper and lower points and scale by chord
        x_points_upper = segment_chord * geometry.x_upper_surface
        x_points_lower = segment_chord * geometry.x_lower_surface
        y_points_upper = segment_chord * geometry.y_upper_surface
        y_points_lower = segment_chord * geometry.y_lower_surface
        # position points correct using segment origin (this is based on sweep and dihedral)
        x_points_upper_positioned = x_points_upper + wing_segment_origins[seg_i][0]
        x_points_lower_positioned = x_points_lower + wing_segment_origins[seg_i][0]
        y_points_upper_positioned = y_points_upper + wing_segment_origins[seg_i][2] - fuel_tank.wall_clearance
        y_points_lower_positioned = y_points_lower + wing_segment_origins[seg_i][2] + fuel_tank.wall_clearance
        # Use interpolant to evaluate polygon points
        upper_y_function = interp1d(x_points_upper_positioned, y_points_upper_positioned, kind='linear')
        lower_y_function = interp1d(x_points_lower_positioned, y_points_lower_positioned, kind='linear')
        upper_y_raw      = upper_y_function(x_tank_bounds)
        lower_y_raw      = lower_y_function(x_tank_bounds)
        # Reflexed airfoils can have upper_y < lower_y near the trailing edge;
        # clamp so the polygon is always non-self-intersecting.
        upper_y_points   = np.maximum(upper_y_raw, lower_y_raw)
        lower_y_points   = np.minimum(upper_y_raw, lower_y_raw)

        # Create polygon
        polygon = []
        # upper points
        idx = 0
        for polygon_corner in range(n):
            polygon.append((x_tank_bounds[polygon_corner], upper_y_points[polygon_corner]))
            idx += 1
        # lower points
        for polygon_corner_rev in range(n-1, -1, -1):
            polygon.append((x_tank_bounds[polygon_corner_rev], lower_y_points[polygon_corner_rev]))
            idx += 1
        # close polygon
        polygon.append((x_tank_bounds[0], upper_y_points[0]))
        # store polygon points
        polygon_points.append(polygon) 

    fuel_tank.transverse_tank_chord_bounds[1] = tank_end_percent_current
    # ------------------------------------------------------
    # Compute prismatic volumes from intersections
    # ------------------------------------------------------
    tank_volumes = np.zeros(num_tank_sections - 1)
    tank_lengths = np.zeros(num_tank_sections - 1)
    intersection_polygons = []

    for seg_i in range(1, num_tank_sections):

        if seg_i == 1:
            inner_polygon = Polygon(polygon_points[seg_i - 1]).buffer(0)
        else:
            inner_polygon = intersection_polygon

        outer_polygon = Polygon(polygon_points[seg_i]).buffer(0)

        intersection_polygon = inner_polygon.intersection(outer_polygon)

        if intersection_polygon.is_empty:
            tank_volumes[seg_i - 1] = 0.0
            tank_lengths[seg_i - 1] = 0.0
            intersection_polygons.append(None)
            continue

        if not isinstance(intersection_polygon, shapely.geometry.Polygon):
            polys = [g for g in getattr(intersection_polygon, 'geoms', []) if isinstance(g, shapely.geometry.Polygon)]
            intersection_polygon = max(polys, key=lambda g: g.area) if polys else None
        if intersection_polygon is None:
            tank_volumes[seg_i - 1] = 0.0
            tank_lengths[seg_i - 1] = 0.0
            intersection_polygons.append(None)
            continue

        area = intersection_polygon.area

        y_curr = segments[seg_names[seg_i]].percent_span_location * wing_span
        y_prev = segments[seg_names[seg_i - 1]].percent_span_location * wing_span
        span_length = y_curr - y_prev

        volume = area * span_length

        tank_volumes[seg_i - 1] = volume
        tank_lengths[seg_i - 1] = span_length
        intersection_polygons.append(intersection_polygon)

    # ------------------------------------------------------
    # Get maximum volume section
    # ------------------------------------------------------
    max_volume = np.max(tank_volumes)
    max_idx = np.argmax(tank_volumes)

    fuel_tank.volume_external = max_volume
    fuel_tank.length_external = tank_lengths[max_idx]

    # ------------------------------------------------------
    # Plot shaded intersection polygon
    # ------------------------------------------------------
    best_polygon = intersection_polygons[max_idx]
    if best_polygon is None:
        raise AttributeError("No valid intersection polygon found for aft tank volume.")

    polygon_for_calc = best_polygon
    if best_polygon.geom_type == 'MultiPolygon':
        polygon_for_calc = max(best_polygon.geoms, key=lambda g: g.area)

    # Extract coordinates
    coords = list(polygon_for_calc.exterior.coords)

    # Remove duplicate last point (Shapely closes polygon automatically)
    if np.allclose(coords[0], coords[-1]):
        coords = coords[:-1]

    # Ensure it's 4-sided
    if len(coords) != 4:
        raise AttributeError(f"Polygon has {len(coords)} sides, not 4.")

    # Compute edge lengths
    edge_lengths = []
    for i in range(len(coords)):
        x1, y1 = coords[i]
        x2, y2 = coords[(i + 1) % len(coords)]
        length = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
        edge_lengths.append(length)

    fuel_tank.max_volume_intersection_edge_lengths = np.array(edge_lengths)
    fuel_tank.max_volume_intersection_num_edges    = int(len(edge_lengths)) 
    fuel_tank.widths.external                      = (edge_lengths[0]+edge_lengths[2])/2
    fuel_tank.lengths.external                     = fuel_tank.length_external
    fuel_tank.heights.external                     = (edge_lengths[1]+edge_lengths[3])/2 
    fuel_tank.aspect_ratio                         = fuel_tank.lengths.external /fuel_tank.heights.external 
    fuel_tank.volume_properties.net_volume         = max_volume
    fuel_tank.volume_properties.gross_volume       = max_volume

    if fuel_tank.fuel.mass_properties.mass != 0:
        actual_fuel_volume = fuel_tank.fuel.mass_properties.mass /  fuel_tank.fuel.density
        if actual_fuel_volume > fuel_tank.volume_properties.net_volume + 1e-8 :
            print('Warning:Specified fuel mass greater than mass of fuel capable of being stored in fuel tank')
        fuel_tank.fuel.volume_properties.net_volume = max_volume
    else:
        fuel_tank.fuel.mass_properties.mass         = max_volume *  fuel_tank.fuel.density
        fuel_tank.fuel.volume_properties.net_volume = max_volume
   
    # Build a 3D tank mesh by extruding the 2D section over length_external.
    # Extrusion is centered about y = 0 (symmetric about origin in spanwise axis).
    tank_mesh = trimesh.creation.extrude_polygon(
        polygon=polygon_for_calc,
        height=float(fuel_tank.length_external),
    )
    R = trimesh.transformations.rotation_matrix(-np.pi / 2.0, [1.0, 0.0, 0.0])
    T = trimesh.transformations.translation_matrix(
        [0.0, -0.5 * float(fuel_tank.length_external), 0.0]
    )
  
    tank_mesh.apply_transform(R)
    tank_mesh.apply_transform(T)

    centroid = np.asarray(tank_mesh.center_mass, dtype=float)
    cg_x     = centroid[0]
    cg_y     = 0
    cg_z     = centroid[2]

    # Non-dimensional moment of inertia (I/mass) using unit density
    tank_mesh.density = 1.0
    I_fuel_nd = tank_mesh.moment_inertia / tank_mesh.mass

    fuel_tank.mass_properties.center_of_gravity                              = [[cg_x, cg_y, cg_z]]
    fuel_tank.fuel.mass_properties.center_of_gravity                         = [[cg_x, cg_y, cg_z]]
    fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor = I_fuel_nd
    fuel_tank.origin                                                         = [[cg_x, cg_y, cg_z]]
    fuel_tank.fuel.origin                                                    = fuel_tank.origin
    
    return
