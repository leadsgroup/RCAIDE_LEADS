# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/compute_integral_tank_volume.py
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
def compute_bwb_aft_tank_volume(fuel_tank, wing,fuel_tanks):
    """
    Computes the volume of an aft fuel tank for a Blended Wing Body (BWB) aircraft configuration.

    This function calculates the maximum possible fuel tank volume that can fit within the aft
    section of a BWB wing, considering airfoil geometry, structural constraints, and tank dimensions.
    The tank is designed as a cylindrical tank with rounded ends positioned within the aft portion
    of the wing segments.

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        Fuel tank object containing tank specifications and parameters
            - aft_tank_start_root_chord : float
                Starting position of aft tank as fraction of root chord
            - aft_tank_end_rood_chord : float
                Ending position of aft tank as fraction of root chord
            - aft_tank_end_segment_tag : str
                Tag of the wing segment where aft tank ends
            - wing_root_tag : str
                Tag of the root wing segment
            - radial_offset : float
                Radial clearance from wing structure
            - wall_thickness : float
                Thickness of tank walls
            - fuel : Fuel
                Fuel properties including density
            - orientation_euler_angles : list
                Euler angles defining tank orientation
    wing : Wing
        Wing object containing segment geometry and airfoil data
            - segments : dict
                Dictionary of wing segments with their properties
            - chords.root : float
                Root chord length
            - spans.projected : float
                Projected wing span

    Returns
    -------
    volume : float
        Maximum possible internal volume of the aft fuel tank

    Notes
    -----
    The function processes multiple wing segments to determine the optimal tank dimensions.
    It uses airfoil coordinate data to find the largest possible circular cross-section
    that fits within the wing geometry at each spanwise location.

    **Major Assumptions**
        * Tank is cylindrical with rounded ends
        * Tank is symmetric about the aircraft centerline
        * Airfoil coordinate files are available and properly formatted
        * Wing segments are properly defined with airfoil data

    **Theory**

    The tank volume is calculated as the sum of a cylindrical section and hemispherical end caps:
    
    .. math::
        V = \\pi r^2 l + \\frac{4}{3}\\pi r^3

    where r is the tank radius and l is the cylindrical length.
    """
    # Check if there are enough properties to accurately compute the maximum possible tank volume 
    if any(val is None for val in [
        fuel_tank.aft_tank_root_chord_bounds[0],
        fuel_tank.aft_tank_root_chord_bounds[1],
        fuel_tank.segments_bounding_tank ,
        fuel_tank.aft_tank_segment_bound
        ]):
        raise ValueError("One or more required aft tank parameters are not set in 'fuel_tank'.")
    if hasattr( wing, 'aft_tank_end_percent'):
        if  (fuel_tank.aft_tank_root_chord_bounds[1] - wing.aft_tank_end_percent - 0.005)*wing.chords.root > 0.25: # Successive Aft Tank diameter needs to be atleast 0.25m
            fuel_tank.aft_tank_root_chord_bounds[0] = wing.aft_tank_end_percent + 0.005
        else:
            print(f"[WARNING] Tank '{fuel_tank.tag}' cannot not fit in the space. Removing from list.")
            fuel_tanks.pop(fuel_tank.tag)
            return
    # ------------------------------------------------------
    # compute tank bounds
    # ------------------------------------------------------
    # root chord of refernce wing
    root_chord = wing.chords.root
    wing_span  = wing.spans.projected
    # where tank is located as a percentage of root chord
    tank_start_percent = fuel_tank.aft_tank_root_chord_bounds[0]
    tank_end_percent   = fuel_tank.aft_tank_root_chord_bounds[1]
    # create x coordinates where airfoils will be interpolated to find polygon of interest
    n = 5
    # ------------------------------------------------------
    # loop through wing segments to get cooridates
    # ------------------------------------------------------
    num_tank_sections    = 0
    wing_segment_origins = np.empty((0, 3))
    segments             = wing.segments
        
    seg_tags = list(wing.segments.keys())
    index = seg_tags.index( fuel_tank.aft_tank_segment_bound)
    seg_names = seg_tags[:index + 1]

    for _,tag in enumerate(seg_names):
        segment = wing.segments[tag]
        num_tank_sections += 1
        wing_segment_origins =  np.concatenate((wing_segment_origins, np.array(segment.origin)), axis=0)

    tank_end_percent_current = tank_end_percent
    while True:
        try:
            # dimensionalized location of tank bounds
            tank_start_dimensional = tank_start_percent * root_chord
            tank_end_dimensional   = tank_end_percent_current * root_chord
            x_tank_bounds = np.linspace(tank_start_dimensional, tank_end_dimensional, n)
            # ------------------------------------------------------
            # loop through wing segments to get cooridates
            # ------------------------------------------------------
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
                upper_y_points   = upper_y_function(x_tank_bounds)
                lower_y_points   = lower_y_function(x_tank_bounds)
                
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
                # test polygon
                # Extract x and y coordinates into separate lists for plotting
                x_coords = [p[0] for p in polygon_points[seg_i]]
                y_coords = [p[1] for p in polygon_points[seg_i]]
            fuel_tank.aft_tank_root_chord_bounds[1] = tank_end_percent_current
            break
        except Exception:
            tank_end_percent_current -= 0.01
            if tank_end_percent_current <= tank_start_percent:
                raise ValueError("Unable to compute aft tank polygon bounds after reducing aft_tank_root_chord_bounds[1].")
    # ------------------------------------------------------------------------------------------------------
    # Iteratively get maximum inscribed circle between wing segment circles and store volume
    # ------------------------------------------------------------------------------------------------------
    tank_radii     = np.zeros(num_tank_sections-1)
    tank_volumes   = np.zeros(num_tank_sections-1)
    tank_lengths   = np.zeros(num_tank_sections-1)
    circle_origins = np.zeros((num_tank_sections-1, 2))
    for seg_i in  range(1,num_tank_sections):
        if seg_i == 1:
            inner_polygon =  Polygon( polygon_points[seg_i-1] )
        else:
            inner_polygon = intersection_polygon
        outer_polygon =  Polygon(  polygon_points[seg_i] )
        # intersection polygon
        intersection_polygon = inner_polygon.intersection(outer_polygon)
        intersection_polygon.exterior.coords.xy
        # maximum radius
        poly             = Polygon(intersection_polygon)
        inscribed_circle =  shapely.maximum_inscribed_circle(poly)
        circle_center_x  =  inscribed_circle.coords[0][0]
        circle_center_y  =  inscribed_circle.coords[0][1]
        boundary_x       =  inscribed_circle.coords[1][0]
        boundary_y       = inscribed_circle.coords[1][1]
        tank_radius      =  np.sqrt( (boundary_x - circle_center_x) ** 2 + (boundary_y - circle_center_y) ** 2 )
        # store radius
        tank_radii[seg_i-1] = tank_radius
        # compute and store volume
        l_total                 =  segments[seg_names[seg_i]].percent_span_location *  wing_span
        height                  = l_total - 2 *tank_radius
        volume                  = 4/3 *np.pi * (tank_radius ** 3) +   np.pi * (tank_radius ** 2) *  height
        tank_volumes[seg_i-1]   =  volume
        tank_lengths[seg_i-1]   = height
        circle_origins[seg_i-1][0] = circle_center_x
        circle_origins[seg_i-1][1] = circle_center_y
    # ------------------------------------------------------------------------------------------------------
    # Get Maximum volume and corresponding properties
    # ------------------------------------------------------------------------------------------------------
    max_volume = np.max(tank_volumes)
    max_idx    = np.argmax(tank_volumes)
    radius_opt = tank_radii[max_idx]
    length_opt = tank_lengths[max_idx]
    fuel_tank.diameters.external = radius_opt * 2
    fuel_tank.lengths.external   = length_opt
    fuel_tank.diameters.internal = radius_opt * 2
    
    # Outer Volume
    tank_volume_o                = max_volume
    fuel_tank.aspect_ratio       = (fuel_tank.lengths.external +fuel_tank.diameters.external )/fuel_tank.diameters.external

    # Inner Volume
    fuel_tank.lengths.internal   = (fuel_tank.aspect_ratio * fuel_tank.diameters.internal) -fuel_tank.diameters.internal
    tank_volume_i                = max_volume
    fuel_tank.volume_properties.net_volume         = tank_volume_i
    fuel_tank.volume_properties.gross_volume       = tank_volume_o
    if fuel_tank.fuel.mass_properties.mass != 0:
        actual_fuel_volume = fuel_tank.fuel.mass_properties.mass /  fuel_tank.fuel.density
        if actual_fuel_volume > fuel_tank.volume_properties.net_volume + 1e-8 :
            print('Warning:Specified fuel mass greater than mass of fuel capable of being stored in fuel tank')
        fuel_tank.fuel.volume_properties.net_volume = tank_volume_i
    else:
        fuel_tank.fuel.mass_properties.mass         = tank_volume_i *  fuel_tank.fuel.density
        fuel_tank.fuel.volume_properties.net_volume = tank_volume_i
    # fuel tank origin
    fuel_tank.origin[0][0]  = circle_origins[max_idx][0] - fuel_tank.diameters.external / 2
    fuel_tank.origin[0][1]  = 0
    fuel_tank.origin[0][2]  = circle_origins[max_idx][1]
    # fuel tank C.G.
    fuel_tank.fuel.mass_properties.center_of_gravity  =  [[fuel_tank.lengths.external /2, 0, 0]]
    fuel_tank.mass_properties.center_of_gravity       =  [[fuel_tank.lengths.external /2, 0, 0]]
    if fuel_tank.orientation_euler_angles   == [0.,0.,np.pi/2]:
        fuel_tank.fuel.mass_properties.center_of_gravity  =  [[fuel_tank.diameters.external /2, 0, 0]]
        fuel_tank.mass_properties.center_of_gravity       =  [[fuel_tank.diameters.external /2, 0, 0]]
        
    fuel_tank.fuel.origin = fuel_tank.origin
    fuel_tank.fuel.xz_plane_symmetric = wing.xz_plane_symmetric
    fuel_tank.fuel.xy_plane_symmetric = wing.xy_plane_symmetric
    fuel_tank.fuel.yz_plane_symmetric = wing.yz_plane_symmetric 
    wing.aft_tank_end_percent =  (fuel_tank.origin[0][0] + fuel_tank.diameters.external )/wing.chords.root
    
    return

def compute_prismatic_fuel_tank_volume(fuel_tank):
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
    
def compute_wing_non_integral_tank_volume(fuel_tank, wing,fuel_tanks):
    """
    Computes the volume of non-integral fuel tanks within wing segments.

    This function iterates through wing segments to find suitable locations for non-integral
    fuel tanks and calculates their volumes. It handles cases where tanks cannot be placed
    in specified segments and attempts placement in subsequent segments.

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        Fuel tank object containing tank specifications
            - fuel : Fuel
                Fuel properties including density
            - symmetric : bool
                Whether the tank is symmetric about aircraft centerline
            - length : float
                Length of the tank (computed)
            - outer_diameter : float
                Outer diameter of the tank (computed)
    wing : Wing
        Wing object containing segment geometry
            - segments : dict
                Dictionary of wing segments with their properties
            - spans.projected : float
                Projected wing span

    Returns
    -------
    volume : float
        Net volume of the non-integral fuel tank

    Notes
    -----
    The function attempts to place tanks in wing segments that have fuel tank capability.
    If placement fails in one segment, it tries the next available segment.

    **Major Assumptions**
        * Wing segments are properly ordered from root to tip
        * At least one wing segment has fuel tank capability
        * Tank placement constraints are reasonable
    """ 
    if len(wing.segments) > 1: 
        seg_tags = fuel_tank.segments_bounding_tank  
        for i in range(len(seg_tags)-1):
            inner_segment = wing.segments[seg_tags[i]]
            outer_segment = wing.segments[seg_tags[i+1]] 
            try:
                try:
                    tank_percent_span_location = inner_segment.tank_percent_span_location    
                except:
                    tank_percent_span_location = 0
                inner_segment.tank_percent_span_location, tank_volume_o, tank_volume_i\
                                        = compute_wing_non_integral_tank_fuel_volume(fuel_tank,wing,inner_segment,outer_segment,tank_percent_span_location)
            except:
                print(f"[WARNING] Tank '{fuel_tank.tag}' does not fit in the segment. Removing from list.")
                fuel_tanks.pop(fuel_tank.tag)
                return 
             
        fuel_tank.volume_properties.net_volume         = tank_volume_i
        fuel_tank.volume_properties.gross_volume       = tank_volume_o

        fuel_tank.mass_properties.center_of_gravity       =  [[(fuel_tank.lengths.external + fuel_tank.diameters.external) /2, 0,0]]     
        fuel_tank.fuel.mass_properties.center_of_gravity  =  [[(fuel_tank.lengths.external + fuel_tank.diameters.external) /2, 0,0]]     
        fuel_tank.mass_properties.center_of_gravity       =  [[(fuel_tank.lengths.external + fuel_tank.diameters.external) /2, 0,0]]   

        if not isinstance(fuel_tank, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank):
            if fuel_tank.fuel.mass_properties.mass != 0:
                actual_fuel_volume = fuel_tank.fuel.mass_properties.mass /  fuel_tank.fuel.density  
                if actual_fuel_volume > fuel_tank.volume_properties.net_volume + 1e-8 :
                    print('Warning:Specified fuel mass greater than mass of fuel capable of being stored in fuel tank') 
                fuel_tank.fuel.volume_properties.net_volume = tank_volume_i
            else:
                fuel_tank.fuel.mass_properties.mass         = tank_volume_i *  fuel_tank.fuel.density 
                fuel_tank.fuel.volume_properties.net_volume = tank_volume_i
             
    return 

def compute_wing_non_integral_tank_fuel_volume(fuel_tank, wing, inner_segment_0, outer_segment, tank_percent_span_location):
    """
    Computes the fuel volume for a non-integral tank between two wing segments.

    This function calculates the optimal tank dimensions and volume that can fit between
    two wing segments, considering wing geometry, structural constraints, and tank specifications.
    The tank is designed as a cylindrical tank with hemispherical end caps.

    Parameters
    ----------
    fuel_tank : Fuel_Tank
        Fuel tank object containing tank specifications
            - wall_thickness : float
                Thickness of tank walls
            - symmetric : bool
                Whether the tank is symmetric about aircraft centerline
            - fuel : Fuel
                Fuel properties including density
    wing : Wing
        Wing object containing geometry and span information
            - chords.root : float
                Root chord length
            - spans.projected : float
                Projected wing span
    inner_segment_0 : Wing_Segment
        Initial inner wing segment for tank placement
            - percent_span_location : float
                Spanwise location as fraction of total span
            - root_chord_percent : float
                Root chord as fraction of wing root chord
            - origin : list
                Origin coordinates of the segment
            - sweeps.leading_edge : float
                Leading edge sweep angle
            - dihedral_outboard : float
                Outboard dihedral angle
            - fuel_tank : Fuel_Tank_Segment
                Fuel tank segment properties
    outer_segment : Wing_Segment
        Outer wing segment defining tank boundary
    tank_percent_span_location : float
        Current spanwise location of tank as fraction of total span

    Returns
    -------
    volume : float
        Net volume of the fuel tank
    tank_percent_span_location : float
        Updated spanwise location for next tank placement

    Notes
    -----
    The function uses an iterative approach to find the optimal tank diameter that fits
    within the wing geometry constraints. It considers wing sweep, dihedral, and structural
    clearances in the calculation.

    **Major Assumptions**
        * Tank is cylindrical with hemispherical end caps
        * Wing segments have linear variation in geometry
        * Structural clearances are maintained
        * Tank placement follows wing sweep and dihedral

    **Theory**

    The tank volume is calculated as:
    
    .. math::
        V = \\pi r^2 (l - D) + \\frac{4}{3}\\pi r^3

    where r is the internal radius, l is the tank length, and D is the tank diameter.

    **Definitions**

    'Non-Integral Tank'
        Fuel tank that is not structurally integrated with the wing, typically mounted
        between wing ribs or spars
    """

    semi_span      = wing.spans.projected / 2
    inner_segment  = deepcopy(inner_segment_0) 
    spar_sweep     = convert_sweep_segments(inner_segment_0.sweeps.quarter_chord, inner_segment_0, outer_segment, wing, old_ref_chord_fraction=0.25, new_ref_chord_fraction=fuel_tank.percent_span_location  )     
    if tank_percent_span_location > inner_segment_0.percent_span_location: 
        inner_segment.percent_span_location = tank_percent_span_location
        m                                   =  (outer_segment.root_chord_percent -  inner_segment_0.root_chord_percent) / (outer_segment.percent_span_location - fuel_tank.percent_span_location)
        delta_y_percent                     =  (tank_percent_span_location - inner_segment_0.percent_span_location)
        inner_segment.root_chord_percent    = inner_segment_0.root_chord_percent + m*delta_y_percent

        # update segment origin
        delta_y                    = delta_y_percent * semi_span
        inner_segment.origin[0][0] = inner_segment_0.origin[0][0] + delta_y * np.tan(inner_segment_0.sweeps.leading_edge) 
        inner_segment.origin[0][1] = inner_segment.percent_span_location * semi_span
        inner_segment.origin[0][2] = inner_segment_0.origin[0][2] +  delta_y *np.tan(inner_segment.dihedral_outboard)

    inner_front_rib_yu,inner_rear_rib_yu,inner_front_rib_yl,inner_rear_rib_yl = compute_non_dimensional_rib_coordinates(inner_segment,fuel_tank,fuel_tank.segments_percent_chord_start[0], fuel_tank.segments_percent_chord_end[0])
    inner_segment_chord     = wing.chords.root * inner_segment.root_chord_percent
    inner_front_rib_length  = inner_segment_chord * (abs(inner_front_rib_yu) + abs(inner_front_rib_yl)) 
    inner_rear_rib_length   = inner_segment_chord * (abs(inner_rear_rib_yu) + abs(inner_rear_rib_yl) )
    inner_wingbox_length    = inner_segment_chord * (fuel_tank.segments_percent_chord_end[0] -fuel_tank.segments_percent_chord_start[0]) 

    clearance  = fuel_tank.wall_clearance
    delta_span = (outer_segment.percent_span_location - inner_segment.percent_span_location) * semi_span

    outer_front_rib_yu,outer_rear_rib_yu,outer_front_rib_yl,outer_rear_rib_yl = compute_non_dimensional_rib_coordinates(outer_segment,fuel_tank,fuel_tank.segments_percent_chord_start[1], fuel_tank.segments_percent_chord_end[1])
    outer_segment_chord     = wing.chords.root * outer_segment.root_chord_percent
    outer_front_rib_length  = outer_segment_chord * (abs(outer_front_rib_yu) + abs(outer_front_rib_yl)) 
    outer_rear_rib_length   = outer_segment_chord * (abs(outer_rear_rib_yu) + abs(outer_rear_rib_yl))
    outer_wingbox_length    = outer_segment_chord * (fuel_tank.segments_percent_chord_end[1] -fuel_tank.segments_percent_chord_start[1]) 

    # inner segment coordinate  
    inner_segment_thickness =  np.minimum(inner_front_rib_length,inner_rear_rib_length)
    z_inner_upper  = inner_segment.origin[0][2] + (inner_segment_thickness / 2) - clearance
    z_inner_lower  = inner_segment.origin[0][2] - (inner_segment_thickness / 2) + clearance
    y_inner_upper  = inner_segment.percent_span_location * wing.spans.projected
    y_inner_lower  = y_inner_upper

    # outer segment coordinates  
    outer_segment_thickness =  np.minimum(outer_front_rib_length,outer_rear_rib_length)
    z_outer_upper  = inner_segment.origin[0][2] + delta_span *np.tan(inner_segment.dihedral_outboard) + outer_segment_thickness / 2 - clearance
    z_outer_lower  = inner_segment.origin[0][2] + delta_span *np.tan(inner_segment.dihedral_outboard) - outer_segment_thickness / 2 + clearance
    y_outer_upper  = outer_segment.percent_span_location * wing.spans.projected
    y_outer_lower  = y_outer_upper  

    dz_upper   = z_outer_upper - z_inner_upper
    dy_upper   = y_outer_upper - y_inner_upper
    dz_lower   = z_outer_lower - z_inner_lower
    dy_lower   = y_outer_lower - y_inner_lower

    # determine slopes 
    upper_slope = np.arctan(dz_upper/dy_upper) # might need to make negative
    lower_slope = np.arctan(dz_lower/dy_lower) 

    # determine tank diameter and location of next spar 
    D         = 0.1
    epsilon_D = 10  

    while abs(epsilon_D) > 0.001:
        AD =  D / np.cos(upper_slope)
        BC =  D / np.cos(lower_slope)

        # get equation of upper line
        f_upper = interp1d(np.array([y_inner_upper ,y_outer_upper  ]), np.array([z_inner_upper ,z_outer_upper  ]))

        # get equation of lower line
        f_lower = interp1d(np.array([y_inner_lower ,y_outer_lower ]), np.array([z_inner_lower ,z_outer_lower ]))

        AB = f_upper(y_inner_upper)  - f_lower(y_inner_upper)
        DC = f_upper(y_inner_upper+D)  - f_lower(y_inner_upper+D)

        epsilon_D  =  AB + DC - AD - BC

        delta_AD   = epsilon_D / (1 +  np.cos(upper_slope) /np.cos(lower_slope) )

        detla_D    = delta_AD *  np.cos(upper_slope)

        D += detla_D 

    # update tank percent span location
    tank_percent_span_location = inner_segment.percent_span_location +  D / semi_span 

    # get orgin of fuel tank 
    origin_x              = inner_segment.origin[0][0] + (fuel_tank.segments_percent_chord_start[0] * inner_segment_chord) + (np.tan( np.pi/2 - spar_sweep) * D / 2) -D/2
    origin_y              = inner_segment.origin[0][1] + D / 2
    origin_z              = inner_segment.origin[0][2] + (D / 2) *np.tan(inner_segment.dihedral_outboard)
    fuel_tank.origin      = [[origin_x,origin_y,origin_z]]
    fuel_tank.fuel.origin = [[origin_x,origin_y,origin_z]] 
    fuel_tank.fuel.xz_plane_symmetric = wing.xz_plane_symmetric
    fuel_tank.fuel.xy_plane_symmetric = wing.xy_plane_symmetric
    fuel_tank.fuel.yz_plane_symmetric = wing.yz_plane_symmetric 

    # get length of tank 
    m_2 =  (outer_wingbox_length -  inner_wingbox_length) / (outer_segment.percent_span_location - inner_segment_0.percent_span_location)  
    l_1 =  inner_wingbox_length +  m_2 * (tank_percent_span_location - inner_segment_0.percent_span_location)
    l_2 =  inner_wingbox_length -  (D / np.tan( np.pi/2 -spar_sweep)) 
    l   =  np.minimum(l_1, l_2) + D

    # internal radius of tank 
    r_out  = (D) / 2
    r_in   = (D -  2 * fuel_tank.wall_thickness ) / 2

    fuel_tank.diameters.external = D
    fuel_tank.diameters.internal = 2*r_in
    fuel_tank.lengths.external   = l-D
    fuel_tank.aspect_ratio       = (fuel_tank.lengths.external+fuel_tank.diameters.external)/fuel_tank.diameters.external

    l_in = fuel_tank.aspect_ratio * fuel_tank.diameters.internal

    fuel_tank.lengths.internal = l_in - fuel_tank.diameters.internal
    
    tank_volume_i = np.pi * ( r_in** 2) * (fuel_tank.lengths.internal )  +  4 / 3 * np.pi * ( r_in** 3) 
    tank_volume_o = np.pi * ( r_out** 2) * (fuel_tank.lengths.external)  +  4 / 3 * np.pi * ( r_out** 3) 

    if fuel_tank.xz_plane_symmetric:
        tank_volume_o *= 2
        tank_volume_i *= 2 

    return tank_percent_span_location, tank_volume_o, tank_volume_i

def compute_non_dimensional_rib_coordinates(compoment,fuel_tank,front_rib_nondim_x,rear_rib_nondim_x): 
    """
    Computes non-dimensional rib coordinates for wing segments based on airfoil geometry.

    This function extracts the upper and lower surface coordinates at the front and rear
    rib locations of a wing segment, accounting for structural clearances and airfoil
    geometry variations.

    Parameters
    ----------
    compoment : Wing_Segment
        Wing segment object containing airfoil and fuel tank information
            - airfoil : Airfoil
                Airfoil object containing geometry data
            - fuel_tank : Fuel_Tank_Segment
                Fuel tank segment properties
                    - percent_chord_start_location : float
                        Front rib location as fraction of chord
                    - percent_chord_end_location : float
                        Rear rib location as fraction of chord

    Returns
    -------
    front_rib_nondim_y_upper : float
        Non-dimensional upper surface coordinate at front rib
    rear_rib_nondim_y_upper : float
        Non-dimensional upper surface coordinate at rear rib
    front_rib_nondim_y_lower : float
        Non-dimensional lower surface coordinate at front rib
    rear_rib_nondim_y_lower : float
        Non-dimensional lower surface coordinate at rear rib

    Notes
    -----
    The function handles both NACA 4-series airfoils and custom airfoil coordinate files.
    A structural clearance is applied to ensure the tank fits within the wing structure.

    **Major Assumptions**
        * Airfoil geometry is properly defined
        * Fuel tank chord locations are within valid range
        * Structural clearance is appropriate for the application

    **Definitions**

    'Non-dimensional Coordinates'
        Airfoil coordinates normalized by chord length, typically ranging from 0 to 1

    'Rib Coordinates'
        Airfoil surface coordinates at specific chordwise locations where wing ribs
        or structural elements are positioned
    """

    if compoment.airfoil != None: 
        if type(compoment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
            geometry = compute_naca_4series(compoment.airfoil.NACA_4_Series_code)
        elif type(compoment.airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil: 
            geometry = import_airfoil_geometry(compoment.airfoil.coordinate_file)
    else:
        geometry = compute_naca_4series('0012')

    clearance = 1.5E-2 
    f_upper = interp1d(geometry.x_upper_surface  ,geometry.y_upper_surface, kind='linear')
    f_lower = interp1d(geometry.x_lower_surface  , geometry.y_lower_surface, kind='linear')

    # non-wing box dimension coordinates 
    front_rib_nondim_y_upper = f_upper([front_rib_nondim_x])[0] - clearance
    rear_rib_nondim_y_upper  = f_upper([rear_rib_nondim_x])[0]  - clearance   
    front_rib_nondim_y_lower = f_lower([front_rib_nondim_x])[0] + clearance   
    rear_rib_nondim_y_lower  = f_lower([rear_rib_nondim_x])[0]  + clearance   

    return front_rib_nondim_y_upper,rear_rib_nondim_y_upper, front_rib_nondim_y_lower, rear_rib_nondim_y_lower 

def compute_largest_circle(x_points, z_upper, z_lower):
    """
    Computes the largest circle that can fit within a polygon defined by airfoil coordinates.

    This function finds the optimal center point and radius for the largest possible circle
    that fits within the polygon formed by the upper and lower airfoil surfaces. It uses
    a grid search approach to find the best center location.

    Parameters
    ----------
    x_points : array_like
        X-coordinates of the airfoil points
    z_upper : array_like
        Z-coordinates of the upper airfoil surface
    z_lower : array_like
        Z-coordinates of the lower airfoil surface

    Returns
    -------
    max_diameter : float
        Diameter of the largest possible circle
    x_center : float
        X-coordinate of the circle center
    z_center : float
        Z-coordinate of the circle center

    Notes
    -----
    The function creates a polygon from the airfoil coordinates and performs a grid search
    within the polygon's bounding box to find the optimal circle center. The radius is
    limited by the distance to the closest polygon edge.

    **Major Assumptions**
        * Airfoil coordinates form a valid polygon
        * Grid resolution is sufficient for accurate results
        * Polygon is simply connected

    **Theory**

    The largest circle is found by maximizing the radius r such that:
    
    .. math::
        r = \\min_{i} d(p, e_i)

    where p is the circle center and e_i are the polygon edges.

    **Definitions**

    'Inscribed Circle'
        The largest circle that can fit completely within a given polygon
    """

    coords = list(zip(x_points, z_upper)) + list(zip(x_points[::-1], z_lower[::-1]))
    poly   = Polygon(coords)

    #scan a fine grid inside the polygon's bounding box to find the best center
    minx, minz, maxx, maxz = poly.bounds
    nx, nz = 200, 200  
    xs = np.linspace(minx, maxx, nx)
    zs = np.linspace(minz, maxz, nz)

    best_r = 0.0
    best_pt = None

    for x in xs:
        for z in zs:
            p = Point(x, z)
            if not poly.contains(p):
                continue
            # the radius is limited by the closest polygon edge
            r = p.distance(poly.exterior)
            if r > best_r:
                best_r = r
                best_pt = (x, z)
    max_diameter = 2 * best_r
    
    return max_diameter , best_pt[0],best_pt[1]
