# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/Non_Integral_Tank/compute_non_integral_tank_volume.py
# 
# 
# Created: Aug 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# RCAIDE imports
import  RCAIDE
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry,  compute_naca_4series
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_non_dimensional_moment_of_inertia import compute_rounded_end_cylinder_non_dimensional_moi

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
    tank_percent_span_location = 0
    if fuel_tank.packs_after_tank is not None:
        tank_percent_span_location = fuel_tanks[fuel_tank.packs_after_tank].tank_percent_span_location
    seg_tags = fuel_tank.segments_bounding_tank
    for i in range(len(seg_tags)-1):
        inner_segment = wing.segments[seg_tags[i]]
        outer_segment = wing.segments[seg_tags[i+1]]
        try:
            tank_percent_span_location, tank_volume_o, tank_volume_i\
                                    = compute_wing_non_integral_tank_fuel_volume(fuel_tank,wing,inner_segment,outer_segment,tank_percent_span_location)
        except:
            print(f"[WARNING] Tank '{fuel_tank.tag}' does not fit in the segment. Removing from list.")
            fuel_tanks.pop(fuel_tank.tag)
            return

    fuel_tank.tank_percent_span_location              = tank_percent_span_location
    fuel_tank.volume_properties.net_volume            = tank_volume_i
    fuel_tank.volume_properties.gross_volume          = tank_volume_o
    fuel_tank.fuel.mass_properties.center_of_gravity  =  [[fuel_tank.lengths.external / 2, 0, 0]]
    fuel_tank.mass_properties.center_of_gravity       =  [[fuel_tank.lengths.external / 2, 0, 0]]

    fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor = compute_rounded_end_cylinder_non_dimensional_moi(fuel_tank.diameters.internal / 2, fuel_tank.lengths.internal - fuel_tank.diameters.internal)

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
    if tank_percent_span_location > inner_segment_0.percent_span_location: 
        inner_segment.percent_span_location = tank_percent_span_location
        m                                   =  (outer_segment.root_chord_percent -  inner_segment_0.root_chord_percent) / (outer_segment.percent_span_location - inner_segment_0.percent_span_location)
        delta_y_percent                     =  (tank_percent_span_location - inner_segment_0.percent_span_location)
        inner_segment.root_chord_percent    = inner_segment_0.root_chord_percent + m*delta_y_percent

        # update segment origin
        delta_y                    = delta_y_percent * semi_span
        inner_segment.origin[0][0] = inner_segment_0.origin[0][0] + delta_y * np.tan(inner_segment_0.sweeps.leading_edge) 
        inner_segment.origin[0][1] = inner_segment.percent_span_location * semi_span
        inner_segment.origin[0][2] = inner_segment_0.origin[0][2] +  delta_y *np.tan(inner_segment.dihedral_outboard)

    inner_ribs = compute_non_dimensional_rib_coordinates(inner_segment_0,fuel_tank,fuel_tank.segments_percent_chord_start[0], fuel_tank.segments_percent_chord_end[0])
    if tank_percent_span_location > inner_segment_0.percent_span_location:
        # a packed tank's inner station lies between the two sections: blend their rib coordinates linearly in span,
        # the same linear airfoil transition as generate_interpolated_airfoils (and as the wing surface is lofted)
        outer_ribs = compute_non_dimensional_rib_coordinates(outer_segment,fuel_tank,fuel_tank.segments_percent_chord_start[0], fuel_tank.segments_percent_chord_end[0])
        w          = (tank_percent_span_location - inner_segment_0.percent_span_location) / (outer_segment.percent_span_location - inner_segment_0.percent_span_location)
        inner_ribs = [(1 - w) * a + w * b for a, b in zip(inner_ribs, outer_ribs)]
    inner_front_rib_yu,inner_rear_rib_yu,inner_front_rib_yl,inner_rear_rib_yl = inner_ribs
    inner_segment_chord     = wing.chords.root * inner_segment.root_chord_percent

    clearance  = fuel_tank.wall_clearance
    delta_span = (outer_segment.percent_span_location - inner_segment.percent_span_location) * semi_span

    outer_front_rib_yu,outer_rear_rib_yu,outer_front_rib_yl,outer_rear_rib_yl = compute_non_dimensional_rib_coordinates(outer_segment,fuel_tank,fuel_tank.segments_percent_chord_start[1], fuel_tank.segments_percent_chord_end[1])
    outer_segment_chord     = wing.chords.root * outer_segment.root_chord_percent

    # inner segment coordinate  
    # upper/lower bounds from the actual (cambered) surfaces at the spars: lower of the upper surfaces, higher of the lower
    z_inner_upper  = inner_segment.origin[0][2] + inner_segment_chord * np.minimum(inner_front_rib_yu, inner_rear_rib_yu) - clearance
    z_inner_lower  = inner_segment.origin[0][2] + inner_segment_chord * np.maximum(inner_front_rib_yl, inner_rear_rib_yl) + clearance
    y_inner_upper  = inner_segment.percent_span_location * semi_span
    y_inner_lower  = y_inner_upper

    # outer segment coordinates  
    z_outer_chord  = inner_segment.origin[0][2] + delta_span *np.tan(inner_segment.dihedral_outboard)
    z_outer_upper  = z_outer_chord + outer_segment_chord * np.minimum(outer_front_rib_yu, outer_rear_rib_yu) - clearance
    z_outer_lower  = z_outer_chord + outer_segment_chord * np.maximum(outer_front_rib_yl, outer_rear_rib_yl) + clearance
    y_outer_upper  = outer_segment.percent_span_location * semi_span
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
        f_upper = interp1d(np.array([y_inner_upper ,y_outer_upper  ]), np.array([z_inner_upper ,z_outer_upper  ]), fill_value='extrapolate')

        # get equation of lower line
        f_lower = interp1d(np.array([y_inner_lower ,y_outer_lower ]), np.array([z_inner_lower ,z_outer_lower ]), fill_value='extrapolate')

        AB = f_upper(y_inner_upper)  - f_lower(y_inner_upper)
        DC = f_upper(y_inner_upper+D)  - f_lower(y_inner_upper+D)

        epsilon_D  =  AB + DC - AD - BC

        delta_AD   = epsilon_D / (1 +  np.cos(upper_slope) /np.cos(lower_slope) )

        detla_D    = delta_AD *  np.cos(upper_slope)

        D += detla_D 

    # a tank cannot be wider than the span left in the segment
    D = np.minimum(D, y_outer_upper - y_inner_upper)
    if D <= 0:
        raise ValueError(f"No span left in the segment for tank '{fuel_tank.tag}'")

    # update tank percent span location
    tank_percent_span_location = inner_segment.percent_span_location +  D / semi_span 

    # front and rear spar lines, straight between the tank's inner station and the outer segment
    y_inner   = inner_segment.percent_span_location * semi_span
    y_outer   = outer_segment.percent_span_location * semi_span
    x_le_o    = inner_segment_0.origin[0][0] + (y_outer - inner_segment_0.percent_span_location * semi_span) * np.tan(inner_segment_0.sweeps.leading_edge)
    x_front_i = inner_segment.origin[0][0] + fuel_tank.segments_percent_chord_start[0] * inner_segment_chord
    x_front_o = x_le_o + fuel_tank.segments_percent_chord_start[1] * outer_segment_chord
    x_rear_i  = inner_segment.origin[0][0] + fuel_tank.segments_percent_chord_end[0] * inner_segment_chord
    x_rear_o  = x_le_o + fuel_tank.segments_percent_chord_end[1] * outer_segment_chord
    tan_front = (x_front_o - x_front_i) / (y_outer - y_inner)
    tan_rear  = (x_rear_o - x_rear_i) / (y_outer - y_inner)

    # the tank spans y_inner to y_inner + D; its hemispherical caps (centered at y_inner + R) clear each spar line
    # by placing the cap centers R / cos(sweep) from it, so the tip-to-tip length stays within the wing box
    R        = D / 2
    origin_x = x_front_i + R * (tan_front + np.sqrt(1 + tan_front**2) - 1)
    l        = x_rear_i + R * (tan_rear - np.sqrt(1 + tan_rear**2) + 1) - origin_x
    if l < D:
        raise ValueError(f"Wing box too short for tank '{fuel_tank.tag}'")

    # get orgin of fuel tank 
    origin_y              = inner_segment.origin[0][1] + D / 2
    origin_z              = (f_upper(y_inner_upper + D / 2) + f_lower(y_inner_upper + D / 2)) / 2   # centred between the bounds
    fuel_tank.origin      = [[origin_x,origin_y,origin_z]]
    fuel_tank.fuel.origin = [[origin_x,origin_y,origin_z]] 
    fuel_tank.fuel.xz_plane_symmetric = wing.xz_plane_symmetric
    fuel_tank.fuel.xy_plane_symmetric = wing.xy_plane_symmetric
    fuel_tank.fuel.yz_plane_symmetric = wing.yz_plane_symmetric 

    # internal radius of tank 
    r_out  = (D) / 2
    r_in   = (D -  2 * fuel_tank.wall_thickness ) / 2

    fuel_tank.diameters.external = D
    fuel_tank.diameters.internal = 2 * r_in
    fuel_tank.lengths.external   = l
    fuel_tank.aspect_ratio       = fuel_tank.lengths.external / fuel_tank.diameters.external

    fuel_tank.lengths.internal   = fuel_tank.aspect_ratio * fuel_tank.diameters.internal

    # volume = cylinder + sphere; cylinder length = total length - diameter
    L_cyl_o = fuel_tank.lengths.external  - fuel_tank.diameters.external
    L_cyl_i = fuel_tank.lengths.internal  - fuel_tank.diameters.internal
    tank_volume_o = np.pi * r_out**2 * L_cyl_o + 4 / 3 * np.pi * r_out**3
    tank_volume_i = np.pi * r_in**2  * L_cyl_i + 4 / 3 * np.pi * r_in**3

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
            geometry = compute_naca_4series(compoment.airfoil.NACA_4_Series_code, thickness_multiplier = compoment.airfoil.thickness_multiplier)
        elif type(compoment.airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil:
            geometry = import_airfoil_geometry(compoment.airfoil.coordinate_file, thickness_multiplier = compoment.airfoil.thickness_multiplier)
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