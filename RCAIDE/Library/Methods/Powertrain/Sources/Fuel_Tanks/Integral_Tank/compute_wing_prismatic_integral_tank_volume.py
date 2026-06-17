
# RCAIDE/Methods/Powertrain/Sources/Fuel_Tanks/Integral_Tank/compute_wing_prismatic_integral_tank_volume.py
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
def compute_wing_prismatic_integral_tank_volume(fuel_tank,wing,fuel_tanks): 

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

    See Also
    --------
    compute_wing_integral_tank_fuel_volume : Calculates volume for single-segment wings
    compute_segmented_wing_integral_tank_fuel_volume : Calculates volume for wing segments
    """ 
    total_fuel_mass    = 0
    total_fuel_volume  = 0

    
    fuel_tank.fuel.xz_plane_symmetric = wing.xz_plane_symmetric
    fuel_tank.fuel.xy_plane_symmetric = wing.xy_plane_symmetric
    fuel_tank.fuel.yz_plane_symmetric = wing.yz_plane_symmetric 
    
    segment_tank_moment = np.array([0.0, 0.0, 0.0])
    seg_bounds =  fuel_tank.segments_bounding_tank  

    # Collect all segment tags between start and end (inclusive)
    collect = False
    seg_tags = []
    for segment in wing.segments:
        if segment.tag == seg_bounds[0]:
            collect = True
        if collect:
            seg_tags.append(segment.tag)
        if segment.tag == seg_bounds[1]:
            break
    
    for i in range(len(seg_tags)-1):
        inner_segment = wing.segments[seg_tags[i]]
        outer_segment = wing.segments[seg_tags[i+1]]

        # compute volume of fuel in wing
        volume = compute_segmented_wing_integral_tank_fuel_volume(wing,inner_segment,outer_segment,fuel_tank)
        inner_segment.volume_properties.fuel = volume

        total_fuel_mass      += volume * fuel_tank.fuel.density
        segment_tank_moment  += np.array(inner_segment.mass_properties.center_of_gravity)[0] * volume * fuel_tank.fuel.density  
        total_fuel_volume    += volume
    
    inner_segment_x_start = wing.segments[seg_tags[0]].origin[0][0] + wing.segments[seg_tags[0]].root_chord_percent * wing.chords.root * (fuel_tank.segments_percent_chord_start[0])
    inner_segment_x_end   = wing.segments[seg_tags[0]].origin[0][0] + wing.segments[seg_tags[0]].root_chord_percent * wing.chords.root * (fuel_tank.segments_percent_chord_end[0])
    outer_segment_x_start = wing.segments[seg_tags[-1]].origin[0][0] + wing.segments[seg_tags[-1]].root_chord_percent * wing.chords.root * (fuel_tank.segments_percent_chord_start[1])
    outer_segment_x_end   = wing.segments[seg_tags[-1]].origin[0][0] + wing.segments[seg_tags[-1]].root_chord_percent * wing.chords.root * (fuel_tank.segments_percent_chord_end[0])  
    inner_segment_y       =  wing.segments[seg_tags[0]].origin[0][1]
    outer_segment_y       =  wing.segments[seg_tags[-1]].origin[0][1]

    inner_front_rib_yu,inner_rear_rib_yu,inner_front_rib_yl,inner_rear_rib_yl = compute_non_dimensional_rib_coordinates(wing.segments[seg_tags[0]],fuel_tank, fuel_tank.segments_percent_chord_start[0],fuel_tank.segments_percent_chord_end[0])
    inner_segment_chord     = wing.chords.root * inner_segment.root_chord_percent
    inner_front_rib_length  = inner_segment_chord * (abs(inner_front_rib_yu) + abs(inner_front_rib_yl))
    inner_rear_rib_length   = inner_segment_chord * (abs(inner_rear_rib_yu) + abs(inner_rear_rib_yl) )

    outer_front_rib_yu,outer_rear_rib_yu,outer_front_rib_yl,outer_rear_rib_yl = compute_non_dimensional_rib_coordinates(wing.segments[seg_tags[-1]],fuel_tank, fuel_tank.segments_percent_chord_start[1],fuel_tank.segments_percent_chord_end[1])
    outer_segment_chord     = wing.chords.root * outer_segment.root_chord_percent
    outer_front_rib_length  = outer_segment_chord * (abs(outer_front_rib_yu) + abs(outer_front_rib_yl) )
    outer_rear_rib_length   = outer_segment_chord * (abs(outer_rear_rib_yu) + abs(outer_rear_rib_yl) )

    p1x = inner_segment_x_start
    p1y = inner_segment_y
    p2x = inner_segment_x_end
    p2y = inner_segment_y
    p3x = outer_segment_x_end
    p3y = outer_segment_y
    p4x = outer_segment_x_start
    p4y = outer_segment_y

    A_num = (p1x*p2y - p2x*p1y + p2x*p3y - p3x*p2y + p3x*p4y - p4x*p3y + p4x*p1y - p1x*p4y)
    A     = 0.5 * A_num
    x_cg  = (1/(6*A)) * ((p1x + p2x)*(p1x*p2y - p2x*p1y) + (p2x + p3x)*(p2x*p3y - p3x*p2y) +
            (p3x + p4x)*(p3x*p4y - p4x*p3y) +  (p4x + p1x)*(p4x*p1y - p1x*p4y))
    y_cg = (1/(6*A)) * (
    (p1y + p2y)*(p1x*p2y - p2x*p1y) +
    (p2y + p3y)*(p2x*p3y - p3x*p2y) +
    (p3y + p4y)*(p3x*p4y - p4x*p3y) +
    (p4y + p1y)*(p4x*p1y - p1x*p4y)
)
    
    # Create an equivalent rectangle to proceed with liquid hydrogen tank buildup
    fuel_tank.average_outer_width   = ((p2x-p1x)+ (p3x-p4x))/2
    fuel_tank.average_outer_length  = outer_segment_y - inner_segment_y
    fuel_tank.average_outer_height  = (inner_front_rib_length + inner_rear_rib_length + outer_front_rib_length +outer_rear_rib_length)/4
        
    fuel_tank.aspect_ratio = fuel_tank.average_outer_length / fuel_tank.average_outer_height
    
    fuel_tank.fuel.mass_properties.center_of_gravity  = [[x_cg, y_cg, 0]]
    fuel_tank.mass_properties.center_of_gravity  = [[x_cg, y_cg, 0]]
    fuel_tank.volume_properties.net_volume            = total_fuel_volume
    fuel_tank.volume_properties.gross_volume          = total_fuel_volume
        

    if fuel_tank.fuel.mass_properties.mass != 0:
        actual_fuel_volume = fuel_tank.fuel.mass_properties.mass /  fuel_tank.fuel.density  
        if actual_fuel_volume > fuel_tank.volume_properties.net_volume + 1e-8:
            raise AttributeError('Specified fuel mass greater than mass of fuel capable of being stored in fuel tank') 
        fuel_tank.fuel.volume_properties.net_volume = actual_fuel_volume
    else:
        fuel_tank.fuel.mass_properties.mass         = total_fuel_volume *  fuel_tank.fuel.density   
        fuel_tank.fuel.volume_properties.net_volume = total_fuel_volume    
    return 

def compute_segmented_wing_integral_tank_fuel_volume(wing,inner_segment,outer_segment,fuel_tank):   
    """
    Computes the fuel volume for an integral fuel tank between two wing segments in a multi-segment wing.

    This function calculates the volume of fuel that can be stored in an integral tank
    located between two adjacent wing segments. It accounts for the varying chord
    dimensions and fuel tank specifications of each segment.

    Parameters
    ---------- 
    wing : Wing
        The wing object containing overall geometry and symmetry properties
    inner_segment : Wing_Segment
        The inner wing segment containing chord percentage and fuel tank specifications
    outer_segment : Wing_Segment
        The outer wing segment containing chord percentage and fuel tank specifications

    Returns
    -------
    volume : float
        The calculated fuel volume in cubic units

    Notes
    -----
    The function calculates wing box dimensions for both segments and uses the
    truncated prism formula. For symmetric wings, the volume is doubled to account
    for both left and right sides. Assumes a NACA 0012 airfoil if no airfoil is provided.

    **Major Assumptions**
        * Fuel tank spans exactly between the two specified segments
        * Linear variation of chord dimensions between segments
        * Wing box geometry follows the airfoil profile

    **Theory**

    The volume is calculated using a truncated prism formula:

    :math:`V = \\frac{1}{3} \\left( A_1 + A_2 + \\sqrt{A_1 A_2} \\right) h`

    where:
        - :math:`A_1` is the wing box area at the inner segment
        - :math:`A_2` is the wing box area at the outer segment
        - :math:`h` is the span distance between segments

    **Definitions**

    'Wing Segment'
        A portion of the wing with defined chord percentage and fuel tank specifications

    'Span Location'
        The position along the wing span where the segment is located
    """

    inner_front_rib_yu,inner_rear_rib_yu,inner_front_rib_yl,inner_rear_rib_yl = compute_non_dimensional_rib_coordinates(inner_segment,fuel_tank, fuel_tank.segments_percent_chord_start[0],fuel_tank.segments_percent_chord_end[0])
    inner_segment_chord     = wing.chords.root * inner_segment.root_chord_percent
    inner_front_rib_length  = inner_segment_chord * (abs(inner_front_rib_yu) + abs(inner_front_rib_yl))
    inner_rear_rib_length   = inner_segment_chord * (abs(inner_rear_rib_yu) + abs(inner_rear_rib_yl) )
    inner_wingbox_length    = inner_segment_chord * (fuel_tank.segments_percent_chord_end[0] -fuel_tank.segments_percent_chord_start[0])  

    outer_front_rib_yu,outer_rear_rib_yu,outer_front_rib_yl,outer_rear_rib_yl = compute_non_dimensional_rib_coordinates(outer_segment,fuel_tank, fuel_tank.segments_percent_chord_start[1],fuel_tank.segments_percent_chord_end[1])
    outer_segment_chord     = wing.chords.root * outer_segment.root_chord_percent
    outer_front_rib_length  = outer_segment_chord * (abs(outer_front_rib_yu) + abs(outer_front_rib_yl) )
    outer_rear_rib_length   = outer_segment_chord * (abs(outer_rear_rib_yu) + abs(outer_rear_rib_yl) )
    outer_wingbox_length    = outer_segment_chord * (fuel_tank.segments_percent_chord_end[1] -fuel_tank.segments_percent_chord_start[1])  

    # volume of truncated prism
    A_1 = inner_wingbox_length * (inner_front_rib_length + inner_rear_rib_length) / 2 
    A_2 = outer_wingbox_length * (outer_front_rib_length + outer_rear_rib_length) / 2
    h   =  (outer_segment.percent_span_location -  inner_segment.percent_span_location) *  wing.spans.projected /2    # assumes wing is symmetric
    volume = (1 /3) * ( A_1 + A_2 + np.sqrt(A_1*A_2)) *h

    if wing.xz_plane_symmetric:
        volume *= 2    
 
    return volume

def compute_non_dimensional_rib_coordinates(compoment,fuel_tank,front_rib_nondim_x,rear_rib_nondim_x): 
    """
    Computes the non-dimensional rib coordinates for fuel tank volume calculations.

    This function determines the upper and lower surface coordinates at the front
    and rear rib locations of a fuel tank, accounting for airfoil geometry and
    clearance requirements.

    Parameters
    ----------
    compoment : {Wing, Wing_Segment}
        The wing or wing segment component containing airfoil data and fuel tank specifications

    Returns
    -------
    front_rib_nondim_y_upper : float
        Non-dimensional y-coordinate of upper surface at front rib location
    rear_rib_nondim_y_upper : float
        Non-dimensional y-coordinate of upper surface at rear rib location
    front_rib_nondim_y_lower : float
        Non-dimensional y-coordinate of lower surface at front rib location
    rear_rib_nondim_y_lower : float
        Non-dimensional y-coordinate of lower surface at rear rib location

    Notes
    -----
    The function interpolates airfoil geometry to find the surface coordinates at
    the fuel tank's chord-wise start and end locations. A clearance is applied to
    ensure the fuel tank doesn't interfere with the airfoil surface. Defaults to using a
    NACA 0012 airfoil if no airfoil is provided.

    **Major Assumptions**
        * Airfoil geometry is available and properly defined
        * Fuel tank chord locations are within the airfoil bounds
        * Linear interpolation is sufficient for coordinate determination

    **Definitions**

    'Non-dimensional Coordinates'
        Coordinates normalized by the chord length (x/c, y/c)

    'Rib Location'
        The position along the chord where the fuel tank front and rear boundaries are located

    'Clearance'
        The minimum distance between the fuel tank boundary and the airfoil surface

    See Also
    --------
    RCAIDE.Library.Methods.Geometry.Airfoil.import_airfoil_geometry : For importing airfoil coordinate files
    RCAIDE.Library.Methods.Geometry.Airfoil.compute_naca_4series : For generating NACA 4-series airfoil geometry
    """
    if compoment.airfoil != None: 
        if type(compoment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
            geometry = compute_naca_4series(compoment.airfoil.NACA_4_Series_code)
        elif type(compoment.airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil: 
            geometry = import_airfoil_geometry(compoment.airfoil.coordinate_file)
    else:
        geometry = compute_naca_4series('0012')

    clearance = 1.5E-2 
    f_upper            = interp1d(geometry.x_upper_surface, geometry.y_upper_surface, kind='linear')
    f_lower            = interp1d(geometry.x_lower_surface, geometry.y_lower_surface, kind='linear')

    # non-wing box dimension coordinates 
    front_rib_nondim_y_upper = f_upper([front_rib_nondim_x])[0] - clearance
    rear_rib_nondim_y_upper  = f_upper([rear_rib_nondim_x])[0]  - clearance   
    front_rib_nondim_y_lower = f_lower([front_rib_nondim_x])[0] + clearance   
    rear_rib_nondim_y_lower  = f_lower([rear_rib_nondim_x])[0]  + clearance   

    return front_rib_nondim_y_upper,rear_rib_nondim_y_upper, front_rib_nondim_y_lower, rear_rib_nondim_y_lower 
