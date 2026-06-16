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
            
    # Non-dimensional (I/mass) MOI computed from geometry alone; mass cancels out of the
    # underlying formulas, so this remains valid as the (changing) fuel mass burns down.
    I_local_tank = _non_dimensional_rounded_cylinder_moi(fuel_tank.lengths.external, fuel_tank.diameters.external / 2,
                                                          fuel_tank.lengths.internal, fuel_tank.diameters.internal / 2,
                                                          bwb_aft_tank=False)
    I_local_fuel = _non_dimensional_rounded_cylinder_moi(fuel_tank.lengths.internal, fuel_tank.diameters.internal / 2,
                                                          bwb_aft_tank=True)

    cg = [[(fuel_tank.lengths.external + fuel_tank.diameters.external) / 2, 0, 0]]

    # pack properties into fuel tank object
    fuel_tank.mass_properties.center_of_gravity                              = cg
    fuel_tank.fuel.mass_properties.center_of_gravity                         = cg
    fuel_tank.mass_properties.moments_of_inertia.non_dimensional_tensor      = I_local_tank
    fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor = I_local_fuel
    fuel_tank.volume_properties.net_volume                                   = tank_volume_i
    fuel_tank.volume_properties.gross_volume                                 = tank_volume_o

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
