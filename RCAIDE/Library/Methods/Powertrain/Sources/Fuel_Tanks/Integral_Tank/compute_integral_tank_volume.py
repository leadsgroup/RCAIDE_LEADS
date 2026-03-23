
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
def compute_fuselage_integral_tank_fuel_volume(fuel_tank,fuselage):
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
            volume_o = (1 /3) * ( A_1_o + A_2_o + np.sqrt(A_2_o*A_2_o)) *h

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
    
        if fuel_tank.fuel.mass_properties.mass != 0:
            actual_fuel_volume = fuel_tank.fuel.mass_properties.mass /  fuel_tank.fuel.density  
            if actual_fuel_volume > fuel_tank.volume_properties.net_volume + 1e-8 :
                raise AttributeError('Specified fuel mass greater than mass of fuel capable of being stored in fuel tank') 
        else:
            fuel_tank.fuel.mass_properties.mass           = tank_volume_i *  fuel_tank.fuel.density    
            fuel_tank.fuel.volume_properties.gross_volume = tank_volume_i
            
    # update orign of tank 
    fuel_tank.origin      = [[origin_x, origin_y, origin_z]]             
    fuel_tank.fuel.origin = [[origin_x, origin_y, origin_z]]  

    # Compute the MOI
    # intialize matrices
    fuel           = fuel_tank.fuel
    tank_mass      = fuel_tank.mass_properties.mass
    fuel_mass      = fuel.mass_properties.mass
    I_local_fuel   = np.zeros((3, 3)) 
    I_global_fuel  = np.zeros((3, 3)) 
    I_local_tank   = np.zeros((3, 3))    
    
    # Collect all segment tags between start and end (inclusive)
    if len(fuselage.segments) > 1:
        collect = False
        seg_tags = []
        seg_bounds =  fuel_tank.segments_bounding_tank

        total_wing_volume = 0
        for segment in fuselage.segments:
            if segment.tag == seg_bounds[0]:
                collect = True
            if collect:
                seg_tags.append(segment.tag) 
                total_wing_volume += segment.volume_properties.gross_volume                
            if segment.tag == seg_bounds[1]:
                break
            
        tessellation = 20
        num_fus_segs = len(seg_tags)
        fuselage_points = np.zeros((num_fus_segs*tessellation ,3))
          
        for i_seg, segment in enumerate(seg_tags):
            segment   = fuselage.segments[segment]
            segment_0 = fuselage.segments[seg_tags[0]]
            a         = segment.width/2
            b         = segment.height/2
            n         = segment.curvature
            theta     = np.linspace(0,2*np.pi,tessellation) 
            fus_ypts  =  (abs((np.cos(theta)))**(2/n))*a * ((np.cos(theta)>0)*1 - (np.cos(theta)<0)*1) 
            fus_zpts  =  (abs((np.sin(theta)))**(2/n))*b * ((np.sin(theta)>0)*1 - (np.sin(theta)<0)*1)
            
            
            start_idx  = i_seg * tessellation
            end_idx    = (i_seg + 1 )* tessellation
            fuselage_points[start_idx:end_idx,0] = (segment.percent_x_location - segment_0.percent_x_location) *fuselage.lengths.total  
            fuselage_points[start_idx:end_idx,1] = fus_ypts + segment.percent_y_location*fuselage.lengths.total + fuselage.origin[0][1]
            fuselage_points[start_idx:end_idx,2] = fus_zpts + segment.percent_z_location*fuselage.lengths.total + fuselage.origin[0][2]
   
        # Convex hull → watertight volume mesh
        solid_segment = trimesh.convex.convex_hull(fuselage_points) 
    
        # Rotate to match the RCAIDE aircraft axes convention
        R = trimesh.transformations.rotation_matrix(np.deg2rad(90), [1, 0, 0], [0, 0, 0])
        solid_segment.apply_transform(R)
    
        # Calculate MOI of the fuel within the fuel tank 
        solid_segment.density = fuel.density  
        I_local_fuel          = solid_segment.moment_inertia
        
    # Store moment of inertia tensors of tank and fuel 
    fuel_tank.fuel.mass_properties.moments_of_inertia.tensor                 = I_local_fuel
    fuel_tank.fuel.mass_properties.moments_of_inertia.non_dimensional_tensor = I_local_fuel / tank_mass
    fuel_tank.mass_properties.moments_of_inertia.tensor                      = I_local_tank 
    fuel_tank.mass_properties.moments_of_inertia.non_dimensional_tensor      = I_local_tank  / fuel_mass

    return 

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

    See Also
    --------
    compute_wing_integral_tank_fuel_volume : Calculates volume for single-segment wings
    compute_segmented_wing_integral_tank_fuel_volume : Calculates volume for wing segments
    """ 

    # get orgin of fuel tank     
    fuel_tank.origin                  = wing.origin 
    fuel_tank.fuel.origin             = wing.origin  
    fuel_tank.fuel.xz_plane_symmetric = wing.xz_plane_symmetric
    fuel_tank.fuel.xy_plane_symmetric = wing.xy_plane_symmetric
    fuel_tank.fuel.yz_plane_symmetric = wing.yz_plane_symmetric 
    
    if len(wing.segments) > 1: 

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

        # 1. copy the mesh
        combined_mesh_sym = deepcopy(combinde_mesh)

        # 2. apply the mirror transform
        combined_mesh_sym.vertices = (Ry @ combined_mesh_sym.vertices.T).T

        # 3. fix face orientation (reverse winding)
        combined_mesh_sym.faces = combined_mesh_sym.faces[:, ::-1]

        # 4. concatenate original + mirrored
        combined_mesh_full         = trimesh.util.concatenate([combinde_mesh, combined_mesh_sym]) 
        combined_mesh_full.density = fuel_tank.fuel.density 
        centroid = combined_mesh_full.centroid
        cg_x     = centroid[0]
        cg_y     = centroid[1]
        cg_z     = centroid[2] 
        
        # Shift inertia tensor from origin to the requested (actual) centroid
        I = combined_mesh_full.moment_inertia 
        total_fuel_volume   = combined_mesh_full.volume 
        
        fuel_tank.fuel.mass_properties.center_of_gravity          = [[cg_x, cg_y, cg_z]]
        fuel_tank.fuel.mass_properties.moments_of_inertia.tensor  = I

        fuel_tank.volume_properties.gross_volume          = total_fuel_volume
        fuel_tank.volume_properties.net_volume            = total_fuel_volume

         
        if fuel_tank.fuel.mass_properties.mass != 0:
            actual_fuel_volume = fuel_tank.fuel.mass_properties.mass /  fuel_tank.fuel.density  
            if actual_fuel_volume > fuel_tank.volume_properties.net_volume + 1e-8 :
                raise AttributeError('Specified fuel mass greater than mass of fuel capable of being stored in fuel tank') 
        else:
            fuel_tank.fuel.mass_properties.mass = total_fuel_volume *  fuel_tank.fuel.density   
            
    else:  
        # assume whole wing has fuel 
        total_fuel_volume                                 = compute_wing_integral_tank_fuel_volume(wing,fuel_tank) # We might want to reconsider doing this with trimesh or smth 
        total_fuel_mass                                   = total_fuel_volume  * fuel_tank.fuel.density 
        fuel_tank.volume_properties.internal_volume       = total_fuel_volume
        fuel_tank.volume_properties.external_volume       = total_fuel_volume 
        fuel_tank.volume_properties.net_volume            = total_fuel_volume

    if fuel_tank.fuel.mass_properties.mass != 0:
        actual_fuel_volume = fuel_tank.fuel.mass_properties.mass /  fuel_tank.fuel.density  
        if actual_fuel_volume > fuel_tank.volume_properties.net_volume + 1e-8:
            raise AttributeError('Specified fuel mass greater than mass of fuel capable of being stored in fuel tank') 
        fuel_tank.fuel.volume_properties.net_volume = actual_fuel_volume
    else:
        fuel_tank.fuel.mass_properties.mass         = total_fuel_volume *  fuel_tank.fuel.density   
        fuel_tank.fuel.volume_properties.net_volume = total_fuel_volume    
    return 

def compute_wing_integral_tank_fuel_volume(wing,fuel_tank):     
    """
    Computes the fuel volume for an integral fuel tank in a single-segment wing.

    This function calculates the volume of fuel that can be stored in an integral tank
    spanning the entire wing span. It uses the wing's root and tip chord dimensions
    along with the fuel tank's chord-wise location specifications.

    Parameters
    ---------- 
    wing : Wing
        The wing object containing chord dimensions, span, and fuel tank specifications

    Returns
    -------
    volume : float
        The calculated fuel volume in cubic meters

    Notes
    -----
    The function calculates the wing box dimensions at both root and tip locations
    and uses the truncated prism formula to determine the total volume. The wing box
    is defined by the fuel tank's chord-wise start and end locations. Assumes a NACA 0012
    airfoil if no airfoil is provided.

    **Major Assumptions**
        * Fuel tank spans the entire wing from root to tip
        * Wing box geometry follows the airfoil profile
        * Linear variation of chord dimensions from root to tip

    **Theory**

    The volume is calculated using a truncated prism formula:

    :math:`V = \\frac{1}{3} \\left( A_1 + A_2 + \\sqrt{A_1 A_2} \\right) h`

    where:
        - :math:`A_1` is the wing box area at the root
        - :math:`A_2` is the wing box area at the tip
        - :math:`h` is the wing span

    **Definitions**

    'Wing Box'
        The structural box formed by the front and rear spars, containing the fuel tank

    'Chord Location'
        The position along the wing chord where the fuel tank begins and ends
    """

    inner_front_rib_yu,inner_rear_rib_yu,inner_front_rib_yl,inner_rear_rib_yl = compute_non_dimensional_rib_coordinates(wing,fuel_tank,fuel_tank.segments_percent_chord_start[0], fuel_tank.segments_percent_chord_end[0])
    inner_front_rib_length  = wing.chords.root * (abs(inner_front_rib_yu) + abs(inner_front_rib_yl))
    inner_rear_rib_length   = wing.chords.root * (abs(inner_rear_rib_yu) + abs(inner_rear_rib_yl))
    inner_wingbox_length    = wing.chords.root * (fuel_tank.segments_percent_chord_end[0] -fuel_tank.segments_percent_chord_start[0]) 

    outer_front_rib_yu,outer_rear_rib_yu,outer_front_rib_yl,outer_rear_rib_yl = compute_non_dimensional_rib_coordinates(wing,fuel_tank,fuel_tank.segments_percent_chord_start[1], fuel_tank.segments_percent_chord_end[1])
    outer_front_rib_length  = wing.chords.tip * (abs(outer_front_rib_yu) + abs(outer_front_rib_yl))
    outer_rear_rib_length   = wing.chords.tip * (abs(outer_rear_rib_yu) + abs(outer_rear_rib_yl))
    outer_wingbox_length    = wing.chords.tip * (fuel_tank.segments_percent_chord_end[1] -fuel_tank.segments_percent_chord_start[1]) 

    # volume of truncated prism
    A_1 = inner_wingbox_length * (inner_front_rib_length + inner_rear_rib_length) / 2 
    A_2 = outer_wingbox_length * (outer_front_rib_length + outer_rear_rib_length) / 2
    h =  wing.spans.projected
    volume  = (1 /3) * ( A_1 + A_2 + np.sqrt(A_1*A_2)) *h   

    return volume


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

def compute_wing_integral_prismatic_tank_volume(fuel_tank,wing,fuel_tanks):


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
    
    if len(wing.segments) > 1: 
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
            if actual_fuel_volume > fuel_tank.volume_properties.net_volume + 1e-8 :
                raise AttributeError('Specified fuel mass greater than mass of fuel capable of being stored in fuel tank') 
        else:
            fuel_tank.fuel.mass_properties.mass = total_fuel_volume *  fuel_tank.fuel.density   
            
    else:  
       raise AttributeError('Need more than one segment in the wing to calculate segmented tank properties.')

    if fuel_tank.fuel.mass_properties.mass != 0:
        actual_fuel_volume = fuel_tank.fuel.mass_properties.mass /  fuel_tank.fuel.density  
        if actual_fuel_volume > fuel_tank.volume_properties.net_volume + 1e-8:
            raise AttributeError('Specified fuel mass greater than mass of fuel capable of being stored in fuel tank') 
        fuel_tank.fuel.volume_properties.net_volume = actual_fuel_volume
    else:
        fuel_tank.fuel.mass_properties.mass         = total_fuel_volume *  fuel_tank.fuel.density   
        fuel_tank.fuel.volume_properties.net_volume = total_fuel_volume    
    return 

def compute_bwb_aft_integral_prismatic_tank_volume(fuel_tank, wing,fuel_tanks):

    # Check if there are enough properties to accurately compute the maximum possible tank volume 
    if any(val is None for val in [
        fuel_tank.aft_tank_root_chord_bounds[0],
        fuel_tank.aft_tank_root_chord_bounds[1],
        fuel_tank.segments_bounding_tank ,
        fuel_tank.aft_tank_segment_bound
        ]):
        raise ValueError("One or more required aft tank parameters are not set in 'fuel_tank'.")

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
    n = 2 # 2 gives us a straight tank
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

        # ax.plot(x_coords, y_coords, color='tab:blue', linewidth=1.0, alpha=0.85)

    fuel_tank.aft_tank_root_chord_bounds[1] = tank_end_percent_current
    # ------------------------------------------------------
    # Compute prismatic volumes from intersections
    # ------------------------------------------------------
    tank_volumes = np.zeros(num_tank_sections - 1)
    tank_lengths = np.zeros(num_tank_sections - 1)
    intersection_polygons = []

    for seg_i in range(1, num_tank_sections):

        if seg_i == 1:
            inner_polygon = Polygon(polygon_points[seg_i - 1])
        else:
            inner_polygon = intersection_polygon

        outer_polygon = Polygon(polygon_points[seg_i])

        intersection_polygon = inner_polygon.intersection(outer_polygon)

        if intersection_polygon.is_empty:
            tank_volumes[seg_i - 1] = 0.0
            tank_lengths[seg_i - 1] = 0.0
            intersection_polygons.append(None)
            continue

        if intersection_polygon.geom_type == 'MultiPolygon':
            intersection_polygon = max(
                intersection_polygon.geoms,
                key=lambda g: g.area
            )

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
    fuel_tank.max_volume_intersection_num_edges = int(len(edge_lengths))

    # # Print results
    # for i, L in enumerate(edge_lengths):
    #     print(f"Edge {i+1} length: {L:.6f} m")


    # if best_polygon.geom_type == 'Polygon':
    #     x, y = best_polygon.exterior.xy
    #     ax.fill(x, y, color='tab:orange', alpha=0.4)

    # elif best_polygon.geom_type == 'MultiPolygon':
    #     for poly in best_polygon.geoms:
    #         x, y = poly.exterior.xy
    #         ax.fill(x, y, color='tab:orange', alpha=0.4)

    # ax.set_aspect('equal', 'box')
    # ax.set_xlabel("x (m)")
    # ax.set_ylabel("z (m)")
    # ax.set_title("Tank Section Polygons and Max Volume Intersection")
    # plt.show()

    fuel_tank.average_outer_width   = (edge_lengths[0]+edge_lengths[2])/2
    fuel_tank.average_outer_length  = fuel_tank.length_external
    fuel_tank.average_outer_height  = (edge_lengths[1]+edge_lengths[3])/2
    
    fuel_tank.aspect_ratio  = fuel_tank.average_outer_length /fuel_tank.average_outer_height
    
    # Outer Volume
    tank_volume_o                = max_volume
    # Inner Volume
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
   
    # Build a 3D tank mesh by extruding the 2D section over length_external.
    # Extrusion is centered about y = 0 (symmetric about origin in spanwise axis).
    tank_mesh = trimesh.creation.extrude_polygon(
        polygon=polygon_for_calc,
        height=float(fuel_tank.length_external)
    )
    R = trimesh.transformations.rotation_matrix(-np.pi / 2.0, [1.0, 0.0, 0.0])
    T = trimesh.transformations.translation_matrix(
        [0.0, -0.5 * float(fuel_tank.length_external), 0.0]
    )
  
    tank_mesh.apply_transform(R)
    tank_mesh.apply_transform(T)

    centroid = np.asarray(tank_mesh.center_mass, dtype=float)
    I = tank_mesh.moment_inertia 
    cg_x = centroid[0]
    cg_y = 0
    cg_z = centroid[2]
    
    fuel_tank.mass_properties.center_of_gravity = [[cg_x, cg_y, cg_z]]
    fuel_tank.fuel.mass_properties.center_of_gravity = [[cg_x, cg_y, cg_z]]

    fuel_tank.fuel.mass_properties.moments_of_inertia.tensor  = I

    fuel_tank.origin     =  [[0, 0, 0]]
    fuel_tank.fuel.origin   = fuel_tank.origin

    # tank_scene = trimesh.Scene()
    # tank_scene.add_geometry(tank_mesh, geom_name="aft_integral_tank")
    # fuel_tank.trimesh_scene = tank_scene
    # tank_scene.show()

    
    return
