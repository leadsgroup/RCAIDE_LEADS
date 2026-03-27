# RCAIDE/Library/Plots/Geometry/generate_3d_fuel_tank_points.py
# 
# 
# Created:  Jul 2023, M. Clarke
# Modified: Aug 2025, S. Shekar

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------    
import RCAIDE
from RCAIDE.Framework.Core import Data 
from RCAIDE.Library.Methods.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank import compute_non_dimensional_rib_coordinates 
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry, compute_naca_4series

# python imports
import numpy as np
from scipy.interpolate import interp1d
from shapely import Polygon

# ----------------------------------------------------------------------------------------------------------------------
#  generate_integral_wing_tank_points
# ----------------------------------------------------------------------------------------------------------------------  
def generate_integral_wing_tank_points(wing, n_points, segment_list,fuel_tank):
    """
    Generates 3D coordinate points that define a wing surface.

    Parameters
    ----------
    wing : Wing
        RCAIDE wing data structure containing geometry information
        
    n_points : int
        Number of points used to discretize airfoil sections
        
    dim : int
        Number of wing segments 

    Returns
    -------
    G : Data
        Data structure containing generated points with attributes:
            - X, Y, Z : ndarray
                Raw coordinate points
            - PTS : ndarray
                Combined coordinate array
            - XA1, YA1, ZA1, XA2, YA2, ZA2 : ndarray
                Leading edge surface points
            - XB1, YB1, ZB1, XB2, YB2, ZB2 : ndarray
                Trailing edge surface points

    Notes
    -----
    Generates wing geometry by:
        1. Creating airfoil sections at specified span positions
        2. Applying twist, sweep, and dihedral
        3. Scaling sections by local chord
        4. Positioning in aircraft coordinate system
    
    **Definitions**
    
    'Leading Edge Sweep'
        Angle between leading edge and y-axis
    'Quarter Chord Sweep'
        Angle between quarter chord line and y-axis
    'Dihedral'
        Upward angle of wing from horizontal
    """    
    # unpack  
    # obtain the geometry for each segment in a loop                                            
    symm                 = wing.xz_plane_symmetric
    semispan             = wing.spans.projected*0.5 * (2 - symm)  
    segments             = wing.segments
    n_segments           = len(segment_list) 
    origin               = wing.origin   
        
    if len(segments) > 0: 
        if segment_list[0] == None or  segment_list[1] == None:
            raise Exception('Tank segments must be defined')
        pts              = np.zeros((2,n_points, 3,1))  
        section_twist    = np.zeros((2,n_points, 3,3))
        section_twist[:, :, 0, 0] = 1        
        section_twist[:, :, 1, 1] = 1
        section_twist[:, :, 2, 2] = 1 
        translation        = np.zeros((2,n_points, 3,1))    
        translation[:, :, 0,:] = origin[0][0]  
        translation[:, :, 1,:] = origin[0][1]  
        translation[:, :, 2,:] = origin[0][2]  
        for i in range(len(segment_list)):
            current_seg = segments[segment_list[i]]
            fs = fuel_tank.segments_percent_chord_start
            rs = fuel_tank.segments_percent_chord_end  
            front_rib_yu,rear_rib_yu,front_rib_yl,rear_rib_yl = compute_non_dimensional_rib_coordinates(current_seg,fuel_tank,fs[i], rs[i])
            x_coordinates =  np.array([rs[i], rs[i], fs[i], fs[i], rs[i]])
            y_coordinates =  np.array([rear_rib_yl, rear_rib_yu, front_rib_yu,front_rib_yl,rear_rib_yl ])   
            twist    = current_seg.twist 
            if wing.vertical:  
                pts[i,:,0,0]   = x_coordinates * current_seg.root_chord_percent * wing.chords.root 
                pts[i,:,1,0]   = y_coordinates * current_seg.root_chord_percent * wing.chords.root 
                pts[i,:,2,0]   = np.zeros_like(y_coordinates) 
              
                section_twist[i,:,0,0] = np.cos(twist) 
                section_twist[i,:,0,1] = -np.sin(twist)  
                section_twist[i,:,1,0] = np.sin(twist) 
                section_twist[i,:,1,1] = np.cos(twist)  
            else: 
                pts[i,:,0,0]   = x_coordinates * current_seg.root_chord_percent * wing.chords.root
                pts[i,:,1,0]   = np.zeros_like(y_coordinates) 
                pts[i,:,2,0]   = y_coordinates * current_seg.root_chord_percent * wing.chords.root   

                section_twist[i,:,0,0] = np.cos(twist) 
                section_twist[i,:,0,2] = np.sin(twist)  
                section_twist[i,:,2,0] = -np.sin(twist) 
                section_twist[i,:,2,2] =  np.cos(twist)  
    
            translation[i, :, 0,:] += current_seg.origin[0][0]  
            translation[i, :, 1,:] += current_seg.origin[0][1]  
            translation[i, :, 2,:] += current_seg.origin[0][2]
                
            if (i == n_segments-1):  
                # update origin for next segment 
                prev_seg             = segments[segment_list[i-1]]                
                segment_percent_span = current_seg.percent_span_location -  prev_seg.percent_span_location

                sweep    = prev_seg.sweeps.leading_edge
                dihedral = prev_seg.dihedral_outboard
            
                if wing.vertical:
                    dz = semispan*segment_percent_span
                    dy = dz*np.tan(dihedral)
                    l  = dz/np.cos(dihedral)
                    dx = l*np.tan(sweep)
                else:
                    dy = semispan*segment_percent_span
                    dz = dy*np.tan(dihedral)
                    l  = dy/np.cos(dihedral)
                    dx = l*np.tan(sweep)
                    
                translation[i,:,0,:] = translation[i-1,:,0,:] + dx
                translation[i,:,1,:] = translation[i-1,:,1,:] + dy
                translation[i,:,2,:] = translation[i-1,:,2,:] + dz 
    else:

        pts                       = np.zeros((2,n_points, 3,1))  
        section_twist             = np.zeros((2,n_points, 3,3))
        section_twist[:, :, 0, 0] = 1        
        section_twist[:, :, 1, 1] = 1
        section_twist[:, :, 2, 2] = 1
        translation               = np.zeros((2,n_points, 3,1))

        fs                = fuel_tank.segments_percent_chord_start
        rs                = fuel_tank.segments_percent_chord_end       
        front_rib_yu_i,rear_rib_yu_i,front_rib_yl_i,rear_rib_yl_i = compute_non_dimensional_rib_coordinates(wing,fuel_tank,fs[0],rs[0])
        front_rib_yu_o,rear_rib_yu_o,front_rib_yl_o,rear_rib_yl_o = compute_non_dimensional_rib_coordinates(wing,fuel_tank,fs[1],rs[1])
        x_coordinates_i   =  np.array([rs[0], rs[0], fs[0], fs[0], rs[0]])
        x_coordinates_o   =  np.array([rs[1], rs[1], fs[1], fs[1], rs[1]])
        y_coordinates_i   =  np.array([rear_rib_yl_i, rear_rib_yu_i, front_rib_yu_i,front_rib_yl_i,rear_rib_yl_i ]) 
        y_coordinates_o   =  np.array([rear_rib_yl_o, rear_rib_yu_o, front_rib_yu_o,front_rib_yl_o,rear_rib_yl_o ]) 
            
        dihedral              = wing.dihedral
        if wing.sweeps.leading_edge  is not None: 
            sweep      = wing.sweeps.leading_edge
        else:  
            sweep_quarter_chord = wing.sweeps.quarter_chord 
            chord_fraction      = 0.25                          
            segment_root_chord  = wing.chords.root
            segment_tip_chord   = wing.chords.tip
            segment_span        = semispan 
            sweep       = np.arctan(((segment_root_chord*chord_fraction) + (np.tan(sweep_quarter_chord )*segment_span - chord_fraction*segment_tip_chord)) /segment_span)  
           
        # append root section     
        translation[:, :, 0,:] = origin[0][0]  
        translation[:, :, 1,:] = origin[0][1]  
        translation[:, :, 2,:] = origin[0][2] 
       
        if wing.vertical: 
            pts[0,:,0,0]   = x_coordinates_i *  wing.chords.root
            pts[0,:,1,0]   = y_coordinates_i *  wing.chords.root
            pts[0,:,2,0]   = np.zeros_like(y_coordinates_i)
            
            pts[1,:,0,0]   = x_coordinates_o *  wing.chords.tip  
            pts[1,:,1,0]   = y_coordinates_o *  wing.chords.tip  
            pts[1,:,2,0]   = np.zeros_like(y_coordinates_o)   
            
            translation[1, :, 0,:] += semispan*np.tan(sweep)
            translation[1, :, 1,:] += semispan*np.tan(dihedral) 
            translation[1, :, 2,:] += semispan 

            section_twist[0,:,0,0] = np.cos(wing.twists.root) 
            section_twist[0,:,0,1] = -np.sin(wing.twists.root)  
            section_twist[0,:,1,0] = np.sin(wing.twists.root) 
            section_twist[0,:,1,1] = np.cos(wing.twists.root)
             
            section_twist[1,:,0,0] = np.cos(wing.twists.tip) 
            section_twist[1,:,0,1] = -np.sin(wing.twists.tip)  
            section_twist[1,:,1,0] = np.sin(wing.twists.tip) 
            section_twist[1,:,1,1] = np.cos(wing.twists.tip)
            
            
        else:
            pts[0,:,0,0]   = x_coordinates_i *  wing.chords.root
            pts[0,:,1,0]   = np.zeros_like(y_coordinates_i) 
            pts[0,:,2,0]   = y_coordinates_i *  wing.chords.root
            
            pts[1,:,0,0]   = x_coordinates_o *  wing.chords.tip  
            pts[1,:,1,0]   = np.zeros_like(y_coordinates_o)  
            pts[1,:,2,0]   = y_coordinates_o *  wing.chords.tip   
    
            translation[1, :, 0,:] += semispan*np.tan(sweep)
            translation[1, :, 1,:] += semispan 
            translation[1, :, 2,:] += semispan*np.tan(dihedral)     

            section_twist[0,:,0,0] = np.cos(wing.twists.root) 
            section_twist[0,:,0,2] = np.sin(wing.twists.root)  
            section_twist[0,:,2,0] = -np.sin(wing.twists.root) 
            section_twist[0,:,2,2] =  np.cos(wing.twists.root)
             
            section_twist[1,:,0,0] = np.cos(wing.twists.tip) 
            section_twist[1,:,0,2] = np.sin(wing.twists.tip)  
            section_twist[1,:,2,0] = -np.sin(wing.twists.tip) 
            section_twist[1,:,2,2] =  np.cos(wing.twists.tip) 
 
    mat     = translation + np.matmul(section_twist ,pts)
    
    # ---------------------------------------------------------------------------------------------
    # create empty data structure for storing geometry
    G = Data()

    # store node points
    G.X    = mat[:,:,0,0]  
    G.Y    = mat[:,:,1,0]  
    G.Z    = mat[:,:,2,0]
    G.PTS  = mat[:,:,:,0]

    # store points
    G.XA1  = mat[:-1,:-1,0,0] 
    G.YA1  = mat[:-1,:-1,1,0] 
    G.ZA1  = mat[:-1,:-1,2,0] 
    G.XA2  = mat[:-1,1:,0,0]  
    G.YA2  = mat[:-1,1:,1,0]  
    G.ZA2  = mat[:-1,1:,2,0]  
    G.XB1  = mat[1:,:-1,0,0]  
    G.YB1  = mat[1:,:-1,1,0]  
    G.ZB1  = mat[1:,:-1,2,0]  
    G.XB2  = mat[1:,1:,0,0]   
    G.YB2  = mat[1:,1:,1,0]   
    G.ZB2  = mat[1:,1:,2,0]      
    
    return G

def generate_integral_fuel_tank_points(fuselage,fuel_tank, segment_list, tessellation = 24):
    """
    Generates 3D coordinate points that define a fuel_tank surface.

    Parameters
    ----------
    fuel_tank : fuel_tank
        RCAIDE fuel_tank data structure containing geometry information
        
    tessellation : int, optional
        Number of points to use in circumferential discretization (default: 24)

    Returns
    -------
    G : Data
        Data structure containing generated points
            - PTS : ndarray
                Array of shape (num_segments, tessellation, 3) containing 
                x,y,z coordinates of surface points

    Notes
    -----
    Points are generated by creating super-elliptical cross-sections at each segment
    and positioning them according to segment locations.
    
    **Major Assumptions**
        * Cross-sections lie in y-z plane
        * Segments are ordered from nose to tail
        * Origin is at the nose of the fuel_tank
    """ 
    tank_segs         = fuselage.segments
    num_tank_segs     = len(segment_list)
    if segment_list[0] == None or  segment_list[1] == None:
        raise Exception('Tank segments must be defined') 
    fuel_tank_points = np.zeros((num_tank_segs+2,tessellation ,3))
        
    if num_tank_segs > 0: 
        # first segment
        segment_start = tank_segs[segment_list[0]]
        a        = 1E-6
        b        = 1E-6
        n        = segment_start.curvature
        theta    = np.linspace(0,2*np.pi,tessellation) 
        tank_ypts =  (abs((np.cos(theta)))**(2/n))*a * ((np.cos(theta)>0)*1 - (np.cos(theta)<0)*1) 
        tank_zpts =  (abs((np.sin(theta)))**(2/n))*b * ((np.sin(theta)>0)*1 - (np.sin(theta)<0)*1)  
        fuel_tank_points[0,:,0] = segment_start.percent_x_location*fuselage.lengths.total + fuselage.origin[0][0]
        fuel_tank_points[0,:,1] = tank_ypts + segment_start.percent_y_location*fuselage.lengths.total + fuselage.origin[0][1]
        fuel_tank_points[0,:,2] = tank_zpts + segment_start.percent_z_location*fuselage.lengths.total + fuselage.origin[0][2]
        
                
        for i in range(num_tank_segs):  
            segment = tank_segs[segment_list[i]]  
            a = segment.width/2
            b = segment.height/2
            n = segment.curvature
            theta    = np.linspace(0,2*np.pi,tessellation) 
            tank_ypts =  (abs((np.cos(theta)))**(2/n))*a * ((np.cos(theta)>0)*1 - (np.cos(theta)<0)*1) 
            tank_zpts =  (abs((np.sin(theta)))**(2/n))*b * ((np.sin(theta)>0)*1 - (np.sin(theta)<0)*1)  
            fuel_tank_points[i+1,:,0] = segment.percent_x_location*fuselage.lengths.total + fuselage.origin[0][0]
            fuel_tank_points[i+1,:,1] = tank_ypts + segment.percent_y_location*fuselage.lengths.total + fuselage.origin[0][1]
            fuel_tank_points[i+1,:,2] = tank_zpts + segment.percent_z_location*fuselage.lengths.total + fuselage.origin[0][2] 
       
        # last segment
        segment_start = tank_segs[segment_list[-1]]
        a        = 1E-6
        b        = 1E-6
        n        = segment_start.curvature
        theta    = np.linspace(0,2*np.pi,tessellation) 
        tank_ypts =  (abs((np.cos(theta)))**(2/n))*a * ((np.cos(theta)>0)*1 - (np.cos(theta)<0)*1) 
        tank_zpts =  (abs((np.sin(theta)))**(2/n))*b * ((np.sin(theta)>0)*1 - (np.sin(theta)<0)*1)  
        fuel_tank_points[-1,:,0] = segment_start.percent_x_location*fuselage.lengths.total + fuselage.origin[0][0]
        fuel_tank_points[-1,:,1] = tank_ypts + segment_start.percent_y_location*fuselage.lengths.total + fuselage.origin[0][1]
        fuel_tank_points[-1,:,2] = tank_zpts + segment_start.percent_z_location*fuselage.lengths.total + fuselage.origin[0][2]
        
    G = Data()
    
    G.PTS  = fuel_tank_points

    return G

def generate_non_integral_fuel_tank_points(fuel_tank, tessellation = 24):
    """
    Generates 3D coordinate points that define a fuel_tank surface.

    Parameters
    ----------
    fuel_tank : fuel_tank
        RCAIDE fuel_tank data structure containing geometry information
        
    tessellation : int, optional
        Number of points to use in circumferential discretization (default: 24)

    Returns
    -------
    G : Data
        Data structure containing generated points
            - PTS : ndarray
                Array of shape (num_segments, tessellation, 3) containing 
                x,y,z coordinates of surface points

    Notes
    -----
    Points are generated by creating super-elliptical cross-sections at each segment
    and positioning them according to segment locations.
    
    **Major Assumptions**
        * Cross-sections lie in y-z plane
        * Segments are ordered from nose to tail
        * Origin is at the nose of the fuel_tank
    """  

    N = 9
    fuel_tank_points = np.zeros((2*N,tessellation ,3))
    R = fuel_tank.diameters.external / 2
    L = fuel_tank.lengths.external    
         
    # front segments
    front_angles = np.linspace(0, np.pi/2,N) 
    for i in range(len(front_angles)):
        a        = np.sin(front_angles[i]) * R 
        b        = np.sin(front_angles[i]) * R 
        n        = 2
        theta    = np.linspace(0,2*np.pi,tessellation) 
        tank_ypts =  (abs((np.cos(theta)))**(2/n))*a * ((np.cos(theta)>0)*1 - (np.cos(theta)<0)*1) 
        tank_zpts =  (abs((np.sin(theta)))**(2/n))*b * ((np.sin(theta)>0)*1 - (np.sin(theta)<0)*1)  

        fuel_tank_points[i,:,0] = R  * (1 -  np.cos(front_angles[i])) 
        fuel_tank_points[i,:,1] = tank_ypts 
        fuel_tank_points[i,:,2] = tank_zpts 
      
       
    # rear angles 
    rear_angles = np.linspace(np.pi/2,0,N) 
    for j in range(len(rear_angles)):
        a        = np.sin(rear_angles[j]) *R 
        b        = np.sin(rear_angles[j]) *R 
        n        = 2
        theta    = np.linspace(0,2*np.pi,tessellation) 
        tank_ypts =  (abs((np.cos(theta)))**(2/n))*a * ((np.cos(theta)>0)*1 - (np.cos(theta)<0)*1) 
        tank_zpts =  (abs((np.sin(theta)))**(2/n))*b * ((np.sin(theta)>0)*1 - (np.sin(theta)<0)*1)  
        
        fuel_tank_points[i+1+j,:,0] = R *(np.cos(rear_angles[j]))  +  L + R
        fuel_tank_points[i+1+j,:,1] = tank_ypts 
        fuel_tank_points[i+1+j,:,2] = tank_zpts 

    x_rotation = np.zeros(( 3, 3))
    x_rotation[0,0] = 1
    x_rotation[1,1] = np.cos(fuel_tank.orientation_euler_angles[0])
    x_rotation[1,2] = -np.sin(fuel_tank.orientation_euler_angles[0])
    x_rotation[2,1] = np.sin(fuel_tank.orientation_euler_angles[0])
    x_rotation[2,2] = np.cos(fuel_tank.orientation_euler_angles[0])

    y_rotation = np.zeros((3, 3))
    y_rotation[0,0] = np.cos(fuel_tank.orientation_euler_angles[1])
    y_rotation[0,2] = np.sin(fuel_tank.orientation_euler_angles[1])
    y_rotation[1,1] = 1
    y_rotation[2,0] = -np.sin(fuel_tank.orientation_euler_angles[1])
    y_rotation[2,2] = np.cos(fuel_tank.orientation_euler_angles[1]) 

    z_rotation = np.zeros(( 3, 3))
    z_rotation[0,0] = np.cos(fuel_tank.orientation_euler_angles[2])
    z_rotation[0,1] = -np.sin(fuel_tank.orientation_euler_angles[2])
    z_rotation[1,0] = np.sin(fuel_tank.orientation_euler_angles[2])
    z_rotation[1,1] = np.cos(fuel_tank.orientation_euler_angles[2])
    z_rotation[2,2] = 1
    
    R_total = z_rotation @ y_rotation @ x_rotation
    fuel_tank_points = fuel_tank_points @ R_total.T 
    
    # translate to location on aircraft 
    if fuel_tank.orientation_euler_angles   == [0.,0.,np.pi/2]:
        fuel_tank_points[:, :, 0] +=  fuel_tank.origin[0][0] + fuel_tank.diameters.external/2
        fuel_tank_points[:, :, 1] +=  fuel_tank.origin[0][1] - (R + L/2)
        fuel_tank_points[:, :, 2] +=  fuel_tank.origin[0][2]
    else:
        fuel_tank_points[:, :, 0] += fuel_tank.origin[0][0]
        fuel_tank_points[:, :, 1] += fuel_tank.origin[0][1]
        fuel_tank_points[:, :, 2] += fuel_tank.origin[0][2]
    

    if hasattr(fuel_tank, "wing_root_twist"):
        # do one last rotation for root twist of wing 
        wing_root_rotation = np.zeros((3, 3))
        wing_root_rotation[0,0] = np.cos(fuel_tank.wing_root_twist)
        wing_root_rotation[0,2] = np.sin(fuel_tank.wing_root_twist)
        wing_root_rotation[1,1] = 1
        wing_root_rotation[2,0] = -np.sin(fuel_tank.wing_root_twist)
        wing_root_rotation[2,2] = np.cos(fuel_tank.wing_root_twist)
        fuel_tank_points =  fuel_tank_points @ wing_root_rotation.T 
    
    G= Data()
    G.PTS  = fuel_tank_points 

    return G 

def aft_tank_root_chord_bounds(fuel_tank, tessalation = 24):
    """
    Returns aft tank root chord bounds used by aft BWB tank generators.
    """
    return getattr(fuel_tank, "aft_tank_root_chord_bounds", None)


def generate_aft_integral_wing_tank_points(wing, n_points, segment_list, fuel_tank):
    """
    Generates 3D points for a BWB aft conformal integral tank.

    This mirrors the polygon-intersection logic used by
    compute_bwb_aft_integral_prismatic_tank_volume so plotting geometry
    matches volume geometry.
    """
    if segment_list is None or len(segment_list) < 2:
        raise ValueError("segment_list must contain [start_percent, end_percent].")

    if any(val is None for val in [
        segment_list[0],
        segment_list[1],
        fuel_tank.aft_tank_segment_bound
    ]):
        raise ValueError("Aft tank bounds and segment bound must be defined.")

    root_chord = wing.chords.root
    wing_span = wing.spans.projected
    tank_start_percent = segment_list[0]
    tank_end_percent = segment_list[1]

    segments = wing.segments
    seg_tags = list(segments.keys())
    index = seg_tags.index(fuel_tank.aft_tank_segment_bound)
    seg_names = seg_tags[:index + 1]
    num_tank_sections = len(seg_names)

    wing_segment_origins = np.array([segments[tag].origin[0] for tag in seg_names])
    x_tank_bounds = np.linspace(tank_start_percent * root_chord, tank_end_percent * root_chord, 2)

    polygon_points = []
    for seg_i, seg_name in enumerate(seg_names):
        segment = segments[seg_name]
        if seg_i == 0:
            fuel_tank.wing_root_twist = segment.twist

        if segment.airfoil is not None:
            if type(segment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                geometry = compute_naca_4series(segment.airfoil.NACA_4_Series_code)
            elif type(segment.airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil:
                geometry = import_airfoil_geometry(segment.airfoil.coordinate_file)
            else:
                geometry = compute_naca_4series('0012')
        else:
            geometry = compute_naca_4series('0012')

        segment_chord = segment.root_chord_percent * root_chord
        x_points_upper = segment_chord * geometry.x_upper_surface + wing_segment_origins[seg_i][0]
        x_points_lower = segment_chord * geometry.x_lower_surface + wing_segment_origins[seg_i][0]
        z_points_upper = segment_chord * geometry.y_upper_surface + wing_segment_origins[seg_i][2] - fuel_tank.wall_clearance
        z_points_lower = segment_chord * geometry.y_lower_surface + wing_segment_origins[seg_i][2] + fuel_tank.wall_clearance

        upper_fn = interp1d(x_points_upper, z_points_upper, kind='linear')
        lower_fn = interp1d(x_points_lower, z_points_lower, kind='linear')
        upper_z = upper_fn(x_tank_bounds)
        lower_z = lower_fn(x_tank_bounds)

        polygon = [
            (x_tank_bounds[0], upper_z[0]),
            (x_tank_bounds[1], upper_z[1]),
            (x_tank_bounds[1], lower_z[1]),
            (x_tank_bounds[0], lower_z[0]),
            (x_tank_bounds[0], upper_z[0]),
        ]
        polygon_points.append(polygon)

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
            intersection_polygon = max(intersection_polygon.geoms, key=lambda g: g.area)

        area = intersection_polygon.area
        y_curr = segments[seg_names[seg_i]].percent_span_location * wing_span
        y_prev = segments[seg_names[seg_i - 1]].percent_span_location * wing_span
        span_length = y_curr - y_prev

        tank_volumes[seg_i - 1] = area * span_length
        tank_lengths[seg_i - 1] = span_length
        intersection_polygons.append(intersection_polygon)

    if np.all(tank_volumes <= 0.0):
        raise AttributeError("No valid intersection polygon found for aft tank geometry.")

    max_idx = int(np.argmax(tank_volumes))
    best_polygon = intersection_polygons[max_idx]
    if best_polygon is None:
        raise AttributeError("No valid intersection polygon found for aft tank geometry.")

    polygon_for_plot = best_polygon
    if polygon_for_plot.geom_type == 'MultiPolygon':
        polygon_for_plot = max(polygon_for_plot.geoms, key=lambda g: g.area)

    coords = list(polygon_for_plot.exterior.coords)
    if np.allclose(coords[0], coords[-1]):
        coords = coords[:-1]
    if len(coords) != 4:
        # Robust fallback for slightly over-resolved intersections.
        rect = polygon_for_plot.minimum_rotated_rectangle
        coords = list(rect.exterior.coords)[:-1]

    coords_np = np.asarray(coords, dtype=float)
    center = np.mean(coords_np, axis=0)
    angles = np.arctan2(coords_np[:, 1] - center[1], coords_np[:, 0] - center[0])
    coords_np = coords_np[np.argsort(angles)]
    coords_np = np.vstack([coords_np, coords_np[0]])

    span_length = float(tank_lengths[max_idx])
    y0 = -0.5 * span_length
    y1 = 0.5 * span_length

    section_0 = np.column_stack((coords_np[:, 0], np.full(coords_np.shape[0], y0), coords_np[:, 1]))
    section_1 = np.column_stack((coords_np[:, 0], np.full(coords_np.shape[0], y1), coords_np[:, 1]))
    tank_points = np.stack((section_0, section_1), axis=0)

    if hasattr(fuel_tank, "wing_root_twist"):
        wing_root_rotation = np.zeros((3, 3))
        wing_root_rotation[0, 0] = np.cos(fuel_tank.wing_root_twist)
        wing_root_rotation[0, 2] = np.sin(fuel_tank.wing_root_twist)
        wing_root_rotation[1, 1] = 1
        wing_root_rotation[2, 0] = -np.sin(fuel_tank.wing_root_twist)
        wing_root_rotation[2, 2] = np.cos(fuel_tank.wing_root_twist)
        tank_points = tank_points @ wing_root_rotation.T

    G = Data()
    G.PTS = tank_points

    return G
