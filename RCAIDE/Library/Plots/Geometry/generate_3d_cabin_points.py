# RCAIDE/Library/Plots/Geometry/generate_3d_cabin_points.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data     
from RCAIDE.Library.Methods.Geometry.Airfoil                import  compute_naca_4series, import_airfoil_geometry  

import numpy as np     

# ----------------------------------------------------------------------------------------------------------------------
#  generate_3d_cabin_points
# ----------------------------------------------------------------------------------------------------------------------   
def generate_3d_cabin_points(component, n_points,plot_centerline = False):
    """
    Generates 3D coordinate points that define a cabin surface.

    Parameters
    ----------
    cabin : Cabin
        RCAIDE cabin data structure containing geometry information
    component : Component
        RCAIDE component data structure containing geometry information 
    n_points : int
        Number of points used to discretize airfoil sections
        
    dim : int
        Number of wing segments plus one

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

    if issubclass(type(component), RCAIDE.Library.Components.Wings.Wing): 
        G = generate_3d_wing_cabin_points(component, n_points, plot_centerline)
    else:
        G = generate_3d_fuselage_cabin_points(component, n_points, plot_centerline)

    return G

def generate_3d_wing_cabin_points(wing, n_points, plot_centerline=False):
    symbolic_cabin_wing =  generate_bwb_cabin_geometry(wing,n_points)

    LOPA_origin   = wing.layout_of_passenger_accommodations.origin
    LOPA_origin_z = LOPA_origin[0][2]
      
    # obtain the geometry for each segment in a loop                                            
    symm                 = wing.xz_plane_symmetric
    semispan             = wing.spans.projected*0.5 * (2 - symm) 
    root_chord           = wing.chords.root
    segments             = symbolic_cabin_wing.segments
    n_segments           = len(segments.keys()) 
    origin               = wing.origin   
         
    pts              = np.zeros((n_segments+2,n_points, 3,1))  
    section_twist    = np.zeros((n_segments+2,n_points, 3,3))
    section_twist[:, :, 0, 0] = 1        
    section_twist[:, :, 1, 1] = 1
    section_twist[:, :, 2, 2] = 1 
    translation        = np.zeros((n_segments+2,n_points, 3,1))  
    translation[:, :, 0,:] = origin[0][0]  
    translation[:, :, 1,:] = origin[0][1]  
    translation[:, :, 2,:] = origin[0][2]   
    for i in range(n_segments+2): 
        if i == 0: 
            current_seg = list(segments.keys())[i]
            airfoil  = symbolic_cabin_wing.segments[current_seg].airfoil  
            geometry = airfoil.geometry 
            twist    = symbolic_cabin_wing.segments[current_seg].twist  
             
            pts[i,:,0,0]   = geometry.x_coordinates * symbolic_cabin_wing.segments[current_seg].root_chord_percent * root_chord
            pts[i,:,1,0]   = np.zeros_like(geometry.y_coordinates) 
            pts[i,:,2,0]   = (geometry.y_coordinates + geometry.y_coordinates[::-1]) / 2  

            section_twist[i,:,0,0] = np.cos(twist) 
            section_twist[i,:,0,2] = np.sin(twist)  
            section_twist[i,:,2,0] = -np.sin(twist) 
            section_twist[i,:,2,2] =  np.cos(twist)   
    
            translation[i, :, 0,:] += symbolic_cabin_wing.segments[current_seg].origin[0][0]  
            translation[i, :, 1,:] += symbolic_cabin_wing.segments[current_seg].origin[0][1]  
            translation[i, :, 2,:] += symbolic_cabin_wing.segments[current_seg].origin[0][2]
            
        elif i == n_segments + 1: 
            current_seg = list(segments.keys())[i-2]
            airfoil  = symbolic_cabin_wing.segments[current_seg].airfoil  
            geometry = airfoil.geometry 
            twist    = symbolic_cabin_wing.segments[current_seg].twist  
             
            pts[i,:,0,0]   = geometry.x_coordinates * symbolic_cabin_wing.segments[current_seg].root_chord_percent * root_chord
            pts[i,:,1,0]   = np.zeros_like(geometry.y_coordinates) 
            pts[i,:,2,0]   = (geometry.y_coordinates + geometry.y_coordinates[::-1]) / 2 

            section_twist[i,:,0,0] = np.cos(twist) 
            section_twist[i,:,0,2] = np.sin(twist)  
            section_twist[i,:,2,0] = -np.sin(twist) 
            section_twist[i,:,2,2] =  np.cos(twist)   
    
            translation[i, :, 0,:] += symbolic_cabin_wing.segments[current_seg].origin[0][0]  
            translation[i, :, 1,:] += symbolic_cabin_wing.segments[current_seg].origin[0][1]  
            translation[i, :, 2,:] += symbolic_cabin_wing.segments[current_seg].origin[0][2]
            
        else:
            current_seg = list(segments.keys())[i-1]
            airfoil  = symbolic_cabin_wing.segments[current_seg].airfoil  
            geometry = airfoil.geometry 
            twist    = symbolic_cabin_wing.segments[current_seg].twist  
             
            pts[i,:,0,0]   = geometry.x_coordinates * symbolic_cabin_wing.segments[current_seg].root_chord_percent * root_chord
            pts[i,:,1,0]   = np.zeros_like(geometry.y_coordinates) 
            pts[i,:,2,0]   = geometry.y_coordinates * symbolic_cabin_wing.segments[current_seg].root_chord_percent * wing.chords.root 
                            
    
            section_twist[i,:,0,0] = np.cos(twist) 
            section_twist[i,:,0,2] = np.sin(twist)  
            section_twist[i,:,2,0] = -np.sin(twist) 
            section_twist[i,:,2,2] =  np.cos(twist)   
    
            translation[i, :, 0,:] += symbolic_cabin_wing.segments[current_seg].origin[0][0]  
            translation[i, :, 1,:] += symbolic_cabin_wing.segments[current_seg].origin[0][1]  
            translation[i, :, 2,:] += symbolic_cabin_wing.segments[current_seg].origin[0][2]
        
        if i > 1:
            # update origin for next segment
            prev_seg = list(segments.keys())[i-2]  

            sweep    = symbolic_cabin_wing.segments[prev_seg].sweeps.leading_edge
            dihedral = symbolic_cabin_wing.segments[prev_seg].dihedral_outboard
        
            segment_percent_span =  symbolic_cabin_wing.segments[current_seg].percent_span_location  -  symbolic_cabin_wing.segments[prev_seg].percent_span_location   
            dy = semispan*segment_percent_span
            dz = dy*np.tan(dihedral)
            l  = dy/np.cos(dihedral)
            dx = l*np.tan(sweep)
            translation[i,:,0,:] = translation[i-1,:,0,:] + dx
            translation[i,:,1,:] = translation[i-1,:,1,:] + dy
            translation[i,:,2,:] = translation[i-1,:,2,:] + dz  
 
    mat     = translation + np.matmul(section_twist ,pts)
    mat[:, :, 2, 0] = np.maximum(mat[:, :, 2, 0], LOPA_origin_z)

    if not plot_centerline:
        mat = mat[1:, :, :, :]

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
     
def generate_bwb_cabin_geometry(wing, n_points):
    
    symbolic_cabin_wing = RCAIDE.Library.Components.Wings.Wing()  

    LOPA_data = wing.layout_of_passenger_accommodations
    LOPA      = LOPA_data.object_coordinates 
    
    # Step 1: plot cabin bounds  
    # get points at x min 
    x_min_locs   =  np.where( LOPA[:,2] == min(LOPA[:,2]))[0]
    x_min        =  LOPA[x_min_locs[0],2] -  LOPA[x_min_locs[0],5]/2
    x_min_y_max  =  max(LOPA[x_min_locs,3] + LOPA[x_min_locs,6]/2 )
    x_min_y_min  =  min(LOPA[x_min_locs,3] - LOPA[x_min_locs,6]/2 ) 
    x_border_pts = [x_min, x_min] 
    y_border_pts = [x_min_y_min, x_min_y_max] 

    # get points at y max 
    y_max_locs   =  np.where( LOPA[:,3] == max(LOPA[:,3]))[0]
    y_max        =  LOPA[y_max_locs[0],3] + LOPA[y_max_locs[0],6]/2 
    y_max_x_max  =  max(LOPA[y_max_locs,2] + LOPA[y_max_locs[0],5]/2)
    y_max_x_min  =  min(LOPA[y_max_locs,2] - LOPA[y_max_locs[0],5]/2) 
    x_border_pts.append(y_max_x_min)
    x_border_pts.append(y_max_x_max)
    y_border_pts.append(y_max)
    y_border_pts.append(y_max) 

    # get points at x max 
    x_max_locs   =  np.where( LOPA[:,2] == max(LOPA[:,2]))[0]
    x_max        =  LOPA[x_max_locs[0],2] + LOPA[x_max_locs[0],5]/2
    x_max_y_max  =  max(LOPA[x_max_locs,3] + LOPA[x_max_locs,6]/2)
    x_max_y_min  =  min(LOPA[x_max_locs,3] - LOPA[x_max_locs,6]/2)  
    x_border_pts.append(x_max)
    x_border_pts.append(x_max)
    y_border_pts.append(x_max_y_max)
    y_border_pts.append(x_max_y_min)

    # get points at y min  
    y_min_locs   =  np.where( LOPA[:,3] == min(LOPA[:,3]))[0]
    y_min        =  LOPA[y_min_locs[0],3] - LOPA[y_min_locs[0],6]/2 
    y_min_x_max  =  max(LOPA[y_min_locs,2] + LOPA[y_min_locs[0],5]/2)
    y_min_x_min  =  min(LOPA[y_min_locs,2] - LOPA[y_min_locs[0],5]/2)
    x_border_pts.append(y_min_x_max)  
    x_border_pts.append(y_min_x_min)
    y_border_pts.append(y_min)
    y_border_pts.append(y_min)    

    # loop through points and determine if there are duplicates
    y_border_pts = np.array(y_border_pts)
    x_border_pts = np.array(x_border_pts)
    # cut where y is negative
    port_idxs  =  np.where(y_border_pts<0)[0]
    starboard_x_points = np.delete(x_border_pts, port_idxs) 
    starboard_y_points = np.delete(y_border_pts, port_idxs)

    leading_edge_points  = np.vstack((starboard_x_points[:2] + LOPA_data.origin[0][0], starboard_y_points[:2]))
    trailing_edge_points = np.vstack((starboard_x_points[2:] + LOPA_data.origin[0][0], starboard_y_points[2:]))

    for wing_segment in wing.segments:
        local_chord = wing.chords.root * wing_segment.root_chord_percent
        y_seg = wing_segment.origin[0][0]
        x_le = np.interp(y_seg, leading_edge_points[1], leading_edge_points[0])
        x_te = np.interp(y_seg, trailing_edge_points[1], trailing_edge_points[0])

        airfoil = wing_segment.airfoil  

        if  airfoil !=  None:                 
            if type(airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                geometry = compute_naca_4series(airfoil.NACA_4_Series_code,n_points)
            elif type(airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil: 
                geometry     = import_airfoil_geometry(airfoil.coordinate_file,n_points)
        else:
            t_c = str(int(wing_segment.thickness_to_chord *  100)).zfill(4)
            geometry = compute_naca_4series(t_c,n_points) 


        x_coords = geometry.x_coordinates * local_chord + wing_segment.origin[0][0]
        x_coords[x_coords < x_le] = x_le
        x_coords[x_coords > x_te] = x_te

        x_coords = (x_coords - wing_segment.origin[0][0]) / (local_chord)
        y_coords =  geometry.y_coordinates  * wing.outer_mold_line_cabin_offset_factor 

        section_y = wing_segment.percent_span_location * wing.spans.projected / 2

        if  section_y <= max(starboard_y_points):
            segment = RCAIDE.Library.Components.Wings.Segments.Segment() 
            segment.tag = wing_segment.tag + '_cabin'
            segment.origin = wing_segment.origin
            segment.percent_span_location = wing_segment.percent_span_location
            segment.root_chord_percent = wing_segment.root_chord_percent
            segment.thickness_to_chord = wing_segment.thickness_to_chord
            segment.sweeps.leading_edge = wing_segment.sweeps.leading_edge
            segment.dihedral_outboard = wing_segment.dihedral_outboard
            segment.twist = wing_segment.twist
            segment.append_airfoil(wing_segment.airfoil)   
            segment.airfoil.geometry.x_coordinates = x_coords
            segment.airfoil.geometry.y_coordinates = y_coords
            
            symbolic_cabin_wing.append_segment(segment)
    return symbolic_cabin_wing

def generate_3d_fuselage_cabin_points(fuselage, n_points, plot_centerline=False):

    LOPA_data = fuselage.layout_of_passenger_accommodations
    LOPA      = LOPA_data.object_coordinates 
    
    # Step 1: plot cabin bounds  
    # get points at x min 
    x_min_locs   =  np.where( LOPA[:,2] == min(LOPA[:,2]))[0]
    x_min        =  LOPA[x_min_locs[0],2] -  LOPA[x_min_locs[0],5]/2
    x_min_y_max  =  max(LOPA[x_min_locs,3] + LOPA[x_min_locs,6]/2 )
    x_min_y_min  =  min(LOPA[x_min_locs,3] - LOPA[x_min_locs,6]/2 ) 
    x_border_pts = [x_min, x_min] 
    y_border_pts = [x_min_y_min, x_min_y_max] 

    # get points at y max 
    y_max_locs   =  np.where( LOPA[:,3] == max(LOPA[:,3]))[0]
    y_max        =  LOPA[y_max_locs[0],3] + LOPA[y_max_locs[0],6]/2 
    y_max_x_max  =  max(LOPA[y_max_locs,2] + LOPA[y_max_locs[0],5]/2)
    y_max_x_min  =  min(LOPA[y_max_locs,2] - LOPA[y_max_locs[0],5]/2) 
    x_border_pts.append(y_max_x_min)
    x_border_pts.append(y_max_x_max)
    y_border_pts.append(y_max)
    y_border_pts.append(y_max) 

    # get points at x max 
    x_max_locs   =  np.where( LOPA[:,2] == max(LOPA[:,2]))[0]
    x_max        =  LOPA[x_max_locs[0],2] + LOPA[x_max_locs[0],5]/2
    x_max_y_max  =  max(LOPA[x_max_locs,3] + LOPA[x_max_locs,6]/2)
    x_max_y_min  =  min(LOPA[x_max_locs,3] - LOPA[x_max_locs,6]/2)  
    x_border_pts.append(x_max)
    x_border_pts.append(x_max)
    y_border_pts.append(x_max_y_max)
    y_border_pts.append(x_max_y_min)

    # get points at y min  
    y_min_locs   =  np.where( LOPA[:,3] == min(LOPA[:,3]))[0]
    y_min        =  LOPA[y_min_locs[0],3] - LOPA[y_min_locs[0],6]/2 
    y_min_x_max  =  max(LOPA[y_min_locs,2] + LOPA[y_min_locs[0],5]/2)
    y_min_x_min  =  min(LOPA[y_min_locs,2] - LOPA[y_min_locs[0],5]/2)
    x_border_pts.append(y_min_x_max)  
    x_border_pts.append(y_min_x_min)
    y_border_pts.append(y_min)
    y_border_pts.append(y_min)    

    # loop through points and determine if there are duplicates
    y_border_pts = np.array(y_border_pts)
    x_border_pts = np.array(x_border_pts)
    # cut where y is negative
    port_idxs  =  np.where(y_border_pts<0)[0]
    starboard_x_points = np.delete(x_border_pts, port_idxs) 
    starboard_y_points = np.delete(y_border_pts, port_idxs)

    leading_edge_points  = np.vstack((starboard_x_points[:2] + LOPA_data.origin[0][0], starboard_y_points[:2]))
    trailing_edge_points = np.vstack((starboard_x_points[2:] + LOPA_data.origin[0][0], starboard_y_points[2:]))

    # Cabin longitudinal extent in absolute fuselage x coordinates
    x_cabin_min = x_min + LOPA_data.origin[0][0]
    x_cabin_max = x_max + LOPA_data.origin[0][0]

    # Scale factor for cabin interior surface relative to outer fuselage mold line
    if hasattr(fuselage, 'outer_mold_line_cabin_offset_factor'):
        cabin_factor = fuselage.outer_mold_line_cabin_offset_factor
    else:
        cabin_factor = 0.95

    # Collect segments whose x position falls within the cabin range
    cabin_segs = []
    for segment in fuselage.segments:
        seg_x = segment.percent_x_location * fuselage.lengths.total + fuselage.origin[0][0]
        if x_cabin_min <= seg_x <= x_cabin_max:
            cabin_segs.append(segment)

    if not cabin_segs:
        cabin_segs = list(fuselage.segments)

    theta        = np.linspace(0, 2 * np.pi, n_points)
    cabin_points = np.zeros((len(cabin_segs), n_points, 3))

    for i, segment in enumerate(cabin_segs):
        a = segment.width  / 2 * cabin_factor
        b = segment.height / 2 * cabin_factor
        n = segment.curvature
        cabin_points[i, :, 0] = (segment.percent_x_location * fuselage.lengths.total
                                  + fuselage.origin[0][0])
        cabin_points[i, :, 1] = ((abs(np.cos(theta)) ** (2 / n)) * a
                                  * ((np.cos(theta) > 0) * 1 - (np.cos(theta) < 0) * 1)
                                  + segment.percent_y_location * fuselage.lengths.total
                                  + fuselage.origin[0][1])
        cabin_points[i, :, 2] = ((abs(np.sin(theta)) ** (2 / n)) * b
                                  * ((np.sin(theta) > 0) * 1 - (np.sin(theta) < 0) * 1)
                                  + segment.percent_z_location * fuselage.lengths.total
                                  + fuselage.origin[0][2])

    G      = Data()
    G.PTS  = cabin_points
    return G