# RCAIDE/Library/Plots/Geometry/plot_3d_fuel_tank.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------   
import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Plots.Geometry.Common.contour_surface_slice import contour_surface_slice
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry
from RCAIDE.Library.Methods.Geometry.Airfoil import compute_naca_4series 
import numpy as np     
from RCAIDE.Library.Plots.Geometry.Common.contour_surface_slice import contour_surface_slice
import numpy as np   

# ----------------------------------------------------------------------------------------------------------------------
#  plot_3d_concetric_fuel_tank
# ----------------------------------------------------------------------------------------------------------------------   
def plot_3d_concetric_fuel_tank(plot_data, fuel_tank, tessellation = 24, color_map = 'orange', alpha=1):
    """
    Creates a 3D visualization of a fuel_tank surface using tessellated panels.

    Parameters
    ----------
    plot_data : list
        Collection of plot vertices to be rendered
        
    fuel_tank : fuel_tank
        RCAIDE fuel_tank data structure containing geometry information
        
    tessellation : int, optional
        Number of points to use in circumferential discretization (default: 24)
        
    color_map : str, optional
        Color specification for the fuel_tank surface (default: 'teal')

    Returns
    -------
    plot_data : list
        Updated collection of plot vertices including fuel_tank surface

    Notes
    -----
    Creates a 3D surface by generating points along fuel_tank segments and 
    creating surface panels between adjacent cross-sections.
    
    **Major Assumptions**
    
    * fuel_tank cross-sections are super-elliptical
    * Surface is continuous between segments
    * Tessellation is uniform around circumference
    
    See Also
    --------
    generate_3d_fuel_tank_points : Function to generate fuel_tank surface points
    """
    G  = generate_3d_fuel_tank_points(fuel_tank,tessellation = 24 ) 
    num_fus_segs = len(G.PTS[:,0,0])
    if num_fus_segs > 0:
        tesselation  = len(G.PTS[0,:,0])
        for i_seg in range(num_fus_segs-1):
            for i_tes in range(tesselation-1):
                X = np.array([[G.PTS[i_seg  ,i_tes,0],G.PTS[i_seg+1,i_tes  ,0]],
                              [G.PTS[i_seg  ,i_tes+1,0],G.PTS[i_seg+1,i_tes+1,0]]])
                Y = np.array([[G.PTS[i_seg  ,i_tes  ,1],G.PTS[i_seg+1,i_tes  ,1]],
                              [G.PTS[i_seg  ,i_tes+1,1],G.PTS[i_seg+1,i_tes+1,1]]])
                Z = np.array([[G.PTS[i_seg  ,i_tes  ,2],G.PTS[i_seg+1,i_tes  ,2]],
                              [G.PTS[i_seg  ,i_tes+1,2],G.PTS[i_seg+1,i_tes+1,2]]])  
                 
                values = np.ones_like(X) 
                verts  = contour_surface_slice(X, Y, Z,values,color_map )
                plot_data.append(verts)          

    return plot_data 
 
# ----------------------------------------------------------------------------------------------------------------------
#  plot_3d_integral_wing_tank
# ----------------------------------------------------------------------------------------------------------------------  
def plot_3d_integral_wing_tank(plot_data, wing, number_of_airfoil_points = 201, color_map='orange', alpha=1):
    """
    Creates a 3D visualization of wing surfaces including symmetric sections if applicable.

    Parameters
    ----------
    plot_data : list
        Collection of plot vertices to be rendered
        
    wing : Wing
        RCAIDE wing data structure containing geometry information
        
    number_of_airfoil_points : int, optional
        Number of points used to discretize airfoil sections (default: 21)
        
    color_map : str, optional
        Color specification for wing surface (default: 'greys')
        
    alpha : float, optional
        Transparency value between 0 and 1 (default: 1)

    Returns
    -------
    plot_data : list
        Updated collection of plot vertices including wing surfaces

    Notes
    -----
    Creates wing visualization by:
        - Generating points for each segment
        - Creating surface panels between sections
        - Adding symmetric wing if specified
    
    **Major Assumptions**
    
    * Wing segments are ordered from root to tip
    * Airfoil sections lie in x-z plane
    * Symmetric wing is mirror image about y-axis
    """ 
    af_pts    = 4
    n_segments = len(wing.segments)  
    if n_segments>0:
        dim =  n_segments
    else:
        dim = 2 
 
    G = generate_integral_wing_tank_points(wing,number_of_airfoil_points,dim)
    # ------------------------------------------------------------------------
    # Plot Rotor Blade
    # ------------------------------------------------------------------------
    for sec in range(dim-1):
        for loc in range(af_pts):
            X = np.array([[G.XA1[sec,loc],G.XA2[sec,loc]],
                 [G.XB1[sec,loc],G.XB2[sec,loc]]])
            Y = np.array([[G.YA1[sec,loc],G.YA2[sec,loc]],
                 [G.YB1[sec,loc],G.YB2[sec,loc]]])
            Z = np.array([[G.ZA1[sec,loc],G.ZA2[sec,loc]],
                 [G.ZB1[sec,loc],G.ZB2[sec,loc]]]) 
             
            values      = np.ones_like(X) 
            verts       = contour_surface_slice(X, Y, Z ,values,color_map)
            plot_data.append(verts)
    if wing.symmetric:
        if wing.vertical: 
            for sec in range(dim-1):
                for loc in range(af_pts):
                    X = np.array([[G.XA1[sec,loc],G.XA2[sec,loc]],[G.XB1[sec,loc],G.XB2[sec,loc]]])
                    Y = np.array([[G.YA1[sec,loc],G.YA2[sec,loc]],[G.YB1[sec,loc],G.YB2[sec,loc]]])
                    Z = np.array([[-G.ZA1[sec,loc], -G.ZA2[sec,loc]],[-G.ZB1[sec,loc], -G.ZB2[sec,loc]]]) 
                     
                    values      = np.ones_like(X) 
                    verts       = contour_surface_slice(X, Y, Z ,values,color_map)
                    plot_data.append(verts)
        else:
            for sec in range(dim-1):
                for loc in range(af_pts):
                    X = np.array([[G.XA1[sec,loc],G.XA2[sec,loc]],[G.XB1[sec,loc],G.XB2[sec,loc]]])
                    Y = np.array([[-G.YA1[sec,loc], -G.YA2[sec,loc]], [-G.YB1[sec,loc], -G.YB2[sec,loc]]])
                    Z = np.array([[G.ZA1[sec,loc],G.ZA2[sec,loc]], [G.ZB1[sec,loc],G.ZB2[sec,loc]]]) 
                     
                    values      = np.ones_like(X) 
                    verts       = contour_surface_slice(X, Y, Z ,values,color_map)
                    plot_data.append(verts)
            
             
    return plot_data
 
# ----------------------------------------------------------------------------------------------------------------------
#  helper functions
# ----------------------------------------------------------------------------------------------------------------------  
 
def generate_integral_wing_tank_points(wing, n_points, dim):
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
    symm                 = wing.symmetric
    semispan             = wing.spans.projected*0.5 * (2 - symm) 
    root_chord           = wing.chords.root
    segments             = wing.segments
    n_segments           = len(segments.keys()) 
    origin               = wing.origin   
        
    if n_segments > 0: 
        pts              = np.zeros((dim,n_points, 3,1))  
        section_twist    = np.zeros((dim,n_points, 3,3))
        section_twist[:, :, 0, 0] = 1        
        section_twist[:, :, 1, 1] = 1
        section_twist[:, :, 2, 2] = 1 
        translation        = np.zeros((dim,n_points, 3,1)) 
        translation[0, :, 0,:] = origin[0][0]  
        translation[0, :, 1,:] = origin[0][1]  
        translation[0, :, 2,:] = origin[0][2]  
        
        for i in range(n_segments):
            current_seg = list(segments.keys())[i]
        
            inner_front_rib_yu,inner_rear_rib_yu,inner_front_rib_yl,inner_rear_rib_yl = compute_non_dimensional_rib_coordinates(current_seg)
            fs = current_seg.structural.front_spar_percent_chord
            rs = current_seg.structural.rear_spar_percent_chord  
            x_coordinates =  np.array([rs, rs, fs, fs, rs])
            y_coordinates =  np.array([inner_rear_rib_yl, inner_rear_rib_yu, inner_front_rib_yu,inner_front_rib_yl,inner_rear_rib_yl ]) 
             
            
            if (i == n_segments-1):
                sweep = 0                                 
            else: 
                next_seg = list(segments.keys())[i+1]                
                if wing.segments[current_seg].sweeps.leading_edge is not None: 
                    # If leading edge sweep is defined 
                    sweep       = wing.segments[current_seg].sweeps.leading_edge  
                else:   
                    # If quarter chord sweep is defined, convert it to leading edge sweep
                    sweep_quarter_chord = wing.segments[current_seg].sweeps.quarter_chord 
                    chord_fraction      = 0.25                          
                    segment_root_chord  = root_chord*wing.segments[current_seg].root_chord_percent
                    segment_tip_chord   = root_chord*wing.segments[next_seg].root_chord_percent
                    segment_span        = semispan*(wing.segments[next_seg].percent_span_location - wing.segments[current_seg].percent_span_location )
                    sweep               = np.arctan(((segment_root_chord*chord_fraction) + (np.tan(sweep_quarter_chord )*segment_span - chord_fraction*segment_tip_chord)) /segment_span) 
            dihedral = wing.segments[current_seg].dihedral_outboard    
            twist    = wing.segments[current_seg].twist
            
            if wing.vertical:  
                pts[i,:,0,0]   = x_coordinates * wing.segments[current_seg].root_chord_percent * wing.chords.root 
                pts[i,:,1,0]   = y_coordinates * wing.segments[current_seg].root_chord_percent * wing.chords.root 
                pts[i,:,2,0]   = np.zeros_like(y_coordinates) 
              
                section_twist[i,:,0,0] = np.cos(twist) 
                section_twist[i,:,0,1] = -np.sin(twist)  
                section_twist[i,:,1,0] = np.sin(twist) 
                section_twist[i,:,1,1] = np.cos(twist) 
            
            else: 
                pts[i,:,0,0]   = x_coordinates * wing.segments[current_seg].root_chord_percent * wing.chords.root
                pts[i,:,1,0]   = np.zeros_like(y_coordinates) 
                pts[i,:,2,0]   = y_coordinates * wing.segments[current_seg].root_chord_percent * wing.chords.root   

                section_twist[i,:,0,0] = np.cos(twist) 
                section_twist[i,:,0,2] = np.sin(twist)  
                section_twist[i,:,2,0] = -np.sin(twist) 
                section_twist[i,:,2,2] =  np.cos(twist)  
             
            if (i != n_segments-1):
                # update origin for next segment 
                segment_percent_span =    wing.segments[next_seg].percent_span_location - wing.segments[current_seg].percent_span_location     
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
                translation[i+1,:,0,:] = translation[i,:,0,:] + dx
                translation[i+1,:,1,:] = translation[i,:,1,:] + dy
                translation[i+1,:,2,:] = translation[i,:,2,:] + dz 
    else:

        pts              = np.zeros((dim,n_points, 3,1))  
        section_twist    = np.zeros((dim,n_points, 3,3))
        section_twist[:, :, 0, 0] = 1        
        section_twist[:, :, 1, 1] = 1
        section_twist[:, :, 2, 2] = 1
        translation      = np.zeros((dim,n_points, 3,1))
        

    
        inner_front_rib_yu,inner_rear_rib_yu,inner_front_rib_yl,inner_rear_rib_yl = compute_non_dimensional_rib_coordinates(wing)
        fs = current_seg.structural.front_spar_percent_chord
        rs = current_seg.structural.rear_spar_percent_chord  
        x_coordinates =  np.array([rs, rs, fs, fs, rs])
        y_coordinates =  np.array([inner_rear_rib_yl, inner_rear_rib_yu, inner_front_rib_yu,inner_front_rib_yl,inner_rear_rib_yl ]) 
        
         
            
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
            pts[0,:,0,0]   = x_coordinates *  wing.chords.root
            pts[0,:,1,0]   = y_coordinates *  wing.chords.root
            pts[0,:,2,0]   = np.zeros_like(y_coordinates)
            
            pts[1,:,0,0]   = x_coordinates *  wing.chords.tip  
            pts[1,:,1,0]   = y_coordinates *  wing.chords.tip  
            pts[1,:,2,0]   = np.zeros_like(y_coordinates)   
            
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
            pts[0,:,0,0]   = x_coordinates *  wing.chords.root
            pts[0,:,1,0]   = np.zeros_like(y_coordinates) 
            pts[0,:,2,0]   = y_coordinates *  wing.chords.root
            
            pts[1,:,0,0]   = x_coordinates *  wing.chords.tip  
            pts[1,:,1,0]   = np.zeros_like(y_coordinates)  
            pts[1,:,2,0]   = y_coordinates *  wing.chords.tip  
            
    
            translation[1, :, 0,:] +=  semispan*np.tan(sweep)
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


def compute_non_dimensional_rib_coordinates(compoment):
    # import inner_segment airfoil 
    if compoment.airfoil != None: 
        if type(compoment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
            geometry = compute_naca_4series(compoment.airfoil.NACA_4_Series_code)
        elif type(compoment.airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil: 
            geometry = import_airfoil_geometry(compoment.airfoil.coordinate_file)
    else:
        geometry = compute_naca_4series('0012')
  
    # inner spar 
    inner_front_rib_nondim_x       = compoment.structural.front_spar_percent_chord   
    inner_rear_rib_nondim_x       = compoment.structural.rear_spar_percent_chord 
    
    # non-wing box dimension coordinates 
    front_rib_nondim_y_upper = np.interp(inner_front_rib_nondim_x, geometry.x_upper_surface  ,geometry.y_upper_surface ) 
    rear_rib_nondim_y_upper  = np.interp(inner_rear_rib_nondim_x, geometry.x_upper_surface  ,geometry.y_upper_surface )     
    front_rib_nondim_y_lower = np.interp(inner_front_rib_nondim_x, geometry.x_lower_surface   , geometry.y_lower_surface)     
    rear_rib_nondim_y_lower  = np.interp(inner_rear_rib_nondim_x, geometry.x_lower_surface   , geometry.y_lower_surface)
    
         
    return front_rib_nondim_y_upper,rear_rib_nondim_y_upper, front_rib_nondim_y_lower, rear_rib_nondim_y_lower


def generate_3d_fuel_tank_points(fuel_tank, tessellation = 24):
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
    
    See Also
    --------
    plot_3d_fuel_tank : Function to visualize the generated surface
    """
    num_fus_segs = len(fuel_tank.segments.keys())
    fuel_tank_points = np.zeros((num_fus_segs,tessellation ,3))
     
    if num_fus_segs > 0:
        for i_seg, segment in enumerate(fuel_tank.segments): 
            a = segment.width/2
            b = segment.height/2
            n = segment.curvature
            theta    = np.linspace(0,2*np.pi,tessellation) 
            fus_ypts =  (abs((np.cos(theta)))**(2/n))*a * ((np.cos(theta)>0)*1 - (np.cos(theta)<0)*1) 
            fus_zpts =  (abs((np.sin(theta)))**(2/n))*b * ((np.sin(theta)>0)*1 - (np.sin(theta)<0)*1)  
            fuel_tank_points[i_seg,:,0] = segment.percent_x_location*fuel_tank.lengths.total + fuel_tank.origin[0][0]
            fuel_tank_points[i_seg,:,1] = fus_ypts + segment.percent_y_location*fuel_tank.lengths.total + fuel_tank.origin[0][1]
            fuel_tank_points[i_seg,:,2] = fus_zpts + segment.percent_z_location*fuel_tank.lengths.total + fuel_tank.origin[0][2]
        
    G = Data()         
    G.PTS  = fuel_tank_points

    return G
