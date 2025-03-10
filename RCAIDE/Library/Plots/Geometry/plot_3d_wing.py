# RCAIDE/Library/Plots/Geometry/plot_3d_wing.py
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

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------  
def plot_3d_wing(plot_data, wing, number_of_airfoil_points = 21, color_map='greys', alpha=1):
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
     
    af_pts    = number_of_airfoil_points-1 
    n_segments = len(wing.segments)  
    if n_segments>0:
        dim =  n_segments
    else:
        dim = 2 
 
    G = generate_3d_wing_points(wing,number_of_airfoil_points,dim)
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
 
def generate_3d_wing_points(wing, n_points, dim):
    """
    Generates 3D coordinate points that define a wing surface.

    Parameters
    ----------
    wing : Wing
        RCAIDE wing data structure containing geometry information
        
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
            airfoil = wing.segments[current_seg].airfoil 
            if  airfoil !=  None:                 
                if type(airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                    geometry = compute_naca_4series(airfoil.NACA_4_Series_code,n_points)
                elif type(airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil: 
                    geometry     = import_airfoil_geometry(airfoil.coordinate_file,n_points)
            else:
                geometry = compute_naca_4series('0012',n_points)  
            
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
                pts[i,:,0,0]   = geometry.x_coordinates * wing.segments[current_seg].root_chord_percent * wing.chords.root 
                pts[i,:,1,0]   = geometry.y_coordinates * wing.segments[current_seg].root_chord_percent * wing.chords.root 
                pts[i,:,2,0]   = np.zeros_like(geometry.y_coordinates) 
              
                section_twist[i,:,0,0] = np.cos(twist) 
                section_twist[i,:,0,1] = -np.sin(twist)  
                section_twist[i,:,1,0] = np.sin(twist) 
                section_twist[i,:,1,1] = np.cos(twist) 
            
            else: 
                pts[i,:,0,0]   = geometry.x_coordinates * wing.segments[current_seg].root_chord_percent * wing.chords.root
                pts[i,:,1,0]   = np.zeros_like(geometry.y_coordinates) 
                pts[i,:,2,0]   = geometry.y_coordinates * wing.segments[current_seg].root_chord_percent * wing.chords.root  
                                 

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
        
        airfoil = wing.airfoil 
        if wing.airfoil != None: 
            if type(airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                geometry = compute_naca_4series(airfoil.NACA_4_Series_code,n_points)
            elif type(airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil: 
                geometry     = import_airfoil_geometry(airfoil.coordinate_file,n_points)
        else:
            geometry = compute_naca_4series('0012',n_points)
            
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
            pts[0,:,0,0]   = geometry.x_coordinates *  wing.chords.root
            pts[0,:,1,0]   = geometry.y_coordinates *  wing.chords.root
            pts[0,:,2,0]   = np.zeros_like(geometry.y_coordinates)
            
            pts[1,:,0,0]   = geometry.x_coordinates *  wing.chords.tip  
            pts[1,:,1,0]   = geometry.y_coordinates *  wing.chords.tip  
            pts[1,:,2,0]   = np.zeros_like(geometry.y_coordinates)   
            
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
            pts[0,:,0,0]   = geometry.x_coordinates *  wing.chords.root
            pts[0,:,1,0]   = np.zeros_like(geometry.y_coordinates) 
            pts[0,:,2,0]   = geometry.y_coordinates *  wing.chords.root
            
            pts[1,:,0,0]   = geometry.x_coordinates *  wing.chords.tip  
            pts[1,:,1,0]   = np.zeros_like(geometry.y_coordinates)  
            pts[1,:,2,0]   = geometry.y_coordinates *  wing.chords.tip  
            
    
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
     


