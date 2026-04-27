# RCAIDE/Library/Plots/Geometry/generate_3d_wing_points.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry
from RCAIDE.Library.Methods.Geometry.Airfoil import compute_naca_4series 
import numpy as np     

# ----------------------------------------------------------------------------------------------------------------------
#  generate_3d_wing_points
# ----------------------------------------------------------------------------------------------------------------------   
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
    segments             = wing.segments
    n_segments           = len(segments.keys()) 
    origin               = wing.origin   
         
    pts              = np.zeros((dim,n_points, 3,1))  
    section_twist    = np.zeros((dim,n_points, 3,3))
    section_twist[:, :, 0, 0] = 1        
    section_twist[:, :, 1, 1] = 1
    section_twist[:, :, 2, 2] = 1 
    translation        = np.zeros((dim,n_points, 3,1))  
    translation[:, :, 0,:] = origin[0][0]  
    translation[:, :, 1,:] = origin[0][1]  
    translation[:, :, 2,:] = origin[0][2]   
    for i in range(n_segments):
        current_seg = list(segments.keys())[i]
        airfoil = wing.segments[current_seg].airfoil 
        if  airfoil !=  None:                 
            if type(airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                geometry = compute_naca_4series(airfoil.NACA_4_Series_code,n_points)
            elif type(airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil: 
                geometry     = import_airfoil_geometry(airfoil.coordinate_file,n_points)
        else:
            t_c = str(int(wing.segments[current_seg].thickness_to_chord *  100)).zfill(4)
            geometry = compute_naca_4series(t_c,n_points)
             
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

        translation[i, :, 0,:] += segments[current_seg].origin[0][0]  
        translation[i, :, 1,:] += segments[current_seg].origin[0][1]  
        translation[i, :, 2,:] += segments[current_seg].origin[0][2]             
 
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
     


