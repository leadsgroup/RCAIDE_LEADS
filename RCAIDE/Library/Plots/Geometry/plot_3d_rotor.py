## @ingroup Library-Plots-Geometry  
# RCAIDE/Library/Plots/Geometry/plot_3d_rotor.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Data 
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry
from RCAIDE.Library.Methods.Geometry.Airfoil import compute_naca_4series 

# python imports
import numpy as np
import pyvista as pv
import matplotlib.colors as mcolors

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ------------------------- ---------------------------------------------------------------------------------------------     
def plot_3d_rotor(rotor,
                  save_filename            = "rotor",
                  save_figure              = False,
                  plot_data                = None, 
                  show_figure              = True, 
                  camera_eye_x             = -1, 
                  camera_eye_y             = -1, 
                  camera_eye_z             = 0.35,                 
                  number_of_airfoil_points = 101,
                  color                    = 'black',
                  opacity                  = 1):
    """
    Creates a 3D visualization of a rotor with multiple blades.

    Parameters
    ----------
    rotor : Rotor
        RCAIDE rotor data structure containing geometry and blade information
        
    save_filename : str, optional
        Name of file for saved figure (default: "Rotor")
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    plot_data : list, optional
        Existing plot data to append to (default: None)
        
    show_figure : bool, optional
        Flag to display the figure (default: True) 
        
    number_of_airfoil_points : int, optional
        Number of points used to discretize airfoil sections (default: 21)
        
    color_map : str, optional
        Color specification for the rotor surface (default: 'turbid')
        
    alpha : float, optional
        Transparency value between 0 and 1 (default: 1)

    Returns
    -------
    None or plot_data : list
        If plot_data provided, returns updated list of plot vertices

    Notes
    -----
    Creates an interactive 3D visualization with:
        - Multiple blades at specified angular positions
        - Airfoil sections properly twisted and scaled
        - Optional coordinate axes
        - Adjustable view angles
    
    """

    rotor_rgb_color = mcolors.to_rgb(color)
    num_B = rotor.number_of_blades
    dim   = len(rotor.radius_distribution)

    plotter = pv.Plotter(off_screen=save_figure)

    for i in range(num_B):
        GEOM = generate_3d_blade_points(rotor, number_of_airfoil_points, dim, i)
        make_object(plotter, GEOM, rotor_rgb_color, opacity)

    plotter.camera_position = [
        (camera_eye_x, camera_eye_y, camera_eye_z),
        (0, 0, 0),
        (0, 0, 1),
    ]
    plotter.set_background('white')
    plotter.window_size = [1500, 1500]

    if save_figure:
        plotter.screenshot(save_filename + ".png")
    elif show_figure:
        plotter.show()

    return
 
def generate_3d_blade_points(rotor, n_points, dim, i, aircraftRefFrame = True):
    """
    Generates 3D coordinate points for a single rotor blade.

    Parameters
    ----------
    rotor : Rotor
        RCAIDE rotor data structure containing blade geometry information
        
    n_points : int
        Number of points around airfoil sections
        
    dim : int
        Number of radial blade sections
        
    i : int
        Blade number (0 to number_of_blades-1)
        
    aircraftRefFrame : bool, optional
        Convert coordinates to aircraft frame if True (default: True)

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
    Generates blade geometry by:
        1. Creating airfoil sections at specified radial positions
        2. Applying twist, chord, and thickness distributions
        3. Rotating to proper azimuthal position
        4. Converting to aircraft frame if requested
    
    **Definitions**
    
    'Mid-chord Alignment'
        Reference point for blade section positioning and twist
    'Aircraft Frame'
        Coordinate system with x-back, z-up orientation
    """
    # unpack 
    num_B        = rotor.number_of_blades
    airfoils     = rotor.airfoils 
    beta         = rotor.twist_distribution 
    a_o          = rotor.start_angle
    b            = rotor.chord_distribution
    r            = rotor.radius_distribution
    MCA          = rotor.mid_chord_alignment
    t            = rotor.max_thickness_distribution
    a_loc        = rotor.airfoil_polar_stations
    origin       = rotor.origin 

    theta  = np.linspace(0,2*np.pi,num_B+1)[:-1] 
    flip_2 =  (np.pi/2)

    MCA_2d             = np.repeat(np.atleast_2d(MCA).T,n_points,axis=1)
    b_2d               = np.repeat(np.atleast_2d(b).T  ,n_points,axis=1)
    t_2d               = np.repeat(np.atleast_2d(t).T  ,n_points,axis=1)
    r_2d               = np.repeat(np.atleast_2d(r).T  ,n_points,axis=1)
    airfoil_le_offset  = np.repeat(b[:,None], n_points, axis=1)/2  

    # get airfoil coordinate geometry
    a_loc =  np.array(a_loc)
    if len(airfoils.keys())>0:
        xpts  = np.zeros((dim,n_points))
        zpts  = np.zeros((dim,n_points))
        max_t = np.zeros(dim)
        for af_idx,airfoil in enumerate(airfoils):
            geometry     = import_airfoil_geometry(airfoil.coordinate_file,n_points)
            locs         = np.where(a_loc == af_idx)
            xpts[locs]   = geometry.x_coordinates  
            zpts[locs]   = geometry.y_coordinates  
            max_t[locs]  = geometry.thickness_to_chord 

    else: 
        airfoil_data = compute_naca_4series('2410',n_points)
        xpts         = np.repeat(np.atleast_2d(airfoil_data.x_coordinates) ,dim,axis=0)
        zpts         = np.repeat(np.atleast_2d(airfoil_data.y_coordinates) ,dim,axis=0)
        max_t        = np.repeat(airfoil_data.thickness_to_chord,dim,axis=0)

    # store points of airfoil in similar format as Vortex Points (i.e. in vertices)
    max_t2d = np.repeat(np.atleast_2d(max_t).T ,n_points,axis=1)

    xp      = (- MCA_2d + xpts*b_2d - airfoil_le_offset)     # x-coord of airfoil
    yp      = r_2d*np.ones_like(xp)                          # radial location
    zp      = zpts*(t_2d/max_t2d)                            # former airfoil y coord
    
    if rotor.clockwise_rotation:
        zp *= -1
     
    commanded_thrust_vector      = np.zeros((1,1))
    rotor_vel_to_body,orientaion = rotor.prop_vel_to_body(commanded_thrust_vector)
    cpts                         = len(rotor_vel_to_body[:,0,0])

    matrix        = np.zeros((len(zp),n_points,3)) # radial location, airfoil pts (same y)
    matrix[:,:,0] = xp
    matrix[:,:,1] = yp
    matrix[:,:,2] = zp
    matrix        = np.repeat(matrix[None,:,:,:], cpts, axis=0)


    # ROTATION MATRICES FOR INNER SECTION
    # rotation about y axis to create twist and position blade upright
    trans_1        = np.zeros((dim,3,3))
    trans_1[:,0,0] = np.cos(- beta)
    trans_1[:,0,2] = -np.sin(- beta)
    trans_1[:,1,1] = 1
    trans_1[:,2,0] = np.sin(- beta)
    trans_1[:,2,2] = np.cos(- beta)
    trans_1        = np.repeat(trans_1[None,:,:,:], cpts, axis=0)

    # rotation about x axis to create azimuth locations
    trans_2 = np.array([[1 , 0 , 0],
                        [0 , np.cos(theta[i] + a_o + flip_2 ), -np.sin(theta[i] +a_o +  flip_2)],
                        [0,np.sin(theta[i] + a_o + flip_2), np.cos(theta[i] + a_o + flip_2)]])
    trans_2 = np.repeat(trans_2[None,:,:], dim, axis=0)
    trans_2 = np.repeat(trans_2[None,:,:,:], cpts, axis=0)

    # rotation about y to orient propeller/rotor to thrust angle (from propeller frame to aircraft frame)
    trans_3 =  rotor_vel_to_body
    trans_3 =  np.repeat(trans_3[:, None,:,: ],dim,axis=1) 

    trans     = np.matmul(trans_2,trans_1)
    rot_mat   = np.repeat(trans[:,:, None,:,:],n_points,axis=2)    

    # ---------------------------------------------------------------------------------------------
    # ROTATE POINTS
    if aircraftRefFrame:
        # rotate all points to the thrust angle with trans_3
        mat  =  np.matmul(np.matmul(rot_mat,matrix[...,None]).squeeze(axis=-1), trans_3)
    else:
        # use the rotor frame
        mat  =  np.matmul(rot_mat,matrix[...,None]).squeeze(axis=-1)
    # ---------------------------------------------------------------------------------------------
    # create empty data structure for storing geometry
    G = Data()

    # store node points
    G.X  = mat[0,:,:,0] + origin[0][0]
    G.Y  = mat[0,:,:,1] + origin[0][1]
    G.Z  = mat[0,:,:,2] + origin[0][2]

    G.PTS = np.zeros((len(zp),n_points,3))    
    G.PTS[:,:,0] =  mat[0,:,:,0] + origin[0][0]    
    G.PTS[:,:,1] =  mat[0,:,:,1] + origin[0][1]    
    G.PTS[:,:,2] =  mat[0,:,:,2] + origin[0][2]    

    # store points
    G.XA1  = mat[0,:-1,:-1,0] + origin[0][0]
    G.YA1  = mat[0,:-1,:-1,1] + origin[0][1]
    G.ZA1  = mat[0,:-1,:-1,2] + origin[0][2]
    G.XA2  = mat[0,:-1,1:,0]  + origin[0][0]
    G.YA2  = mat[0,:-1,1:,1]  + origin[0][1]
    G.ZA2  = mat[0,:-1,1:,2]  + origin[0][2]

    G.XB1  = mat[0,1:,:-1,0] + origin[0][0]
    G.YB1  = mat[0,1:,:-1,1] + origin[0][1]
    G.ZB1  = mat[0,1:,:-1,2] + origin[0][2]
    G.XB2  = mat[0,1:,1:,0]  + origin[0][0]
    G.YB2  = mat[0,1:,1:,1]  + origin[0][1]
    G.ZB2  = mat[0,1:,1:,2]  + origin[0][2]    
    
    return G

def make_object(plotter, GEOM, rgb_color, opacity):
    mesh  = generate_vtk_object(GEOM.PTS)
    actor = plotter.add_mesh(mesh, color=rgb_color, opacity=opacity, show_scalar_bar=False)
    prop  = actor.GetProperty()
    prop.SetDiffuse(1.0)
    prop.SetSpecular(0.0)
    return


def generate_vtk_object(pts):
    """Convert a GEOM.PTS array to a pv.PolyData quad mesh."""
    n_r, n_a = pts.shape[0], pts.shape[1]
    n = n_a * (n_r - 1)
    X     = pts.reshape(n_r * n_a, 3).astype(float)
    cells = write_azimuthal_cell_values(X, n, n_a).astype(int)
    faces = np.empty((n, 5), dtype=int)
    faces[:, 0] = 4
    faces[:, 1:] = cells
    return pv.PolyData(X, faces.ravel())


def write_azimuthal_cell_values(f, n_cells, n_a):
    rlap = 0
    adjacent_cells = np.zeros((n_cells, 4))

    for i in range(n_cells):
        if i == (n_a - 1 + n_a * rlap):
            b = i - (n_a - 1)
            c = i + 1
            rlap += 1
        else:
            b = i + 1
            c = i + n_a + 1
        a = i
        d = i + n_a
        adjacent_cells[i, 0] = a
        adjacent_cells[i, 1] = b
        adjacent_cells[i, 2] = c
        adjacent_cells[i, 3] = d
    return adjacent_cells 