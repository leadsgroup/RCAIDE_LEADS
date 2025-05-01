## @ingroup Library-Plots-Geometry  
# RCAIDE/Library/Plots/Geometry/plot_3d_rotor.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Plots.Geometry.Common.contour_surface_slice import contour_surface_slice
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry
from RCAIDE.Library.Methods.Geometry.Airfoil import compute_naca_4series

# python imports 
import numpy as np 
import plotly.graph_objects as go     

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------    
def plot_3d_rotor(rotor, save_filename = "Rotor", save_figure = False, plot_data = None,
                  show_figure = True, plot_axis = False, cpt = 0, 
                  number_of_airfoil_points = 21, color_map = 'turbid', alpha = 1):
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
        
    plot_axis : bool, optional
        Flag to show coordinate axes (default: False)
        
    cpt : int, optional
        Control point at which to plot the rotor (default: 0)
        
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
    plot_propeller_only = False
    if plot_data == None: 
        print("\nPlotting rotor") 
    
        plot_propeller_only = True         
        camera        = dict(up=dict(x=0.5, y=0.5, z=1), center=dict(x=0, y=0, z=-0.5), eye=dict(x=1.5, y=1.5, z=.8))
        plot_data     = []
        
    num_B     = rotor.number_of_blades 
    af_pts    = number_of_airfoil_points-1
    dim       = len(rotor.radius_distribution)

    for i in range(num_B):
        G = generate_3d_blade_points(rotor,number_of_airfoil_points,dim,i)
        # ------------------------------------------------------------------------
        # Plot Rotor Blade
        # ------------------------------------------------------------------------
        for sec in range(dim-1):
            for loc in range(af_pts):
                X = np.array([[G.XA1[cpt,sec,loc],G.XA2[cpt,sec,loc]],
                     [G.XB1[cpt,sec,loc],G.XB2[cpt,sec,loc]]])
                Y = np.array([[G.YA1[cpt,sec,loc],G.YA2[cpt,sec,loc]],
                     [G.YB1[cpt,sec,loc],G.YB2[cpt,sec,loc]]])
                Z = np.array([[G.ZA1[cpt,sec,loc],G.ZA2[cpt,sec,loc]],
                     [G.ZB1[cpt,sec,loc],G.ZB2[cpt,sec,loc]]]) 
                 
                values      = np.ones_like(X) 
                verts       = contour_surface_slice(X,Y,Z,values,color_map,alpha)
                plot_data.append(verts)      
            
    axis_limits = np.maximum(np.max(G.XA1), np.maximum(np.max(G.YA1),np.max(G.ZA1)))*2 
    if plot_propeller_only:
        fig = go.Figure(data=plot_data)
        fig.update_scenes(aspectmode   = 'auto',
                          xaxis_visible=plot_axis,
                          yaxis_visible=plot_axis,
                          zaxis_visible=plot_axis
                          )
        fig.update_layout( 
                 width     = 1500,
                 height    = 1500, 
                 scene = dict(
                            xaxis = dict(backgroundcolor="lightgrey", gridcolor="white", showbackground=plot_axis,
                                         zerolinecolor="white", range=[-axis_limits,axis_limits]),
                            yaxis = dict(backgroundcolor="lightgrey", gridcolor="white", showbackground=plot_axis, 
                                         zerolinecolor="white", range=[-axis_limits,axis_limits]),
                            zaxis = dict(backgroundcolor="lightgrey",gridcolor="white",showbackground=plot_axis,
                                         zerolinecolor="white", range=[-axis_limits,axis_limits])),             
                 scene_camera=camera) 
        fig.update_coloraxes(showscale=False)
        fig.update_traces(opacity = alpha) 
        if save_figure: 
            fig.write_image(save_filename + ".png")
        if show_figure:
            fig.write_html( save_filename + '.html', auto_open=True)
        return 
    else: 
        return plot_data
 
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

    if rotor.clockwise_rotation:
        # negative chord and twist to give opposite rotation direction
        b    = -b    
        beta = -beta

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
    G.X  = mat[:,:,:,0] + origin[0][0]
    G.Y  = mat[:,:,:,1] + origin[0][1]
    G.Z  = mat[:,:,:,2] + origin[0][2]

    G.PTS = np.zeros((cpts,len(zp),n_points,3))    
    G.PTS[:,:,:,0] =  mat[:,:,:,0] + origin[0][0]    
    G.PTS[:,:,:,1] =  mat[:,:,:,1] + origin[0][1]    
    G.PTS[:,:,:,2] =  mat[:,:,:,2] + origin[0][2]    

    # store points
    G.XA1  = mat[:,:-1,:-1,0] + origin[0][0]
    G.YA1  = mat[:,:-1,:-1,1] + origin[0][1]
    G.ZA1  = mat[:,:-1,:-1,2] + origin[0][2]
    G.XA2  = mat[:,:-1,1:,0]  + origin[0][0]
    G.YA2  = mat[:,:-1,1:,1]  + origin[0][1]
    G.ZA2  = mat[:,:-1,1:,2]  + origin[0][2]

    G.XB1  = mat[:,1:,:-1,0] + origin[0][0]
    G.YB1  = mat[:,1:,:-1,1] + origin[0][1]
    G.ZB1  = mat[:,1:,:-1,2] + origin[0][2]
    G.XB2  = mat[:,1:,1:,0]  + origin[0][0]
    G.YB2  = mat[:,1:,1:,1]  + origin[0][1]
    G.ZB2  = mat[:,1:,1:,2]  + origin[0][2]    
    
    return G
 