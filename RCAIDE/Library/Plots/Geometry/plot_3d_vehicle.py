# RCAIDE/Library/Plots/Geometry/plot_3d_vehicle.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Library.Plots.Geometry.plot_3d_fuselage             import plot_3d_fuselage
from RCAIDE.Library.Plots.Geometry.plot_3d_wing                 import plot_3d_wing 
from RCAIDE.Library.Plots.Geometry.plot_3d_nacelle              import plot_3d_nacelle
from RCAIDE.Library.Plots.Geometry.plot_3d_rotor                import plot_3d_rotor
from RCAIDE.Library.Plots.Geometry.plot_3d_fuel_tank            import plot_3d_concetric_fuel_tank, plot_3d_integral_wing_tank

from RCAIDE.Library.Methods.Geometry.LOPA      import  compute_layout_of_passenger_accommodations
from RCAIDE.Library.Methods.Geometry.Planform  import  update_blended_wing_body_planform  
from RCAIDE.Library.Methods.Geometry.Planform  import  fuselage_planform, wing_planform, bwb_wing_planform 

# python imports 
import numpy as np 
import plotly.graph_objects as go  
import os
import sys

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------  
def plot_3d_vehicle(vehicle,
                    show_axis                   = False,
                    save_figure                 = False,
                    save_filename               = "Vehicle_Geometry",
                    alpha                       = 1.0,  
                    min_x_axis_limit            =  -5,
                    max_x_axis_limit            =  40,
                    min_y_axis_limit            =  -20,
                    max_y_axis_limit            =  20,
                    min_z_axis_limit            =  -20,
                    max_z_axis_limit            =  20,
                    top_view                    = False, 
                    side_view                   = False, 
                    front_view                  = False, 
                    camera_eye_x                = -1.5,
                    camera_eye_y                = -1.5,
                    camera_eye_z                = .8,
                    camera_center_x             = 0.,
                    camera_center_y             = 0.,
                    camera_center_z             = -0.5,
                    wing_color                  = 'greys', 
                    fuselage_color              = 'teal', 
                    nacelle_color               = 'darkmint', 
                    fuel_tank_color             = 'oranges', 
                    rotor_color                 = 'turbid', 
                    wing_alpha                  = 0.5, 
                    fuselage_alpha              = 0.5,
                    nacelle_alpha               = 1.0,
                    fuel_tank_alpha             = 1.0,
                    rotor_alpha                 = 1.0,
                    overwrite_geometry          = True, 
                    show_figure                 = True):
    """
    Creates a complete 3D visualization of an aircraft including all major components.

    Parameters
    ----------
    vehicle : Vehicle
        RCAIDE vehicle data structure containing all component geometries

    show_axis : bool, optional
        Flag to display coordinate axes (default: False)

    save_figure : bool, optional
        Flag for saving the figure (default: False)

    save_filename : str, optional
        Name of file for saved figure (default: "Vehicle_Geometry")

    alpha : float, optional
        Transparency value between 0 and 1 (default: 1.0)

    min_x_axis_limit : float, optional
        Minimum x-axis plot limit (default: -5)

    max_x_axis_limit : float, optional
        Maximum x-axis plot limit (default: 40)

    min_y_axis_limit : float, optional
        Minimum y-axis plot limit (default: -20)

    max_y_axis_limit : float, optional
        Maximum y-axis plot limit (default: 20)

    min_z_axis_limit : float, optional
        Minimum z-axis plot limit (default: -20)

    max_z_axis_limit : float, optional
        Maximum z-axis plot limit (default: 20)

    camera_eye_x : float, optional
        Camera eye x-position (default: -1.5)

    camera_eye_y : float, optional
        Camera eye y-position (default: -1.5)

    camera_eye_z : float, optional
        Camera eye z-position (default: 0.8)

    camera_center_x : float, optional
        Camera target x-position (default: 0.0)

    camera_center_y : float, optional
        Camera target y-position (default: 0.0)

    camera_center_z : float, optional
        Camera target z-position (default: -0.5)

    show_figure : bool, optional
        Flag to display the figure (default: True)

    Returns
    -------
    None

    Notes
    -----
    Creates an interactive 3D visualization showing:
        - Wings and control surfaces
        - Fuselage sections
        - Propulsion systems
        - Customizable view and camera angles
    """

    if front_view:
        camera_eye_x  = -1 
        camera_eye_y  = 0
        camera_eye_z  = 0
        camera_center_x  = 0.0
        camera_center_y  = 0.0
        camera_center_z  = 0.0

    elif side_view:
        camera_eye_x  = 0  
        camera_eye_y  = -1 
        camera_eye_z  = 0 
        camera_center_x  = 0 
        camera_center_y  = 0 
        camera_center_z  = 0 

    elif top_view:
        camera_eye_x  = 0
        camera_eye_y  = 0 
        camera_eye_z  = 1 
        camera_center_x  = 0 
        camera_center_y  = 0 
        camera_center_z  = 0 

    else: 
        camera_eye_x  = -1.5 
        camera_eye_y  = -1.5 
        camera_eye_z  = .8
        camera_center_x  = 0 
        camera_center_y  = 0 
        camera_center_z  = 0
        
        
    print("\nPlotting vehicle") 
    camera = dict(
        eye=dict(x=camera_eye_x, y=camera_eye_y, z=camera_eye_z), 
        center=dict(x=camera_center_x, y=camera_center_y, z=camera_center_z)
    )   
    
    plot_data     = []
    
    plot_data,x_min,x_max,y_min,y_max,z_min,z_max  = generate_3d_vehicle_geometry_data(plot_data,
                                                                                       vehicle,
                                                                                       alpha,  
                                                                                       min_x_axis_limit,
                                                                                       max_x_axis_limit,
                                                                                       min_y_axis_limit,
                                                                                       max_y_axis_limit,
                                                                                       min_z_axis_limit,
                                                                                       max_z_axis_limit,
                                                                                       wing_color,
                                                                                       fuselage_color,
                                                                                       nacelle_color, 
                                                                                       fuel_tank_color, 
                                                                                       rotor_color,
                                                                                       wing_alpha,
                                                                                       fuselage_alpha,
                                                                                       nacelle_alpha,
                                                                                       fuel_tank_alpha,
                                                                                       rotor_alpha,
                                                                                       overwrite_geometry, 
                                                                                       )
    

    fig = go.Figure(data=plot_data)
    
    # Use update_layout instead of update_scenes
    fig.update_layout(
        width=1500,
        height=1500,
        scene=dict(
            aspectmode='cube',
            xaxis=dict(backgroundcolor="grey", gridcolor="white", showbackground=show_axis,
                       zerolinecolor="white", range=[x_min, x_max], visible=show_axis),
            yaxis=dict(backgroundcolor="grey", gridcolor="white", showbackground=show_axis, 
                       zerolinecolor="white", range=[y_min, y_max], visible=show_axis),
            zaxis=dict(backgroundcolor="grey", gridcolor="white", showbackground=show_axis,
                       zerolinecolor="white", range=[z_min, z_max], visible=show_axis)
        ),
        scene_camera=camera
    )
    
    fig.update_coloraxes(showscale=False)   
    fig.update_traces(opacity=alpha)

    # Use the first path from sys.path
    save_filename = os.path.join(sys.path[0], save_filename)
    if save_figure:
        fig.write_image(save_filename + ".png")
        
    if show_figure:
        fig.write_html( save_filename + '.html', auto_open=True) 
    
    return     

def generate_3d_vehicle_geometry_data(plot_data,
                                      vehicle, 
                                      alpha                       = 1.0,  
                                      min_x_axis_limit            =  -5,
                                      max_x_axis_limit            =  40,
                                      min_y_axis_limit            =  -20,
                                      max_y_axis_limit            =  20,
                                      min_z_axis_limit            =  -20,
                                      max_z_axis_limit            =  20,
                                      wing_color                  = 'greys', 
                                      fuselage_color              = 'teal', 
                                      nacelle_color               = 'darkmint', 
                                      fuel_tank_color             = 'oranges', 
                                      rotor_color                 = 'turbid', 
                                      wing_alpha                  = 1.0, 
                                      fuselage_alpha              = 1.0,
                                      nacelle_alpha               = 1.0,
                                      fuel_tank_alpha             = 1.0,
                                      rotor_alpha                 = 1.0,
                                      overwrite_geometry          = True, 
                                      ):
    """
    Generates plot data for all vehicle components.

    Parameters
    ----------
    plot_data : list
        Collection of plot vertices to be rendered
        
    vehicle : Vehicle
        RCAIDE vehicle data structure containing all component geometries
        
    alpha : float, optional
        Transparency value between 0 and 1 (default: 1.0)
        
    min_x_axis_limit : float, optional
        Minimum x-axis plot limit (default: -5)
        
    max_x_axis_limit : float, optional
        Maximum x-axis plot limit (default: 40)
        
    min_y_axis_limit : float, optional
        Minimum y-axis plot limit (default: -20)
        
    max_y_axis_limit : float, optional
        Maximum y-axis plot limit (default: 20)
        
    min_z_axis_limit : float, optional
        Minimum z-axis plot limit (default: -20)
        
    max_z_axis_limit : float, optional
        Maximum z-axis plot limit (default: 20)

    Returns
    -------
    plot_data : list
        Updated collection of plot vertices
        
    min_x_axis_limit : float
        Updated minimum x-axis limit
        
    max_x_axis_limit : float
        Updated maximum x-axis limit
        
    min_y_axis_limit : float
        Updated minimum y-axis limit
        
    max_y_axis_limit : float
        Updated maximum y-axis limit
        
    min_z_axis_limit : float
        Updated minimum z-axis limit
        
    max_z_axis_limit : float
        Updated maximum z-axis limit

    Notes
    -----
    Processes geometry for:

        - Wings (using plot_3d_wing)
        - Fuselages (using plot_3d_fuselage)
        - Booms (using plot_3d_fuselage)
        - Energy networks (using plot_3d_energy_network)
    """ 
    

    # -------------------------------------------------------------------------
    # Run Geoemtry Analysis
    # -------------------------------------------------------------------------
     

    # update fuselage properties
    if overwrite_geometry:
        for fuselage in vehicle.fuselages:
            compute_layout_of_passenger_accommodations(fuselage)
            fuselage_planform(fuselage) 

    for wing in vehicle.wings: 

        # ----------------------------
        #  Blended Wing Body
        # ----------------------------
        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            if overwrite_geometry:
                compute_layout_of_passenger_accommodations(wing)  
                update_blended_wing_body_planform(wing)
            if overwrite_geometry:
                bwb_wing_planform(wing,overwrite_reference = True)
                vehicle.reference_area = wing.areas.reference

        # ----------------------------
        # All other Wing Surfaces
        # ----------------------------
        else:
            if overwrite_geometry:
                wing_planform(wing,overwrite_reference =  overwrite_geometry) 
                if isinstance(wing, RCAIDE.Library.Components.Wings.Main_Wing) and overwrite_geometry:
                    vehicle.reference_area = wing.areas.reference
        
    # -------------------------------------------------------------------------
    # PLOT WING
    # ------------------------------------------------------------------------- 
    number_of_airfoil_points = 21
    for wing in vehicle.wings:
        plot_data       = plot_3d_wing(plot_data,wing,number_of_airfoil_points , color_map=wing_color,alpha=wing_alpha) 
        
    # -------------------------------------------------------------------------
    # PLOT FUSELAGE
    # ------------------------------------------------------------------------- 
    for fus in vehicle.fuselages:
        plot_data = plot_3d_fuselage(plot_data,fus,color_map =fuselage_color,alpha=fuselage_alpha)

    
    # -------------------------------------------------------------------------
    # PLOT FUSELAGES AND BOOMS
    # ------------------------------------------------------------------------- 
    for boom in vehicle.booms:
        plot_data = plot_3d_fuselage(plot_data,boom,color_map =fuselage_color,alpha=fuselage_alpha) 
        
    # -------------------------------------------------------------------------
    # PLOT ENERGY NETWORK
    # ------------------------------------------------------------------------- 
    number_of_airfoil_points = 11
    for network in vehicle.networks:
        plot_data = plot_3d_energy_network(plot_data,network,number_of_airfoil_points,nacelle_color, nacelle_alpha, rotor_color, rotor_alpha) 
 
    return plot_data,min_x_axis_limit,max_x_axis_limit,min_y_axis_limit,max_y_axis_limit,min_z_axis_limit,max_z_axis_limit

def plot_3d_energy_network(plot_data,network,number_of_airfoil_points,nacelle_color, nacelle_alpha, rotor_color, rotor_alpha):
    """
    Generates plot data for vehicle energy network components.

    Parameters
    ----------
    plot_data : list
        Collection of plot vertices to be rendered
        
    network : Network
        RCAIDE network data structure containing propulsion components
        
    number_of_airfoil_points : int
        Number of points used to discretize airfoil sections
        
    color_map : str
        Color specification for network components

    Returns
    -------
    plot_data : list
        Updated collection of plot vertices

    Notes
    -----
    Processes geometry for:
        - Nacelles (using plot_3d_nacelle)
        - Rotors (using plot_3d_rotor)
        - Propellers (using plot_3d_rotor)
    """ 
    show_axis     = False 
    save_figure   = False 
    show_figure   = False
    save_filename = 'propulsor'

    for propulsor in network.propulsors:   
        number_of_airfoil_points = 21
        tessellation             = 24
        if 'nacelle' in propulsor: 
            if propulsor.nacelle !=  None: 
                plot_data = plot_3d_nacelle(plot_data,propulsor.nacelle,tessellation,number_of_airfoil_points,nacelle_color,nacelle_alpha) 
        if 'rotor' in propulsor: 
            plot_data = plot_3d_rotor(propulsor.rotor,save_filename,save_figure,plot_data,show_figure,show_axis,0,number_of_airfoil_points,rotor_color,rotor_alpha) 
        if 'propeller' in propulsor:
            plot_data = plot_3d_rotor(propulsor.propeller,save_filename,save_figure,plot_data,show_figure,show_axis,0,number_of_airfoil_points,rotor_color,rotor_alpha) 
        if 'rotor' in propulsor: 
            plot_data = plot_3d_rotor(propulsor.rotor,save_filename,save_figure,plot_data,show_figure,show_axis,0,number_of_airfoil_points,rotor_color,rotor_alpha) 
        if 'propeller' in propulsor:
            plot_data = plot_3d_rotor(propulsor.propeller,save_filename,save_figure,plot_data,show_figure,show_axis,0,number_of_airfoil_points,rotor_color,rotor_alpha) 
       
        for fuel_line in network.fuel_lines: 
            for fuel_tank in fuel_line.fuel_tanks:   
                if fuel_tank.wing != None: 
                    if type(fuel_tank) == RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank: 
                        plot_3d_integral_wing_tank(plot_data,fuel_tank, tessellation, color_map = 'oranges') 
                    elif type(fuel_tank) == RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank:
                        plot_3d_concetric_fuel_tank(plot_data, fuel_tank, tessellation, color_map = 'oranges')   
                elif fuel_tank.fuselage != None:   
                    plot_3d_concetric_fuel_tank(plot_data, fuel_tank, tessellation, color_map = 'oranges')   
            
    return plot_data