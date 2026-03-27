# RCAIDE/Library/Plots/Geometry/plot_3d_vehicle.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE 
from RCAIDE.Library.Plots.Geometry.generate_3d_wing_points      import *
from RCAIDE.Library.Plots.Geometry.generate_3d_fuselage_points  import *
from RCAIDE.Library.Plots.Geometry.generate_3d_fuel_tank_points import *
from RCAIDE.Library.Plots.Geometry.plot_3d_rotor                import generate_3d_blade_points
from RCAIDE.Library.Plots.Geometry.generate_3d_nacelle_points   import *
from RCAIDE.Library.Plots.Geometry.generate_3d_lopa_points      import generate_3d_lopa_points
from RCAIDE.Library.Plots.Geometry.generate_3d_cargo_bay_points import generate_3d_cargo_bay_points
from RCAIDE.Library.Methods.Geometry.Planform                   import fuselage_planform, wing_planform , compute_fuel_volume  
from RCAIDE.Library.Methods.Geometry.LOPA                       import compute_layout_of_passenger_accommodations  

# python imports 
import numpy as np  
from copy import deepcopy 
import vtk
import pyvista as  pv
import matplotlib.colors as mcolors

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ---------------------------------------------------------------------------------------------------------------------- 
def plot_3d_vehicle(vehicle, 
                    save_figure                 = False,
                    save_filename               = "geometry", 
                    top_view                    = False, 
                    side_view                   = False, 
                    front_view                  = False,   
                    wing_color                  = 'grey', 
                    fuselage_color              = 'grey', 
                    boom_color                  = 'grey', 
                    nacelle_color               = 'grey', 
                    fuel_tank_color             = 'orange', 
                    rotor_color                 = 'black', 
                    cargo_bay_color             = 'blue',
                    plot_actuator_disc          = False,
                    show_LOPA                   = True, 
                    wing_opacity                = 0.5, 
                    fuselage_opacity            = 0.5,
                    boom_opacity                = 1.0,
                    nacelle_opacity             = 1.0,
                    fuel_tank_opacity           = 0.5,
                    lopa_opacity                = 1.0,
                    rotor_opacity               = 0.6, 
                    cargo_bay_opacity           = 0.6, 
                    number_of_airfoil_points    = 101,
                    tessellation                = 96,
                    camera_eye_x                = -1,
                    camera_eye_y                = -1,
                    camera_eye_z                = 0.75,
                    overwrite_geometry          = True,
                    export_gltf                 = False, 
                    show_figure                 = True):
    """
    Creates a complete 3D visualization of an aircraft including all major components.

    Parameters
    ----------
    geometry : geometry
        RCAIDE geometry data structure containing all component geometries

    show_axis : bool, optional
        Flag to display coordinate axes (default: False)

    save_figure : bool, optional
        Flag for saving the figure (default: False)

    save_filename : str, optional
        Name of file for saved figure (default: "Vehicle_Geometry")

    alpha : float, optional
        Transparency value between 0 and 1 (default: 1.0)

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
 
    # -------------------------------------------------------------------------  
    # Initalize Renderer
    # ------------------------------------------------------------------------- 
    if save_figure: 
        plotter = pv.Plotter(off_screen=True)
    else:
        plotter = pv.Plotter()     
    
    # -------------------------------------------------------------------------
    # Object RGB Colors  
    # -------------------------------------------------------------------------    
    fuel_tank_rgb_color  = mcolors.to_rgb(fuel_tank_color)     
    wing_rgb_color       = mcolors.to_rgb(wing_color)
    fuselage_rgb_color   = mcolors.to_rgb(fuselage_color) 
    nacelle_rgb_color    = mcolors.to_rgb(nacelle_color) 
    rotor_rgb_color      = mcolors.to_rgb(rotor_color)
    boom_rgb_color       = mcolors.to_rgb(boom_color)
    cargo_bay_rgb_color  = mcolors.to_rgb(cargo_bay_color)
     
    # -------------------------------------------------------------------------
    # Run Geoemtry Analysis
    # -------------------------------------------------------------------------
    L = 0
    geometry =  deepcopy(vehicle)  
    for wing in geometry.wings:  
        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            if overwrite_geometry: 
                wing_planform(wing) 
                compute_layout_of_passenger_accommodations(wing)
        else:
            if overwrite_geometry:
                wing_planform(wing)
                
        L = np.maximum(L, wing.spans.projected)
                     
    compute_fuel_volume(geometry, compute_fuel_volume=True) 
    
    for fuselage in  geometry.fuselages:    
        compute_layout_of_passenger_accommodations(fuselage)
        fuselage_planform(fuselage) 
        L = np.maximum(L, fuselage.lengths.total)
     
    # -------------------------------------------------------------------------  
    # Plot wings
    # -------------------------------------------------------------------------  
    for wing in geometry.wings:
        n_segments = len(wing.segments)
        dim        = n_segments if n_segments > 0 else 2
        GEOM       = generate_3d_wing_points(wing, number_of_airfoil_points, dim)
        actor        = generate_vtk_object(GEOM.PTS) 
        vtk_data     = actor.GetMapper().GetInput() 
        pyvista_mesh = pv.wrap(vtk_data)                      
        plotter.add_mesh(pyvista_mesh,color= wing_rgb_color,opacity= wing_opacity)
        if wing.yz_plane_symmetric: 
            GEOM.PTS[:, :, 0] = -GEOM.PTS[:, :, 0]
            actor        = generate_vtk_object(GEOM.PTS) 
            vtk_data     = actor.GetMapper().GetInput() 
            pyvista_mesh = pv.wrap(vtk_data)                      
            plotter.add_mesh(pyvista_mesh,color= wing_rgb_color,opacity= wing_opacity)
        if wing.xz_plane_symmetric: 
            GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1]
            actor        = generate_vtk_object(GEOM.PTS) 
            vtk_data     = actor.GetMapper().GetInput() 
            pyvista_mesh = pv.wrap(vtk_data)                      
            plotter.add_mesh(pyvista_mesh,color= wing_rgb_color,opacity= wing_opacity)
        if wing.xy_plane_symmetric: 
            GEOM.PTS[:, :, 2] = -GEOM.PTS[:, :, 2] 
            actor        = generate_vtk_object(GEOM.PTS) 
            vtk_data     = actor.GetMapper().GetInput() 
            pyvista_mesh = pv.wrap(vtk_data)                      
            plotter.add_mesh(pyvista_mesh,color= wing_rgb_color,opacity= wing_opacity)
            
        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            if show_LOPA:
                lopa_geom = generate_3d_lopa_points(wing)
                add_lopa_seats(plotter, lopa_geom, lopa_opacity)
                

    # -------------------------------------------------------------------------  
    # Plot fuselage
    # -------------------------------------------------------------------------  
    for fuselage in geometry.fuselages:
        GEOM = generate_3d_fuselage_points(fuselage, tessellation) 
        actor        = generate_vtk_object(GEOM.PTS) 
        vtk_data     = actor.GetMapper().GetInput() 
        pyvista_mesh = pv.wrap(vtk_data)                      
        plotter.add_mesh(pyvista_mesh,color= fuselage_rgb_color,opacity= fuselage_opacity)        
        if show_LOPA:
            lopa_geom = generate_3d_lopa_points(fuselage)
            add_lopa_seats(plotter, lopa_geom, lopa_opacity) 
    
    # -------------------------------------------------------------------------  
    # Plot cargo bay
    # -------------------------------------------------------------------------  
    for cargo_bay in geometry.cargo_bays:
        GEOM = generate_3d_cargo_bay_points(cargo_bay) 
        actor        = generate_vtk_object(GEOM.PTS) 
        vtk_data     = actor.GetMapper().GetInput() 
        pyvista_mesh = pv.wrap(vtk_data)                      
        plotter.add_mesh(pyvista_mesh,color= cargo_bay_rgb_color,opacity= cargo_bay_opacity)  
        
    # -------------------------------------------------------------------------  
    # Plot boom
    # -------------------------------------------------------------------------  
    for boom in geometry.booms:
        GEOM = generate_3d_fuselage_points(boom, tessellation) 
        actor        = generate_vtk_object(GEOM.PTS) 
        vtk_data     = actor.GetMapper().GetInput() 
        pyvista_mesh = pv.wrap(vtk_data)                      
        plotter.add_mesh(pyvista_mesh,color= boom_rgb_color,opacity= boom_opacity)  

    # -------------------------------------------------------------------------  
    # Plot Nacelle, Rotors and Fuel Tanks 
    # ------------------------------------------------------------------------- 
    for network in geometry.networks:     
        for propulsor in network.propulsors:  
            if propulsor.nacelle !=  None: 
                if type(propulsor.nacelle) == RCAIDE.Library.Components.Nacelles.Stack_Nacelle: 
                    GEOM = generate_3d_stack_nacelle_points(propulsor.nacelle,tessellation = tessellation,number_of_airfoil_points = number_of_airfoil_points)
                elif type(propulsor.nacelle) == RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle: 
                    GEOM = generate_3d_BOR_nacelle_points(propulsor.nacelle,tessellation = tessellation,number_of_airfoil_points = number_of_airfoil_points)
                else:
                    GEOM= generate_3d_basic_nacelle_points(propulsor.nacelle,tessellation = tessellation,number_of_airfoil_points = number_of_airfoil_points) 
                actor        = generate_vtk_object(GEOM.PTS) 
                vtk_data     = actor.GetMapper().GetInput() 
                pyvista_mesh = pv.wrap(vtk_data)                      
                plotter.add_mesh(pyvista_mesh,color= nacelle_rgb_color,opacity= nacelle_opacity)  
                    
            if 'rotor' in propulsor:  
                rot       = propulsor.rotor
                rot_x     = rot.orientation_euler_angles[0]
                rot_y     = rot.orientation_euler_angles[1]
                rot_z     = rot.orientation_euler_angles[2]
                num_B     = int(rot.number_of_blades) 
                if (rot.radius_distribution) is None or (plot_actuator_disc == True):  
                    make_actuator_disc(plotter, rot.hub_radius, rot.tip_radius, rot.origin, rot_x,rot_y,rot_z, rotor_rgb_color,rotor_opacity) 
                else:
                    rot_y += np.pi / 2 
                    dim       = len(rot.radius_distribution) 
                    for i in range(num_B):
                        GEOM = generate_3d_blade_points(rot,number_of_airfoil_points,dim,i) 
                        actor        = generate_vtk_object(GEOM.PTS) 
                        vtk_data     = actor.GetMapper().GetInput() 
                        pyvista_mesh = pv.wrap(vtk_data)                      
                        plotter.add_mesh(pyvista_mesh,color= rotor_rgb_color,opacity= rotor_opacity)  

            if 'propeller' in propulsor:
                prop      = propulsor.propeller
                rot_x     = prop.orientation_euler_angles[0]
                rot_y     = prop.orientation_euler_angles[1]
                rot_z     = prop.orientation_euler_angles[2]
                num_B     = int(prop.number_of_blades) 
                if (prop.radius_distribution is None ) or ( plot_actuator_disc == True):  
                    make_actuator_disc(plotter, prop.hub_radius, prop.tip_radius, prop.origin, rot_x,rot_y,rot_z,rotor_rgb_color,rotor_opacity) 
                else:
                    dim       = len(prop.radius_distribution)
                    for i in range(num_B):
                        GEOM = generate_3d_blade_points(prop,number_of_airfoil_points,dim,i)  
                        actor        = generate_vtk_object(GEOM.PTS) 
                        vtk_data     = actor.GetMapper().GetInput() 
                        pyvista_mesh = pv.wrap(vtk_data)                      
                        plotter.add_mesh(pyvista_mesh,color= rotor_rgb_color,opacity= rotor_opacity)                           

        for fuel_line in network.fuel_lines:        
            for fuel_tank in fuel_line.fuel_tanks:   
                if fuel_tank.wing_tag != None:
                    wing = geometry.wings[fuel_tank.wing_tag]
                    if issubclass(type(fuel_tank), RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank):
                        if issubclass(type(fuel_tank), RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank) and fuel_tank.geometry_type == 'conformal' and fuel_tank.bwb_aft_tank:
                            seg_bounds   = fuel_tank.aft_tank_root_chord_bounds   
                            GEOM         = generate_aft_integral_wing_tank_points(wing,5,seg_bounds,fuel_tank)
                            actor        = generate_vtk_object(GEOM.PTS) 
                            vtk_data     = actor.GetMapper().GetInput() 
                            pyvista_mesh = pv.wrap(vtk_data)                      
                            plotter.add_mesh(pyvista_mesh,color= fuel_tank_rgb_color,opacity= fuel_tank_opacity)  
                        elif issubclass(type(fuel_tank), RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank) and fuel_tank.geometry_type == 'conformal':
                            seg_bounds   = fuel_tank.segments_bounding_tank   
                            GEOM         = generate_integral_wing_tank_points(wing,5,seg_bounds,fuel_tank)
                            actor        = generate_vtk_object(GEOM.PTS) 
                            vtk_data     = actor.GetMapper().GetInput() 
                            pyvista_mesh = pv.wrap(vtk_data)                      
                            plotter.add_mesh(pyvista_mesh,color= fuel_tank_rgb_color,opacity= fuel_tank_opacity)  
                            if wing.xz_plane_symmetric:
                                GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1] 
                                actor        = generate_vtk_object(GEOM.PTS) 
                                vtk_data     = actor.GetMapper().GetInput() 
                                pyvista_mesh = pv.wrap(vtk_data)                      
                                plotter.add_mesh(pyvista_mesh,color= fuel_tank_rgb_color,opacity= fuel_tank_opacity)  
                            
                        else: 
                            GEOM  = generate_non_integral_fuel_tank_points(fuel_tank,tessellation )  
                            actor        = generate_vtk_object(GEOM.PTS) 
                            vtk_data     = actor.GetMapper().GetInput() 
                            pyvista_mesh = pv.wrap(vtk_data)                      
                            plotter.add_mesh(pyvista_mesh,color= fuel_tank_rgb_color,opacity= fuel_tank_opacity)                           
        
                            if wing.xz_plane_symmetric: 
                                GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1] 
                                actor        = generate_vtk_object(GEOM.PTS) 
                                vtk_data     = actor.GetMapper().GetInput() 
                                pyvista_mesh = pv.wrap(vtk_data)                      
                                plotter.add_mesh(pyvista_mesh,color= fuel_tank_rgb_color,opacity= fuel_tank_opacity)   

                    if type(fuel_tank) == RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank:  
                        seg_bounds   = fuel_tank.segments_bounding_tank   
                        GEOM         = generate_integral_wing_tank_points(wing,5,seg_bounds,fuel_tank)
                        actor        = generate_vtk_object(GEOM.PTS) 
                        vtk_data     = actor.GetMapper().GetInput() 
                        pyvista_mesh = pv.wrap(vtk_data)                      
                        plotter.add_mesh(pyvista_mesh,color= fuel_tank_rgb_color,opacity= fuel_tank_opacity)  
                        if wing.xz_plane_symmetric:
                            GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1] 
                            actor        = generate_vtk_object(GEOM.PTS) 
                            vtk_data     = actor.GetMapper().GetInput() 
                            pyvista_mesh = pv.wrap(vtk_data)                      
                            plotter.add_mesh(pyvista_mesh,color= fuel_tank_rgb_color,opacity= fuel_tank_opacity)                               

                elif fuel_tank.fuselage_tag != None:
                    fuselage = geometry.fuselages[fuel_tank.fuselage_tag]
                    if type(fuel_tank) == RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank:  
                        seg_bounds  = fuel_tank.segments_bounding_tank  
                        GEOM        = generate_integral_fuel_tank_points(fuselage,fuel_tank, seg_bounds,tessellation )
                        actor        = generate_vtk_object(GEOM.PTS) 
                        vtk_data     = actor.GetMapper().GetInput() 
                        pyvista_mesh = pv.wrap(vtk_data)                      
                        plotter.add_mesh(pyvista_mesh,color= fuel_tank_rgb_color,opacity= fuel_tank_opacity)   

                elif issubclass(type(fuel_tank), RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank):
                    GEOM  = generate_non_integral_fuel_tank_points(fuel_tank,tessellation )  
                    actor        = generate_vtk_object(GEOM.PTS) 
                    vtk_data     = actor.GetMapper().GetInput() 
                    pyvista_mesh = pv.wrap(vtk_data)                      
                    plotter.add_mesh(pyvista_mesh,color= fuel_tank_rgb_color,opacity= fuel_tank_opacity)    

                    if wing.xz_plane_symmetric: 
                        GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1] 
                        actor        = generate_vtk_object(GEOM.PTS) 
                        vtk_data     = actor.GetMapper().GetInput() 
                        pyvista_mesh = pv.wrap(vtk_data)  
                        plotter.add_mesh(pyvista_mesh,color= fuel_tank_rgb_color,opacity= fuel_tank_opacity)  
                                           
    if front_view:
        plotter.camera_position = [(-2 * L , 0, 0), (0, 0,0), (0, 0, 1)] 
    elif side_view:
        plotter.camera_position = [(L /2 , 2 * L, 0), (L /4, 0, 0), (0, 0, 1)]  
    elif top_view:
        plotter.camera_position = [(L, 0 , 2 * L ), (L/4, 0,0), (0, 0, 1)]       
    else:
        plotter.camera_position = [(L * camera_eye_x, L * camera_eye_y, L * camera_eye_z), (L /2, 0, 0), (0, 0, 1)]
    
    plotter.window_size = [1500, 1500] # Set resolution
    plotter.set_background('white') # Set background color
        
    if export_gltf:
        plotter.export_gltf(save_filename + ".gltf")
        
    if save_figure:  
        # 4. Save the plot as a PNG image
        plotter.screenshot(save_filename + ".png")          
    else:
        if show_figure: 
            plotter.show()  
    return plotter

def add_lopa_seats(plotter, lopa_geometry, opacity):
    seats = getattr(lopa_geometry, "_lopa_seats", [])
    if not seats:
        return

    # Simple color map (no external constants)
    def _rgb(name):
        return mcolors.to_rgb(name)

    color_map = {
        "first":      _rgb("indianred"),
        "business":   _rgb("seagreen"),
        "economy":    _rgb("steelblue"),
        "galley_lav": _rgb("sandybrown"),
        "other":      _rgb("gray"),
    }

    for seat in seats:
        poly = seat.get("polydata", None)
        if poly is None:
            continue

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(poly)

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        seat_class = seat.get("class", "economy")
        rgb = color_map.get(seat_class, color_map["economy"])

        actor.GetProperty().SetColor(*rgb)
        actor.GetProperty().SetDiffuse(1.0)
        actor.GetProperty().SetSpecular(0.0)
        actor.GetProperty().SetOpacity(float(opacity))

        # Optional: outline emergency row seats subtly (no new constants; reuse color)
        if seat.get("emergency_row", False):
            actor.GetProperty().EdgeVisibilityOn()
            actor.GetProperty().SetEdgeColor(*rgb)  # same hue; outline for emphasis
            actor.GetProperty().SetLineWidth(1.0)
 
        vtk_data     = actor.GetMapper().GetInput() 
        pyvista_mesh = pv.wrap(vtk_data)  
        plotter.add_mesh(pyvista_mesh,color= rgb,opacity= opacity)          

def make_actuator_disc(plotter, inner_radius, outer_radius, origin, rot_x,rot_y,rot_z, rgb_color, opacity):
    
    disc_points =  np.array([[1],
                            [0],
                            [0]])
    x_rotation = np.zeros(( 3, 3))
    x_rotation[0,0] = 1
    x_rotation[1,1] = np.cos(rot_x)
    x_rotation[1,2] = -np.sin(rot_x)
    x_rotation[2,1] = np.sin(rot_x)
    x_rotation[2,2] = np.cos(rot_x)

    y_rotation = np.zeros((3, 3))
    y_rotation[0,0] = np.cos(rot_y)
    y_rotation[0,2] = np.sin(rot_y)
    y_rotation[1,1] = 1
    y_rotation[2,0] = -np.sin(rot_y)
    y_rotation[2,2] = np.cos(rot_y) 

    z_rotation = np.zeros(( 3, 3))
    z_rotation[0,0] = np.cos(rot_z)
    z_rotation[0,1] = -np.sin(rot_z)
    z_rotation[1,0] = np.sin(rot_z)
    z_rotation[1,1] = np.cos(rot_z)
    z_rotation[2,2] = 1
    
    R_total = z_rotation @ y_rotation @ x_rotation
    disc_points_rotated =disc_points.T @ R_total.T  
     
    pyvista_mesh = pv.Disc(c_res=50,
                           inner=inner_radius,
                           outer=outer_radius,
                           normal=(disc_points_rotated[0][0], disc_points_rotated[0][1], disc_points_rotated[0][2]),
                           center= (origin[0][0], origin[0][1],origin[0][2]),
                           )  
    plotter.add_mesh(pyvista_mesh,color= rgb_color,opacity= opacity)  
 
    return
    
def generate_vtk_object(pts):
    comp = vtk.vtkPolyData()
    points = vtk.vtkPoints()
    polys = vtk.vtkCellArray()
    scalars = vtk.vtkFloatArray()

    size = np.shape(pts)
    n_r = size[0]
    n_a = size[1]
    n = n_a * (n_r - 1)  # total number of cells
    X = pts.reshape(n_r * n_a, 3)
    geom_pts = write_azimuthal_cell_values(X, n, n_a)

    size = np.shape(X)
    for i, fxi in enumerate(X):
        points.InsertPoint(i, fxi)
        scalars.InsertTuple1(i, i)
    for pt in geom_pts:
        polys.InsertNextCell(mkVtkIdList(pt))

    comp.SetPoints(points)
    comp.SetPolys(polys)
    comp.GetPointData().SetScalars(scalars)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(comp)
    mapper.SetScalarRange(comp.GetScalarRange())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)

    return actor


def mkVtkIdList(it):
    vil = vtk.vtkIdList()
    for i in it:
        vil.InsertNextId(int(i))
    return vil


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
 