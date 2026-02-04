# RCAIDE/Library/Plots/Geometry/plot_3d_vehicle.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Geometry.generate_3d_wing_points      import *
from RCAIDE.Library.Plots.Geometry.generate_3d_fuselage_points  import *
from RCAIDE.Library.Plots.Geometry.generate_3d_fuel_tank_points import *
from RCAIDE.Library.Plots.Geometry.plot_3d_rotor                import generate_3d_blade_points
from RCAIDE.Library.Plots.Geometry.generate_3d_nacelle_points   import *
from RCAIDE.Library.Plots.Geometry.generate_3d_lopa_points      import generate_3d_lopa_points
from RCAIDE.Library.Plots.Geometry.generate_3d_cargo_bay_points import generate_3d_cargo_bay_points
from RCAIDE.Library.Methods.Geometry.Planform                   import  fuselage_planform, wing_planform, bwb_wing_planform , compute_fuel_volume  
from RCAIDE.Library.Methods.Geometry.LOPA                       import  compute_layout_of_passenger_accommodations  

# python imports 
import numpy as np  
from copy import deepcopy 
import vtk 
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
                    export_gltf                 = False, 
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
                    camera_eye_x                = None,
                    camera_eye_y                = None,
                    camera_eye_z                = None,
                    overwrite_geometry          = True, 
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
                elif issubclass(type(fuel_tank), RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank):
                    GEOM  = generate_non_integral_fuel_tank_points(fuel_tank,tessellation ) 
                    make_object(renderer, GEOM,  fuel_tank_rgb_color, fuel_tank_opacity) 

                    if wing.xz_plane_symmetric: 
                        GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1] 
                        make_object(renderer, GEOM,  fuel_tank_rgb_color, fuel_tank_opacity)
                        
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

    elif side_view:
        camera_eye_x  =  0  
        camera_eye_y  = -1 
        camera_eye_z  =  0  

    elif top_view:
        camera_eye_x  = 0
        camera_eye_y  = 0 
        camera_eye_z  = 1  

    else: 
        camera_eye_x  = camera_eye_x if camera_eye_x is not None else -1
        camera_eye_y  = camera_eye_y if camera_eye_y is not None else -1
        camera_eye_z  = camera_eye_z if camera_eye_z is not None else 0.35  
    
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
    geometry =  deepcopy(vehicle)  
    for wing in geometry.wings:  
        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            if overwrite_geometry: 
                bwb_wing_planform(wing) 
                compute_layout_of_passenger_accommodations(wing)
        else:
            if overwrite_geometry:
                wing_planform(wing)  
                     
    compute_fuel_volume(geometry, compute_fuel_volume=True) 
    
    for fuselage in  geometry.fuselages:    
        compute_layout_of_passenger_accommodations(fuselage)
        fuselage_planform(fuselage) 
    
    # -------------------------------------------------------------------------  
    # Initalize Renderer
    # -------------------------------------------------------------------------      
    renderer = vtk.vtkRenderer()
        
    # -------------------------------------------------------------------------  
    # Plot wings
    # -------------------------------------------------------------------------  
    for wing in geometry.wings:
        if isinstance(wing,RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            wing_opacity /= 1.25 # For clearer visualzation of lopa
        n_segments = len(wing.segments)
        dim        = n_segments if n_segments > 0 else 2
        GEOM       = generate_3d_wing_points(wing, number_of_airfoil_points, dim)
        make_object(renderer, GEOM,wing_rgb_color,wing_opacity)
        if wing.yz_plane_symmetric: 
            GEOM.PTS[:, :, 0] = -GEOM.PTS[:, :, 0]
            make_object(renderer, GEOM,wing_rgb_color,wing_opacity)
        if wing.xz_plane_symmetric: 
            GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1]
            make_object(renderer, GEOM,wing_rgb_color,wing_opacity)
        if wing.xy_plane_symmetric: 
            GEOM.PTS[:, :, 2] = -GEOM.PTS[:, :, 2]
            make_object(renderer, GEOM,wing_rgb_color,wing_opacity)
        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            if show_LOPA:
                lopa_geom = generate_3d_lopa_points(wing)
                add_lopa_seats(renderer, lopa_geom, lopa_opacity)

    # -------------------------------------------------------------------------  
    # Plot fuselage
    # -------------------------------------------------------------------------  
    for fuselage in geometry.fuselages:
        GEOM = generate_3d_fuselage_points(fuselage, tessellation)
        make_object(renderer, GEOM, fuselage_rgb_color,fuselage_opacity)
        if show_LOPA:
            lopa_geom = generate_3d_lopa_points(fuselage)
            add_lopa_seats(renderer, lopa_geom, lopa_opacity)
        
    
    # -------------------------------------------------------------------------  
    # Plot cargo bay
    # -------------------------------------------------------------------------  
    for cargo_bay in geometry.cargo_bays:
        GEOM = generate_3d_cargo_bay_points(cargo_bay)
        make_object(renderer, GEOM, cargo_bay_rgb_color,cargo_bay_opacity) 
        
    # -------------------------------------------------------------------------  
    # Plot boom
    # -------------------------------------------------------------------------  
    for boom in geometry.booms:
        GEOM = generate_3d_fuselage_points(boom, tessellation)
        make_object(renderer, GEOM, boom_rgb_color,boom_opacity)

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
                make_object(renderer, GEOM, nacelle_rgb_color,nacelle_opacity)
                    
            if 'rotor' in propulsor:  
                rot       = propulsor.rotor
                rot_x     = rot.orientation_euler_angles[0]
                rot_y     = np.pi / 2 +  rot.orientation_euler_angles[1]
                rot_z     = rot.orientation_euler_angles[2]
                num_B     = int(rot.number_of_blades) 
                if (rot.radius_distribution) is None or (plot_actuator_disc == True):  
                    make_actuator_disc(renderer, rot.hub_radius, rot.tip_radius, rot.origin, rot_x,rot_y,rot_z, rotor_rgb_color,rotor_opacity) 
                else:
                    dim       = len(rot.radius_distribution) 
                    for i in range(num_B):
                        GEOM = generate_3d_blade_points(rot,number_of_airfoil_points,dim,i)
                        make_object(renderer, GEOM, rotor_rgb_color,rotor_opacity) 

            if 'propeller' in propulsor:
                prop      = propulsor.propeller
                rot_x     = prop.orientation_euler_angles[0]
                rot_y     = np.pi / 2 +  prop.orientation_euler_angles[1]
                rot_z     = prop.orientation_euler_angles[2]
                num_B     = int(prop.number_of_blades) 
                if (prop.radius_distribution is None ) or ( plot_actuator_disc == True):  
                    make_actuator_disc(renderer, prop.hub_radius, prop.tip_radius, prop.origin, rot_x,rot_y,rot_z,rotor_rgb_color,rotor_opacity) 
                else:
                    dim       = len(prop.radius_distribution)
                    for i in range(num_B):
                        GEOM = generate_3d_blade_points(prop,number_of_airfoil_points,dim,i) 
                        make_object(renderer, GEOM, rotor_rgb_color,rotor_opacity) 

        for fuel_line in network.fuel_lines:        
            for fuel_tank in fuel_line.fuel_tanks:   
                if fuel_tank.wing_tag != None:
                    wing = geometry.wings[fuel_tank.wing_tag]
                    if issubclass(type(fuel_tank), RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank):
                        GEOM  = generate_non_integral_fuel_tank_points(fuel_tank,tessellation ) 
                        make_object(renderer, GEOM,  fuel_tank_rgb_color, fuel_tank_opacity) 
    
                        if wing.xz_plane_symmetric: 
                            GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1] 
                            make_object(renderer, GEOM,  fuel_tank_rgb_color, fuel_tank_opacity)

                    if type(fuel_tank) == RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank:  
                        seg_bounds = fuel_tank.segments_bounding_tank   
                        GEOM       = generate_integral_wing_tank_points(wing,5,seg_bounds,fuel_tank)
                        make_object(renderer, GEOM, fuel_tank_rgb_color, fuel_tank_opacity)  
                        if wing.xz_plane_symmetric:
                            GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1] 
                            make_object(renderer, GEOM,fuel_tank_rgb_color, fuel_tank_opacity) 

                elif fuel_tank.fuselage_tag != None:
                    fuselage = geometry.fuselages[fuel_tank.fuselage_tag]
                    if type(fuel_tank) == RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank:  
                        seg_bounds  = fuel_tank.segments_bounding_tank  
                        GEOM        = generate_integral_fuel_tank_points(fuselage,fuel_tank, seg_bounds,tessellation )
                        make_object(renderer, GEOM,  fuel_tank_rgb_color, fuel_tank_opacity) 

                elif issubclass(type(fuel_tank), RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank):
                    GEOM  = generate_non_integral_fuel_tank_points(fuel_tank,tessellation ) 
                    make_object(renderer, GEOM,  fuel_tank_rgb_color, fuel_tank_opacity) 

                    if wing.xz_plane_symmetric: 
                        GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1] 
                        make_object(renderer, GEOM,  fuel_tank_rgb_color, fuel_tank_opacity) 
        
    # Set camera and background
    camera = vtk.vtkCamera()
    camera.SetPosition(camera_eye_x, camera_eye_y, camera_eye_z)
    camera.SetFocalPoint(0, 0, 0)
    camera.SetViewUp(0, 0, 1)

    renderer.SetActiveCamera(camera)
    renderer.ResetCamera()
    renderer.SetBackground(1.0, 1.0, 1.0)  # Background color 
    
    # 5. Create a render window to display the scene
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)
    renderWindow.SetSize(1500, 1500)
    renderWindow.SetWindowName(save_filename)
    
    # 6. Create an interactor to handle user input (mouse, keyboard)
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    # Use the custom interactor style
    custom_style = vtk.vtkInteractorStyleTrackballCamera() 
    renderWindowInteractor.SetInteractorStyle(custom_style) 

    if export_gltf:
        # 4. Export to GLTF
        exporter = vtk.vtkGLTFExporter()
        exporter.SetFileName( save_filename + ".gltf")
        exporter.SetRenderWindow(renderWindow)
        exporter.SetInlineData(True)  
        exporter.Write()
        
    if save_figure:
        # Create a vtkWindowToImageFilter to capture the render window content
        window_to_image = vtk.vtkWindowToImageFilter()
        window_to_image.SetInput(renderWindow)
        window_to_image.SetInputBufferTypeToRGBA()  # or RGB
        window_to_image.ReadFrontBufferOff()  # Read from back buffer for off-screen rendering
        window_to_image.Update()
        
        # Create a vtkPNGWriter to save the image
        writer = vtk.vtkPNGWriter()
        writer.SetFileName(save_filename + ".png")
        writer.SetInputConnection(window_to_image.GetOutputPort())
        writer.Write()
        
    # Start the VTK interactor 
    if show_figure:      
        renderWindowInteractor.Initialize()
        renderWindow.Render() # Render the scene initially
        renderWindowInteractor.Start()

    return 
    
def make_object(renderer, GEOM,  rgb_color, opacity): 

    actor = generate_vtk_object(GEOM.PTS)

    # Set color of fuselage
    mapper = actor.GetMapper()
    mapper.ScalarVisibilityOff()
    actor.GetProperty().SetColor(rgb_color[0], rgb_color[1], rgb_color[2])  # Set wing color to Light Grey
    actor.GetProperty().SetDiffuse(1.0)  # Set diffuse reflection
    actor.GetProperty().SetSpecular(0.0)  # Set specular reflection
    actor.GetProperty().SetOpacity(opacity)
    renderer.AddActor(actor)
    
    return


def add_lopa_seats(renderer, lopa_geometry, opacity):
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

        renderer.AddActor(actor)

def make_actuator_disc(renderer, inner_radius, outer_radius, origin, rot_x,rot_y,rot_z, rgb_color, opacity): 
    
    disk_source = vtk.vtkDiskSource()
    disk_source.SetInnerRadius(inner_radius)
    disk_source.SetOuterRadius(outer_radius)
    disk_source.SetRadialResolution(50)
    disk_source.SetCircumferentialResolution(50) 
    
    # 2. Define a rotation using vtkTransform
    transform = vtk.vtkTransform()
    transform.RotateX(rot_x/Units.degrees)  
    transform.RotateY(rot_y/Units.degrees)  
    transform.RotateZ(rot_z/Units.degrees)  

    # 3. Apply the transformation with vtkTransformPolyDataFilter
    transformFilter = vtk.vtkTransformPolyDataFilter()
    transformFilter.SetTransform(transform)
    transformFilter.SetInputConnection(disk_source.GetOutputPort()) 
 
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(transformFilter.GetOutputPort())
    
    actor = vtk.vtkActor()
    actor.SetMapper(mapper) 
    actor.GetProperty().SetColor(rgb_color[0], rgb_color[1], rgb_color[2])  
    actor.GetProperty().SetDiffuse(1.0)  
    actor.GetProperty().SetSpecular(0.0) 
    actor.GetProperty().SetOpacity(opacity)
    actor.SetPosition( origin[0][0],  origin[0][1],  origin[0][2]) 
    renderer.AddActor(actor)
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
 