# RCAIDE/Library/Plots/Geometry/plot_3d_vehicle.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE 
from RCAIDE.Library.Components   import Component    
from RCAIDE.Library.Plots.Geometry.generate_3d_wing_points      import *
from RCAIDE.Library.Plots.Geometry.generate_3d_fuselage_points  import *
from RCAIDE.Library.Plots.Geometry.generate_3d_fuel_tank_points import *
from RCAIDE.Library.Plots.Geometry.plot_3d_rotor                import generate_3d_blade_points, generate_vtk_object
from RCAIDE.Library.Plots.Geometry.generate_3d_nacelle_points   import *
from RCAIDE.Library.Plots.Geometry.generate_3d_lopa_points      import generate_3d_lopa_points
from RCAIDE.Library.Plots.Geometry.generate_3d_torus_points     import generate_3d_torus_points
from RCAIDE.Library.Plots.Geometry.generate_3d_cuboid_points    import generate_3d_cuboid_points
from RCAIDE.Library.Plots.Geometry.generate_3d_propulsor_points import generate_3d_propulsor_points
from RCAIDE.Library.Plots.Geometry.generate_3d_cargo_bay_points import generate_3d_cargo_bay_points
from RCAIDE.Library.Plots.Geometry.generate_3d_cabin_points     import generate_3d_cabin_points
from RCAIDE.Library.Methods.Geometry.Planform                   import fuselage_planform, wing_planform , compute_fuel_volume  
from RCAIDE.Library.Methods.Geometry.LOPA                       import compute_layout_of_passenger_accommodations  

# python imports 
import numpy as np
from copy import deepcopy
import pyvista as pv
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
                    plot_centerline             = False,
                    wing_color                  = 'grey', 
                    fuselage_color              = 'grey', 
                    boom_color                  = 'grey', 
                    nacelle_color               = 'grey', 
                    fuel_tank_color             = 'orange', 
                    rotor_color                 = 'black', 
                    cargo_bay_color             = 'blue',
                    battery_color               = 'green',
                    systems_color               = 'black',
                    propulsor_color             = 'black',
                    cabin_color                 = 'grey',
                    landing_gear_color          = 'grey',
                    plot_actuator_disc          = False,
                    plot_wake                   = False,
                    wake_control_point           = 0,
                    wake_color                  = 'deepskyblue',
                    wake_opacity                = 0.7,
                    wake_tube_radius            = None,
                    wake_stride                 = 1,
                    show_LOPA                   = True,
                    show_Cabin                  = True,
                    wing_opacity                = 0.5, 
                    fuselage_opacity            = 0.5,
                    boom_opacity                = 1.0,
                    nacelle_opacity             = 0.5,
                    fuel_tank_opacity           = 0.5,
                    lopa_opacity                = 1.0,
                    rotor_opacity               = 0.6, 
                    cargo_bay_opacity           = 0.6, 
                    battery_opacity             = 1.0, 
                    propulsor_opacity           = 0.5,
                    cabin_opacity               = 0.75,
                    systems_opacity             = 0.8,
                    landing_gear_opacity        = 1.0,
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

    plot_wake : bool, optional
        Flag to overlay each rotor's prescribed tip-vortex wake geometry (default: False).
        Requires the vehicle's rotors to already have populated `rotor.blades.wake.nodes_body`
        (i.e. a mission using Lifting_Line_Theory must have been evaluated first -- the wake
        is a mission-condition result, not part of the static vehicle geometry). Rotors with no
        wake of their own (e.g. "identical propulsors" that only had thrust/power reused, never
        actually re-solved -- see network.identical_propulsors) reuse another rotor's real wake
        data, translated to their own origin, and mirrored in y only if their own rotation sense
        (clockwise_rotation) actually differs from the source's -- not just because they happen
        to sit on the opposite side of the vehicle, which does not by itself imply the two
        rotors counter-rotate.

    wake_control_point : int, optional
        Which control point's wake geometry to draw, indexing `rotor.blades.wake.nodes_body`
        along its first axis (default: 0).

    wake_color : str, optional
        Color for the wake tube meshes (default: 'deepskyblue')

    wake_opacity : float, optional
        Opacity for the wake tube meshes (default: 0.7)

    wake_tube_radius : float, optional
        Tube radius for rendering each wake filament (default: None, drawn as thin lines
        with no tube geometry)

    wake_stride : int, optional
        Draw every Nth wake-age point along each filament (default: 5). The underlying wake
        geometry is discretized finely (hundreds of points per filament) for aerodynamic
        accuracy in the actual Biot-Savart solve -- that resolution is unnecessary for a visual
        filament and drastically increases render time, especially with wake_tube_radius set
        (each point becomes a tube cross-section). The last point is always kept so the
        filament's tip isn't cut short.

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
    battery_rgb_color    = mcolors.to_rgb(battery_color)
    system_rgb_color     = mcolors.to_rgb(systems_color)
    propulsor_rgb_color  = mcolors.to_rgb(propulsor_color)
    cabin_rgb_color      = mcolors.to_rgb(cabin_color)
    landing_gear_rgb_color = mcolors.to_rgb(landing_gear_color)
     
    # -------------------------------------------------------------------------
    # Run Geometry Analysis
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
    # Plot landing gear 
    # ------------------------------------------------------------------------- 
    for landing_gear in geometry.landing_gears:

        N_t = landing_gear.number_of_gear_types_in_tandem
        N_w = landing_gear.number_of_wheels_in_gear_type

        D            = landing_gear.tire_diameter
        d            = landing_gear.rim_diameter
        w            = landing_gear.tire_width
        strut_length = landing_gear.strut_length
        gear_origin  = landing_gear.origin[0]   # [x, y, z] attachment point on aircraft

        # longitudinal spacing between wheels in the same gear type
        longitudinal_spacing = landing_gear.longitudinal_wheel_spacing * (N_t - 1) if N_t > 1 else 0
        total_wheel_x_span   = D * (N_t - 1) + longitudinal_spacing
        wheel_x_offsets      = np.linspace(-total_wheel_x_span / 2, total_wheel_x_span / 2, N_t) if N_t > 1 else np.array([0.0])

        # lateral spacing between gear types in tandem
        total_wheel_y_span   = w * (N_w - 1) + landing_gear.lateral_wheel_spacing * (N_w - 1) if N_w > 1 else 0
        wheel_y_offsets      = np.linspace(-total_wheel_y_span / 2, total_wheel_y_span / 2, N_w) if N_w > 1 else np.array([0.0])

        for i in range(N_t):
            for j in range(N_w):
                wheel_origin = [
                    gear_origin[0] + wheel_x_offsets[i],
                    gear_origin[1] + wheel_y_offsets[j],
                    gear_origin[2] - strut_length,
                ]
                pts = generate_3d_torus_points(wheel_origin, D, d, w, tessellation=24)
                plotter.add_mesh(generate_vtk_object(pts), color=landing_gear_rgb_color, opacity=landing_gear_opacity)

                if landing_gear.xz_plane_symmetric:
                    wheel_origin[1] = -wheel_origin[1]
                    pts = generate_3d_torus_points(wheel_origin, D, d, w, tessellation=24)
                    plotter.add_mesh(generate_vtk_object(pts), color=landing_gear_rgb_color, opacity=landing_gear_opacity)

    # -------------------------------------------------------------------------
    # Plot wings
    # -------------------------------------------------------------------------
    for wing in geometry.wings: 
        GEOM       = generate_3d_wing_points(wing, number_of_airfoil_points, plot_centerline=False)
        plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=wing_rgb_color, opacity=wing_opacity)
        if wing.yz_plane_symmetric:
            GEOM.PTS[:, :, 0] = -GEOM.PTS[:, :, 0]
            plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=wing_rgb_color, opacity=wing_opacity)
        if wing.xz_plane_symmetric:
            GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1]
            plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=wing_rgb_color, opacity=wing_opacity)
        if wing.xy_plane_symmetric:
            GEOM.PTS[:, :, 2] = -GEOM.PTS[:, :, 2]
            plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=wing_rgb_color, opacity=wing_opacity)
        if isinstance(wing, RCAIDE.Library.Components.Wings.Blended_Wing_Body):
            if show_LOPA:
                if len(wing.cabins) > 0 and len(list(wing.cabins.values())[0].segments_bounding_cabin) > 1:
                    lopa_geom = generate_3d_lopa_points(wing)
                    add_lopa_seats(plotter, lopa_geom, lopa_opacity)
            if show_Cabin and len(wing.cabins) > 0:
                GEOM = generate_3d_cabin_points(wing, number_of_airfoil_points, plot_centerline=False)
                plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=cabin_rgb_color, opacity=cabin_opacity)
                GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1]
                plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=cabin_rgb_color, opacity=cabin_opacity)

    # -------------------------------------------------------------------------
    # Plot fuselage
    # -------------------------------------------------------------------------
    for fuselage in geometry.fuselages:
        GEOM = generate_3d_fuselage_points(fuselage, tessellation)
        plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=fuselage_rgb_color, opacity=fuselage_opacity)
        if show_Cabin:
            if len(fuselage.cabins) > 0 and len(list(fuselage.cabins.values())[0].segments_bounding_cabin) > 1:
                GEOM = generate_3d_cabin_points(fuselage, number_of_airfoil_points, plot_centerline=False)
                plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=cabin_rgb_color, opacity=cabin_opacity)
        if show_LOPA:
            lopa_geom = generate_3d_lopa_points(fuselage)
            add_lopa_seats(plotter, lopa_geom, lopa_opacity)

    # -------------------------------------------------------------------------
    # Plot systems
    # -------------------------------------------------------------------------
    for system in vehicle.systems:
        if isinstance(system, Component):
            GEOM = generate_3d_cuboid_points(system)
            plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=system_rgb_color, opacity=systems_opacity)

    # -------------------------------------------------------------------------
    # Plot cargo bay
    # -------------------------------------------------------------------------
    for cargo_bay in geometry.cargo_bays:
        GEOM = generate_3d_cargo_bay_points(cargo_bay)
        plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=cargo_bay_rgb_color, opacity=cargo_bay_opacity)

    # -------------------------------------------------------------------------
    # Plot boom
    # -------------------------------------------------------------------------
    for boom in geometry.booms:
        GEOM = generate_3d_fuselage_points(boom, tessellation)
        plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=boom_rgb_color, opacity=boom_opacity)

    # -------------------------------------------------------------------------
    # Plot Nacelle, Rotors and Fuel Tanks
    # -------------------------------------------------------------------------
    # Pre-pass: collect (origin, nodes_body, clockwise_rotation) for every rotor/propeller that
    # already has real, origin-consistent wake data (see _wake_matches_origin), so "identical
    # propulsors" that only had their thrust/power reused (rather than actually re-solved -- see
    # network.identical_propulsors in RCAIDE/Framework/Networks/Network.py) can fall back to it,
    # independent of loop order or how many propulsors the vehicle has.
    wake_sources = []
    if plot_wake:
        for network in geometry.networks:
            for propulsor in network.propulsors:
                for attr in ('rotor', 'propeller'):
                    if attr not in propulsor:
                        continue
                    r      = getattr(propulsor, attr)
                    blades = r.get('blades', None)
                    if blades is None:
                        continue
                    wake = blades.get('wake', None)
                    if wake is None or wake.get('nodes_body', None) is None:
                        continue
                    all_cp = wake.nodes_body   # (ctrl_pts, N_wake+1, B, 3)
                    if wake_control_point < all_cp.shape[0]:
                        nodes_body = all_cp[wake_control_point]
                        if _wake_matches_origin(nodes_body, r.origin[0], r.tip_radius):
                            wake_sources.append((np.array(r.origin[0]), nodes_body, bool(r.clockwise_rotation)))

    for network in geometry.networks:
        for propulsor in network.propulsors:

            if type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turbofan() or type(propulsor) == RCAIDE.Library.Components.Powertrain.Propulsors.Turbojet():
       
                GEOM = generate_3d_propulsor_points(propulsor, tessellation)
                plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=propulsor_rgb_color, opacity=propulsor_opacity)
                
            # if nacelle geometry is defined, plot nacelle
            if propulsor.nacelle != None:
                if type(propulsor.nacelle) == RCAIDE.Library.Components.Nacelles.Stack_Nacelle:
                    GEOM = generate_3d_stack_nacelle_points(propulsor.nacelle, tessellation=tessellation, number_of_airfoil_points=number_of_airfoil_points)
                elif type(propulsor.nacelle) == RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle:
                    GEOM = generate_3d_BOR_nacelle_points(propulsor.nacelle, tessellation=tessellation, number_of_airfoil_points=number_of_airfoil_points)
                else:
                    GEOM = generate_3d_basic_nacelle_points(propulsor.nacelle, tessellation=tessellation, number_of_airfoil_points=number_of_airfoil_points)
                plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=nacelle_rgb_color, opacity=nacelle_opacity)

            if 'rotor' in propulsor:
                rot   = propulsor.rotor
                rot_x = rot.orientation_euler_angles[0]
                rot_y = rot.orientation_euler_angles[1]
                rot_z = rot.orientation_euler_angles[2]
                num_B = int(rot.number_of_blades)
                if (rot.radius_distribution) is None or (plot_actuator_disc == True):
                    make_actuator_disc(plotter, rot.hub_radius, rot.tip_radius, rot.origin, rot_x, rot_y, rot_z, rotor_rgb_color, rotor_opacity)
                else:
                    rot_y += np.pi / 2
                    dim = len(rot.radius_distribution)
                    for i in range(num_B):
                        GEOM = generate_3d_blade_points(rot, number_of_airfoil_points, dim, i)
                        plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=rotor_rgb_color, opacity=rotor_opacity)
                if plot_wake:
                    wake_source = find_wake_source(rot.origin[0], wake_sources)
                    add_rotor_wake(plotter, rot, wake_control_point, wake_color, wake_opacity,
                                   wake_tube_radius, wake_stride, wake_source=wake_source)

            if 'propeller' in propulsor:
                prop  = propulsor.propeller
                rot_x = prop.orientation_euler_angles[0]
                rot_y = prop.orientation_euler_angles[1]
                rot_z = prop.orientation_euler_angles[2]
                num_B = int(prop.number_of_blades)
                if (prop.radius_distribution is None) or (plot_actuator_disc == True):
                    make_actuator_disc(plotter, prop.hub_radius, prop.tip_radius, prop.origin, rot_x, rot_y, rot_z, rotor_rgb_color, rotor_opacity)
                else:
                    dim = len(prop.radius_distribution)
                    for i in range(num_B):
                        GEOM = generate_3d_blade_points(prop, number_of_airfoil_points, dim, i)
                        plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=rotor_rgb_color, opacity=rotor_opacity)
                if plot_wake:
                    wake_source = find_wake_source(prop.origin[0], wake_sources)
                    add_rotor_wake(plotter, prop, wake_control_point, wake_color, wake_opacity,
                                   wake_tube_radius, wake_stride, wake_source=wake_source)

        for fuel_line in network.fuel_lines:
            for fuel_tank in fuel_line.fuel_tanks:
                if fuel_tank.wing_tag is not None:
                    wing = geometry.wings[fuel_tank.wing_tag]
                    if issubclass(type(fuel_tank), RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank):
                        if issubclass(type(fuel_tank), RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank) and fuel_tank.geometry_type == 'conformal' and fuel_tank.bwb_aft_tank:
                            seg_bounds = fuel_tank.aft_tank_root_chord_bounds
                            GEOM       = generate_aft_integral_wing_tank_points(wing, 5, seg_bounds, fuel_tank)
                            plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=fuel_tank_rgb_color, opacity=fuel_tank_opacity)
                        elif issubclass(type(fuel_tank), RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Liquid_Hydrogen_Tank) and fuel_tank.geometry_type == 'conformal':
                            seg_bounds = fuel_tank.segments_bounding_tank
                            GEOM       = generate_integral_wing_tank_points(wing, number_of_airfoil_points, seg_bounds, fuel_tank)
                            plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=fuel_tank_rgb_color, opacity=fuel_tank_opacity)
                            if wing.xz_plane_symmetric:
                                GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1]
                                plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=fuel_tank_rgb_color, opacity=fuel_tank_opacity)
                        else:
                            GEOM = generate_non_integral_fuel_tank_points(fuel_tank, tessellation)
                            plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=fuel_tank_rgb_color, opacity=fuel_tank_opacity)
                            if wing.xz_plane_symmetric:
                                GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1]
                                plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=fuel_tank_rgb_color, opacity=fuel_tank_opacity)

                    if type(fuel_tank) == RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank:
                        seg_bounds = fuel_tank.segments_bounding_tank
                        GEOM       = generate_integral_wing_tank_points(wing, number_of_airfoil_points, seg_bounds, fuel_tank, plot_centerline=False)
                        plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=fuel_tank_rgb_color, opacity=fuel_tank_opacity)
                        if wing.xz_plane_symmetric:
                            GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1]
                            plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=fuel_tank_rgb_color, opacity=fuel_tank_opacity)

                elif fuel_tank.fuselage_tag is not None:
                    fuselage = geometry.fuselages[fuel_tank.fuselage_tag]
                    if type(fuel_tank) == RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Integral_Tank:
                        seg_bounds = fuel_tank.segments_bounding_tank
                        GEOM = generate_integral_fuel_tank_points(fuselage, fuel_tank, seg_bounds, tessellation)
                        plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=fuel_tank_rgb_color, opacity=fuel_tank_opacity)

                elif issubclass(type(fuel_tank), RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Non_Integral_Tank):
                    GEOM = generate_non_integral_fuel_tank_points(fuel_tank, tessellation)
                    plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=fuel_tank_rgb_color, opacity=fuel_tank_opacity)
                    if wing.xz_plane_symmetric:
                        GEOM.PTS[:, :, 1] = -GEOM.PTS[:, :, 1]
                        plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=fuel_tank_rgb_color, opacity=fuel_tank_opacity)

        for bus in network.busses:
            for battery in bus.battery_modules:
                GEOM = generate_3d_cuboid_points(battery)
                plotter.add_mesh(generate_vtk_object(GEOM.PTS), color=battery_rgb_color, opacity=battery_opacity)
    
    if front_view:
        plotter.camera_position = [(-2 * L , 0, 0), (0, 0,0), (0, 0, 1)] 
    elif side_view:
        plotter.camera_position = [(L /2 , 2 * L, 0), (L /4, 0, 0), (0, 0, 1)]  
    elif top_view:
        plotter.camera_position = [(L, 0 , 2 * L ), (L/4, 0,0), (0, 0, 1)]       
    else:
        plotter.camera_position = [(L * camera_eye_x, L * camera_eye_y, L * camera_eye_z), (L /2, 0, 0), (0, 0, 1)]
    
    plotter.window_size = [1500, 1500]
    plotter.set_background('white')

    if export_gltf:
        plotter.export_gltf(save_filename + ".gltf")

    if save_figure:
        plotter.screenshot(save_filename + ".png", transparent_background=True)
    else:
        if show_figure:
            plotter.show()
    return plotter

def add_lopa_seats(plotter, lopa_geometry, opacity):
    color_map = {
        "first":      mcolors.to_rgb("indianred"),
        "business":   mcolors.to_rgb("seagreen"),
        "economy":    mcolors.to_rgb("steelblue"),
        "galley_lav": mcolors.to_rgb("sandybrown"),
    }

    # Fast path: batched merged meshes (one add_mesh call per class/emergency group).
    batches = getattr(lopa_geometry, "_lopa_batches", {})
    if batches:
        for (seat_class, is_em), merged_mesh in batches.items():
            rgb = color_map.get(seat_class, mcolors.to_rgb("gray"))
            actor = plotter.add_mesh(merged_mesh, color=rgb, opacity=float(opacity),
                                     show_scalar_bar=False)
            if is_em:
                actor.GetProperty().EdgeVisibilityOn()
                actor.GetProperty().SetEdgeColor(*rgb)
                actor.GetProperty().SetLineWidth(1.0)
        return

    # Legacy path: individual seat dicts produced by older generate_3d_lopa_points.
    seats = getattr(lopa_geometry, "_lopa_seats", [])
    for seat in seats:
        poly = seat.get("polydata", None)
        if poly is None:
            continue
        seat_class = seat.get("class", "economy")
        rgb = color_map.get(seat_class, mcolors.to_rgb("gray"))
        mesh = pv.wrap(poly)
        actor = plotter.add_mesh(mesh, color=rgb, opacity=float(opacity), show_scalar_bar=False)
        if seat.get("emergency_row", False):
            actor.GetProperty().EdgeVisibilityOn()
            actor.GetProperty().SetEdgeColor(*rgb)
            actor.GetProperty().SetLineWidth(1.0)

def _wake_matches_origin(nodes_body, origin, tip_radius, tol_factor=0.5):
    """True if this wake's node nearest the rotor disk (wake age 0, the shed point) sits at
    roughly `tip_radius` from `origin` -- where a genuinely fresh, correctly-positioned wake's
    shed point actually is by construction (r_R_shed * tip_radius from the hub, and r_R_shed
    defaults to 1.0) -- rather than merely "somewhere within a loose distance ceiling."

    A rotor's wake data can be real (non-None) yet centered on the *wrong* origin entirely.
    design_electric_rotor's sea-level-static evaluation (design_electric_rotor.py) calls
    compute_performance() on the shared propulsor template *before* a vehicle's per-propulsor
    origin loop has assigned it a real origin -- at that point rotor.origin is still the
    [[0,0,0]] default. For an "identical propulsors" network, only one propulsor ever gets a
    real solve *during the mission itself* (see network.identical_propulsors); the rest keep
    whatever wake data survived from that shared design-time template (deepcopied into each
    propulsor, each later given its own real origin, but the already-computed wake array is
    never recomputed for it) -- non-None, but still centered on the stale [0,0,0]-ish origin,
    not this rotor's real one.

    A loose "distance < N*tip_radius" ceiling isn't tight enough: an inboard rotor whose real
    origin happens to sit within a few tip radii of [0,0,0] (e.g. a rotor mounted close to the
    fuselage centerline) can have its stale, wrongly-centered wake pass a loose check simply
    because the two origins are coincidentally close -- observed directly: a stale wake ~2.57m
    from a rotor's real origin passed a 3*tip_radius=~4.28m ceiling. Checking that the shed
    point sits at ~tip_radius (not "< some multiple of it") catches this, since the stale wake's
    distance from the real origin has no reason to land near tip_radius specifically.
    """
    near_disc = nodes_body[0]   # (B, 3) -- wake age 0, right at the rotor disk / shed point
    dist      = np.linalg.norm(near_disc - np.array(origin), axis=-1)
    tol       = max(tol_factor * float(tip_radius), 0.15)
    return bool(np.all(np.abs(dist - float(tip_radius)) < tol))


def find_wake_source(origin, wake_sources, tol=1e-3):
    """Finds real wake data to reuse for a rotor that has none of its own (e.g. an "identical
    propulsor" that only had its conditions reused, not actually re-solved -- see
    network.identical_propulsors).

    Prefers this rotor's exact XZ-plane mirror-image origin (same x, same z, opposite y),
    within `tol` meters, if one exists with real wake data -- a reasonable tie-breaker when
    several sources are available. Otherwise falls back to *any* other rotor's real wake data
    (there is normally exactly one, the single real solve under network.identical_propulsors).
    Either way, add_rotor_wake decides whether to actually mirror the shape based on the two
    rotors' rotation sense (clockwise_rotation), not on which side of the vehicle they're on --
    see add_rotor_wake's docstring. Without the "any source" fallback, a vehicle with more than
    one spanwise pair of identical propulsors (e.g. front/outboard/rear rotor pairs) would only
    ever get a wake drawn on the one pair that happens to exactly mirror the single real solve --
    every other pair would silently get no wake at all.

    `wake_sources` is a list of (origin, nodes_body, clockwise_rotation) triples -- see the
    pre-pass in plot_3d_vehicle. Returns (source_origin, source_nodes_body,
    source_clockwise_rotation), or None if wake_sources is empty.
    """
    origin = np.array(origin)
    mirror_target = np.array([origin[0], -origin[1], origin[2]])
    for src_origin, src_nodes, src_cw in wake_sources:
        if np.linalg.norm(src_origin - mirror_target) < tol:
            return src_origin, src_nodes, src_cw
    if wake_sources:
        return wake_sources[0]
    return None


def add_rotor_wake(plotter, rot, control_point, color, opacity, tube_radius, stride=1, wake_source=None):
    """Overlays a rotor's prescribed tip-vortex wake (one polyline per blade) on the plotter.

    Reads `rot.blades.wake.nodes_body`, shape (ctrl_pts, N_wake+1, B, 3), populated by
    RCAIDE.Library.Methods.Powertrain.Converters.Rotor.Performance.Lifting_Line_Theory.
    initialize_wake_geometry after a mission has been evaluated. If this rotor has no wake data
    of its own (e.g. it's an "identical propulsor" that only had its conditions reused, not
    actually re-solved -- see network.identical_propulsors), falls back to `wake_source` (found
    via find_wake_source): an (origin, nodes_body, clockwise_rotation) triple belonging to
    another rotor with a real solve. The wake shape is re-centered on that source rotor's own
    origin, then translated to this rotor's origin -- mirrored in y first only if this rotor's
    own rotation sense (rot.clockwise_rotation) differs from the source's, since only a genuinely
    counter-rotating rotor has a mirror-image wake; two rotors spinning the same direction (the
    common case -- nothing in this vehicle's setup necessarily varies clockwise_rotation by side)
    have the *same* wake shape, just at a different origin, and mirroring it would flip the
    spiral's handedness backwards (observed: a same-direction rotor's "mirrored" wake spiralling
    the wrong way and cutting back into the fuselage instead of trailing cleanly away).
    """
    nodes_body = None
    blades = rot.get('blades', None)
    if blades is not None:
        wake = blades.get('wake', None)
        if wake is not None and wake.get('nodes_body', None) is not None:
            all_cp = wake.nodes_body   # (ctrl_pts, N_wake+1, B, 3)
            if control_point < all_cp.shape[0]:
                own_nodes = all_cp[control_point]   # (N_wake+1, B, 3)
                # non-None doesn't mean *this rotor's* real solve -- see _wake_matches_origin
                if _wake_matches_origin(own_nodes, rot.origin[0], rot.tip_radius):
                    nodes_body = own_nodes

    if nodes_body is None and wake_source is not None:
        src_origin, src_nodes, src_cw = wake_source
        target_origin = np.array(rot.origin[0])
        local = src_nodes - src_origin   # hub-centered wake shape, source rotor's own frame
        if bool(rot.clockwise_rotation) != bool(src_cw):
            local = local.copy()
            local[:, :, 1] = -local[:, :, 1]   # genuinely counter-rotating -- mirror handedness
        nodes_body = local + target_origin

    if nodes_body is None:
        return

    B = nodes_body.shape[1]
    for b in range(B):
        filament_pts = nodes_body[::stride, b, :]
        # keep the true tip point even if the stride skips past it
        if not np.array_equal(filament_pts[-1], nodes_body[-1, b, :]):
            filament_pts = np.vstack([filament_pts, nodes_body[-1:, b, :]])
        line = pv.lines_from_points(filament_pts)
        if tube_radius is not None:
            line = line.tube(radius=tube_radius, n_sides=24)
        plotter.add_mesh(line, color=color, opacity=opacity)
    return


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
    
 