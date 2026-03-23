# RCAIDE/Library/Plots/Mass_Properties/plot_center_of_gravity_drift.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------  
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style 
from RCAIDE.Library.Plots.Geometry import plot_3d_vehicle
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np 
import pyvista as pv

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------    
def plot_aircraft_cg_weight_bubbles(results,vehicle0,
                             save_figure = False,
                             show_legend = False,
                             save_filename = "Bubble Plot" ,
                             file_type = ".png", 
                             width = 11, height = 7,
                             bubble_scale = 1.2,
                             show_figure = True):
    vehicle = results.segments[0].analyses.vehicle
    cg_df = vehicle.mass_properties.center_of_gravity_breakdown
    if cg_df is None or len(cg_df) == 0:
        raise ValueError("center_of_gravity_breakdown is empty. Run mass properties CG preprocessing first.")

    oew_mass = float(vehicle.mass_properties.operating_empty)
    if oew_mass <= 0.0:
        raise ValueError("vehicle.mass_properties.operating_empty must be > 0 for bubble scaling.")
    if bubble_scale <= 0.0:
        raise ValueError("bubble_scale must be > 0.")

    span = max([wing.spans.projected for wing in vehicle.wings], default=30.0)
    min_radius = 0.004 * span
    max_radius = 0.030 * span

    plotter = plot_3d_vehicle(
        vehicle0,
        wing_opacity=0.2,
        fuselage_opacity=0.2,
        boom_opacity=0.0,
        nacelle_opacity=0.2,
        fuel_tank_opacity=0.1,
        lopa_opacity=0.1,
        rotor_opacity=0.0,
        cargo_bay_opacity=0.05,
        show_figure=False,
    )

    for _, row in cg_df.iterrows():
        mass = float(row["Mass (kg)"])
        x_cg = float(row["CG x (m)"])
        y_cg = float(row["CG y (m)"])
        z_cg = float(row["CG z (m)"])
        component = str(row["Component"])

        ratio = np.clip(mass / oew_mass, 0.0, 1.0)
        radius = (min_radius + (max_radius - min_radius) * np.sqrt(ratio)) * bubble_scale

        if component.lower() == "operating_empty":
            bubble_color = (0.85, 0.15, 0.15)
            opacity = 0.95
            radius *= 1.2
        else:
            bubble_color = cm.viridis(ratio)[:3]
            opacity = 0.80

        bubble = pv.Sphere(radius=radius, center=(x_cg, y_cg, z_cg), theta_resolution=20, phi_resolution=20)
        plotter.add_mesh(bubble, color=bubble_color, opacity=opacity, smooth_shading=True)

    if show_legend:
        plotter.add_legend(
            labels=[
                ["Operating Empty CG", (0.85, 0.15, 0.15)],
                ["Other Components (mass-scaled)", cm.viridis(0.6)[:3]],
            ],
            bcolor="white",
        )

    plotter.window_size = [int(width * 150), int(height * 150)]
    if save_figure:
        plotter.screenshot(save_filename + file_type)
    if show_figure:
        plotter.show()

    return
