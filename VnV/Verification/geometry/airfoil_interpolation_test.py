# airfoil_interpolation_test.py
#
# Created:  March 2021, R. Erhard
# Modified: June 2026, M. Clarke

from RCAIDE.Library.Methods.Geometry.Airfoil.generate_interpolated_airfoils import generate_interpolated_airfoils
from RCAIDE.Library.Methods.Geometry.Airfoil.import_airfoil_geometry         import import_airfoil_geometry
from RCAIDE.Library.Plots.Common                                              import plot_style, segment_colors
import os
import numpy as np
import matplotlib.pyplot as plt
import time


def main():
    ti = time.time()

    airfoils_path = os.path.abspath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "Vehicles", "Airfoils")
    ) + os.path.sep
    a_labels  = ["Clark_y", "E63"]
    nairfoils = 4   # number of total airfoils (includes a1 and a2 endpoints)

    a1            = airfoils_path + a_labels[0] + ".txt"
    a2            = airfoils_path + a_labels[1] + ".txt"
    airfoil_files = generate_interpolated_airfoils(a1, a2, nairfoils, npoints=101, save_filename="Transition")

    # import all airfoil geometries (endpoints + interpolated)
    geo = [import_airfoil_geometry(f, npoints=101) for f in airfoil_files]

    # regression check: interpolated airfoils match saved truth values
    airfoil_data_1   = geo[1]
    airfoil_data_2   = geo[2]
    test_dir         = os.path.dirname(os.path.abspath(__file__))
    airfoil_data_1_r = import_airfoil_geometry(os.path.join(test_dir, "Transition1_regression.txt"), npoints=101)
    airfoil_data_2_r = import_airfoil_geometry(os.path.join(test_dir, "Transition2_regression.txt"), npoints=101)

    assert max(abs(airfoil_data_1.x_coordinates - airfoil_data_1_r.x_coordinates)) < 1e-5
    assert max(abs(airfoil_data_2.x_coordinates - airfoil_data_2_r.x_coordinates)) < 1e-5
    assert max(abs(airfoil_data_1.y_coordinates - airfoil_data_1_r.y_coordinates)) < 1e-5
    assert max(abs(airfoil_data_2.y_coordinates - airfoil_data_2_r.y_coordinates)) < 1e-5

    # monotonicity check: t/c and max thickness must vary monotonically between endpoints
    t_c_vals  = np.array([g.thickness_to_chord for g in geo])
    t_max_vals = np.array([g.max_thickness      for g in geo])
    for vals, label in [(t_c_vals, "t/c"), (t_max_vals, "max thickness")]:
        lo, hi = min(vals[0], vals[-1]), max(vals[0], vals[-1])
        assert np.all(vals >= lo - 1e-6) and np.all(vals <= hi + 1e-6), \
            f"Interpolated {label} values fall outside endpoint bounds: {vals}"

    # boundary check: interpolated surface coordinates must lie between the endpoint envelopes.
    # x_upper/lower_surface are linearly interpolated so they are guaranteed bounded; x_coordinates
    # come from splprep reparameterisation and are not pointwise-comparable across shapes.
    xu1, xu2 = geo[0].x_upper_surface, geo[-1].x_upper_surface
    yu1, yu2 = geo[0].y_upper_surface, geo[-1].y_upper_surface
    xl1, xl2 = geo[0].x_lower_surface, geo[-1].x_lower_surface
    yl1, yl2 = geo[0].y_lower_surface, geo[-1].y_lower_surface
    tol = 1e-4
    for i, g in enumerate(geo[1:-1], start=1):
        assert np.all(g.x_upper_surface >= np.minimum(xu1, xu2) - tol) and \
               np.all(g.x_upper_surface <= np.maximum(xu1, xu2) + tol), \
               f"Transition {i} x_upper_surface out of endpoint bounds"
        assert np.all(g.y_upper_surface >= np.minimum(yu1, yu2) - tol) and \
               np.all(g.y_upper_surface <= np.maximum(yu1, yu2) + tol), \
               f"Transition {i} y_upper_surface out of endpoint bounds"
        assert np.all(g.x_lower_surface >= np.minimum(xl1, xl2) - tol) and \
               np.all(g.x_lower_surface <= np.maximum(xl1, xl2) + tol), \
               f"Transition {i} x_lower_surface out of endpoint bounds"
        assert np.all(g.y_lower_surface >= np.minimum(yl1, yl2) - tol) and \
               np.all(g.y_lower_surface <= np.maximum(yl1, yl2) + tol), \
               f"Transition {i} y_lower_surface out of endpoint bounds"

    plot_airfoil_transition(airfoil_files, geo, a_labels, nairfoils)


    elapsed_time = time.time() - ti
    elapsed_time_min = elapsed_time / 60
    print('Elapsed time (min): ', elapsed_time_min)
    return


def plot_airfoil_transition(airfoil_files, geo, a_labels, nairfoils):
    """Plot all airfoils overlaid with a color gradient and a t/c progression panel."""

    ps     = plot_style()
    colors = segment_colors(nairfoils)

    plt.rcParams.update({
        'axes.labelsize':  ps.axis_font_size,
        'xtick.labelsize': ps.axis_font_size,
        'ytick.labelsize': ps.axis_font_size,
        'axes.titlesize':  ps.title_font_size,
    })

    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(f"Airfoil Transition: {a_labels[0]} → {a_labels[1]}", fontsize=ps.title_font_size)

    # left panel: overlaid airfoil shapes
    ax_shape = axes[0]
    labels = [a_labels[0]] + [f"Transition {k}" for k in range(1, nairfoils - 1)] + [a_labels[1]]
    for k, (g, label, c) in enumerate(zip(geo, labels, colors)):
        lw   = ps.line_width + 1 if k in (0, nairfoils - 1) else ps.line_width
        ls   = '-' if k in (0, nairfoils - 1) else '--'
        ax_shape.plot(g.x_coordinates, g.y_coordinates,
                      color=c, linewidth=lw, linestyle=ls, label=label)

    ax_shape.set_xlabel("x/c")
    ax_shape.set_ylabel("y/c")
    ax_shape.set_title("Airfoil Profiles")
    ax_shape.set_aspect('equal')
    ax_shape.legend(fontsize=ps.axis_font_size - 2)
    ax_shape.grid(True, alpha=0.3)

    # right panel: t/c and max thickness vs z (interpolation parameter)
    ax_prop = axes[1]
    z = np.linspace(0, 1, nairfoils)
    t_c_vals   = np.array([g.thickness_to_chord for g in geo])
    t_max_vals = np.array([g.max_thickness      for g in geo])

    ax_prop.plot(z, t_c_vals,   color=colors[0],          linewidth=ps.line_width,
                 marker=ps.markers[0], label="t/c")
    ax_prop.plot(z, t_max_vals, color=colors[nairfoils-1], linewidth=ps.line_width,
                 marker=ps.markers[1], label="max thickness")
    ax_prop.set_xlabel("Interpolation parameter z")
    ax_prop.set_ylabel("Normalized value")
    ax_prop.set_title("Thickness Progression")
    ax_prop.legend(fontsize=ps.axis_font_size - 2)
    ax_prop.grid(True, alpha=0.3)
    ax_prop.set_xlim(-0.05, 1.05)

    plt.tight_layout()

    return fig


if __name__ == "__main__":
    main()
    plt.show()
