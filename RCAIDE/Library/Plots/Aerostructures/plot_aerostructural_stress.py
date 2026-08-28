# RCAIDE/Library/Plots/Aerostructures/plot_aerostructural_stress.py
#
# Created: Aug 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style, segment_colors

import matplotlib.pyplot as plt
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  plot_aerostructural_stress
# ----------------------------------------------------------------------------------------------------------------------
def plot_aerostructural_stress(results,
                               wing_tag      = 'main_wing',
                               save_figure   = False,
                               show_legend   = True,
                               save_filename = 'Aerostructural_Stress',
                               file_type     = '.png',
                               width         = 11,
                               height        = 7):
    """Plot wing structural stress and safety margin over the mission timeline.

    Normal stress is the combined axial + bending stress (conservative corner
    combination); shear stress is the torsional (Bredt-Batho) shear at the
    thinnest wingbox wall; margin of safety is min(yield/normal, yield_shear/shear) - 1,
    all computed in FEA.py from the recovered element internal loads.

    Parameters
    ----------
    results   : RCAIDE mission results
    wing_tag  : tag of the wing to extract (default 'main_wing')
    """

    ps = plot_style()
    parameters = {'axes.labelsize':  ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize':  ps.title_font_size}
    plt.rcParams.update(parameters)

    line_colors = segment_colors(len(results.segments))

    fig    = plt.figure(save_filename)
    fig.set_size_inches(width, height)
    axis_1 = fig.add_subplot(2, 2, 1)
    axis_2 = fig.add_subplot(2, 2, 2)
    axis_3 = fig.add_subplot(2, 2, 3)
    axis_4 = fig.add_subplot(2, 2, 4)

    for i in range(len(results.segments)):
        conds        = results.segments[i].conditions
        time         = conds.frames.inertial.time[:, 0] / Units.min
        segment_name = results.segments[i].tag.replace('_', ' ')

        struct = conds.aerostructures[wing_tag]

        max_normal_stress_mpa = np.max(struct.normal_stress, axis=1) / 1e6
        max_shear_stress_mpa  = np.max(struct.shear_stress,  axis=1) / 1e6
        min_margin_of_safety  = np.min(struct.margin_of_safety, axis=1)
        root_normal_stress_mpa = struct.normal_stress[:, 0] / 1e6

        axis_1.plot(time, max_normal_stress_mpa, color=line_colors[i], marker=ps.markers[0],
                    linewidth=ps.line_width, label=segment_name)
        axis_2.plot(time, max_shear_stress_mpa,  color=line_colors[i], marker=ps.markers[0],
                    linewidth=ps.line_width)
        axis_3.plot(time, min_margin_of_safety,  color=line_colors[i], marker=ps.markers[0],
                    linewidth=ps.line_width)
        axis_4.plot(time, root_normal_stress_mpa, color=line_colors[i], marker=ps.markers[0],
                    linewidth=ps.line_width)

    axis_1.set_ylabel(r'Max Normal Stress (MPa)')
    set_axes(axis_1)

    axis_2.set_ylabel(r'Max Torsional Shear Stress (MPa)')
    set_axes(axis_2)

    axis_3.set_xlabel('Time (mins)')
    axis_3.set_ylabel(r'Min Margin of Safety')
    axis_3.axhline(0.0, color='red', linewidth=ps.line_width, linestyle='--')
    set_axes(axis_3)

    axis_4.set_xlabel('Time (mins)')
    axis_4.set_ylabel(r'Root Normal Stress (MPa)')
    set_axes(axis_4)

    if show_legend:
        leg = fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=4)
        leg.set_title('Flight Segment', prop={'size': ps.legend_font_size, 'weight': 'heavy'})

    fig.tight_layout()
    fig.subplots_adjust(top=0.75)

    title_text = 'Aerostructural Stress — ' + wing_tag.replace('_', ' ').title()
    fig.suptitle(title_text)

    if save_figure:
        plt.savefig(save_filename + file_type)

    return fig
