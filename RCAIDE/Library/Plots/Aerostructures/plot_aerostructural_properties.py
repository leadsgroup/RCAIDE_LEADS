# RCAIDE/Library/Plots/Aerostructures/plot_aerostructural_properties.py
#
# Created: Jul 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORTS
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style, segment_colors

import matplotlib.pyplot as plt
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  plot_aerostructural_properties
# ----------------------------------------------------------------------------------------------------------------------
def plot_aerostructural_properties(results,
                                   wing_tag      = 'main_wing',
                                   save_figure   = False,
                                   show_legend   = True,
                                   save_filename = 'Aerostructural_Properties',
                                   file_type     = '.png',
                                   width         = 11,
                                   height        = 7):
    """Plot wing aerostructural properties over the mission timeline.

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

        tip_def_z_mm  =  struct.deflection[:, -1, 2] * 1e3
        tip_def_x_mm  =  struct.deflection[:, -1, 0] * 1e3
        tip_twist_deg =  np.degrees(struct.elastic_twist[:, -1, 0])
        max_def_mm    =  np.max(np.abs(struct.deflection[:, :, 2]), axis=1) * 1e3

        axis_1.plot(time, tip_def_z_mm,  color=line_colors[i], marker=ps.markers[0],
                    linewidth=ps.line_width, label=segment_name)
        axis_2.plot(time, tip_twist_deg, color=line_colors[i], marker=ps.markers[0],
                    linewidth=ps.line_width)
        axis_3.plot(time, max_def_mm,    color=line_colors[i], marker=ps.markers[0],
                    linewidth=ps.line_width)
        axis_4.plot(time, tip_def_x_mm,  color=line_colors[i], marker=ps.markers[0],
                    linewidth=ps.line_width)

    axis_1.set_ylabel(r'Tip Z-Deflection (mm)')
    set_axes(axis_1)

    axis_2.set_ylabel(r'Tip Elastic Twist (deg)')
    set_axes(axis_2)

    axis_3.set_xlabel('Time (mins)')
    axis_3.set_ylabel(r'Max Z-Deflection (mm)')
    set_axes(axis_3)

    axis_4.set_xlabel('Time (mins)')
    axis_4.set_ylabel(r'Tip X-Deflection (mm)')
    set_axes(axis_4)

    if show_legend:
        leg = fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=4)
        leg.set_title('Flight Segment', prop={'size': ps.legend_font_size, 'weight': 'heavy'})

    fig.tight_layout()
    fig.subplots_adjust(top=0.75)

    title_text = 'Aerostructural Properties — ' + wing_tag.replace('_', ' ').title()
    fig.suptitle(title_text)

    if save_figure:
        plt.savefig(save_filename + file_type)

    return fig
