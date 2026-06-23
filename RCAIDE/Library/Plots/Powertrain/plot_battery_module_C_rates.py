# RCAIDE/Library/Plots/Performance/plot_battery_module_C_rates.py
#
#
# Created:  Jul 2023, M. Clarke
# Modified: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style, segment_colors
import matplotlib.pyplot as plt
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------
def plot_battery_module_C_rates(results,
                        save_figure=False,
                        show_legend=True,
                        save_filename="Battery_Module_C_Rates",
                        file_type=".png",
                        width=8, height=6):
    """
    Creates a two-panel plot showing instantaneous and nominal C-rates of battery
    modules during operation.

    Parameters
    ----------
    results : Results
        RCAIDE results structure containing segment data and battery conditions

    save_figure : bool, optional
        Flag for saving the figure (default: False)

    show_legend : bool, optional
        Flag for displaying plot legend (default: True)

    save_filename : str, optional
        Base name of file for saved figure (default: "Battery_Module_C_Rates")

    file_type : str, optional
        File extension for saved figure (default: ".png")

    width : float, optional
        Figure width in inches (default: 8)

    height : float, optional
        Figure height in inches (default: 6)

    Returns
    -------
    fig : matplotlib.figure.Figure
    """

    ps = plot_style()
    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)

    fig, (ax_instant, ax_nominal) = plt.subplots(1, 2, figsize=(width, height))
    fig.canvas.manager.set_window_title(save_filename)

    line_colors = segment_colors(len(results.segments))

    for network in results.segments[0].analyses.vehicle.networks:
        for b_i, source in enumerate(network.sources):
            if not isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack):
                continue

            battery = source
            if b_i > 0 and source.identical_sources:
                continue

            for i, segment in enumerate(results.segments):
                time = segment.conditions.frames.inertial.time[:, 0] / Units.min
                battery_conditions = segment.conditions.energy.sources[battery.tag]

                module_energy  = battery_conditions.energy[:, 0]
                module_volts   = battery_conditions.voltage_under_load[:, 0]
                module_current = battery_conditions.current[:, 0]

                module_amp_hr = (module_energy / Units.Wh) / module_volts
                c_instant     = module_current / module_amp_hr
                c_nominal     = module_current / np.max(module_amp_hr)

                label = battery.tag if (i == 0) else None

                ax_instant.plot(time, c_instant, color=line_colors[i], marker=ps.markers[b_i],
                                linewidth=ps.line_width, label=label)
                ax_nominal.plot(time, c_nominal, color=line_colors[i], marker=ps.markers[b_i],
                                linewidth=ps.line_width)

    ax_instant.set_ylabel('Inst. C-Rate (C)')
    ax_instant.set_xlabel('Time (min)')
    ax_nominal.set_ylabel('Nom. C-Rate (C)')
    ax_nominal.set_xlabel('Time (min)')
    set_axes(ax_instant)
    set_axes(ax_nominal)

    if show_legend:
        fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=4)

    fig.suptitle('Battery Module C-Rates')
    fig.tight_layout()
    fig.subplots_adjust(top=0.85)

    if save_figure:
        fig.savefig(save_filename + file_type)

    return fig
