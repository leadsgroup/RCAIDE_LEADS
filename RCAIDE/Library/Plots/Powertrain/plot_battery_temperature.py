# RCAIDE/Library/Plots/Performance/plot_battery_temperature.py
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

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------
def plot_battery_temperature(results,
                                  save_figure=False,
                                  show_legend=True,
                                  save_filename="Battery_Temperature",
                                  file_type=".png",
                                  width=11, height=7):
    """
    Creates a three-panel plot showing battery thermal conditions throughout flight.

    Parameters
    ----------
    results : Results
        RCAIDE results structure containing segment data and battery thermal conditions

    save_figure : bool, optional
        Flag for saving the figure (default: False)

    show_legend : bool, optional
        Flag for displaying plot legend (default: True)

    save_filename : str, optional
        Name of file for saved figure (default: "Battery_Temperature")

    file_type : str, optional
        File extension for saved figure (default: ".png")

    width : float, optional
        Figure width in inches (default: 11)

    height : float, optional
        Figure height in inches (default: 7)

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

    fig, axes = plt.subplots(2, 2, figsize=(width, height))
    fig.canvas.manager.set_window_title(save_filename)
    ax_temp   = axes[0, 0]
    ax_charge = axes[0, 1]
    ax_heat   = axes[1, 0]

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
                module_tag         = list(battery.modules)[0].tag
                module_conditions  = battery_conditions[module_tag]

                cell_temp   = module_conditions.cell.temperature[:, 0]
                cell_charge = module_conditions.cell.charge_throughput[:, 0]
                pack_Q      = battery_conditions.heat_energy_generated[:, 0]

                label = battery.tag if (i == 0) else None

                ax_temp.plot(time, cell_temp, color=line_colors[i], marker=ps.markers[b_i],
                             linewidth=ps.line_width, label=label)
                ax_charge.plot(time, cell_charge, color=line_colors[i], marker=ps.markers[b_i],
                               linewidth=ps.line_width)
                ax_heat.plot(time, pack_Q / 1000, color=line_colors[i], marker=ps.markers[b_i],
                             linewidth=ps.line_width)

    ax_temp.set_ylabel('Temperature (K)')
    ax_charge.set_ylabel('Charge Throughput (Ah)')
    ax_charge.set_xlabel('Time (min)')
    ax_heat.set_ylabel(r'$\dot{Q}_{heat}$ (kW)')
    ax_heat.set_xlabel('Time (min)')
    set_axes(ax_temp)
    set_axes(ax_charge)
    set_axes(ax_heat)

    axes[1, 1].set_visible(False)

    if show_legend:
        fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=4)

    fig.suptitle('Battery Temperature')
    fig.tight_layout()
    fig.subplots_adjust(top=0.88)

    if save_figure:
        fig.savefig(save_filename + file_type)

    return fig
