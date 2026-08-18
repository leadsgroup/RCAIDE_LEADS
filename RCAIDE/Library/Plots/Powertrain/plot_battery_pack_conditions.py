# RCAIDE/Library/Plots/Performance/plot_battery_pack_conditions.py
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
def plot_battery_pack_conditions(results,
                                  save_figure=False,
                                  show_legend=True,
                                  save_filename="Battery_Pack_Conditions",
                                  file_type=".png",
                                  width=11, height=7):
    """
    Creates a six-panel plot showing battery pack-level conditions throughout flight.

    Parameters
    ----------
    results : Results
        RCAIDE results structure containing segment data and battery pack conditions

    save_figure : bool, optional
        Flag for saving the figure (default: False)

    show_legend : bool, optional
        Flag for displaying plot legend (default: True)

    save_filename : str, optional
        Base name of file for saved figure (default: "Battery_Pack_Conditions")

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

    fig, axes = plt.subplots(3, 2, figsize=(width, height))
    fig.canvas.manager.set_window_title(save_filename)
    ax_soc, ax_energy = axes[0]
    ax_current, ax_power = axes[1]
    ax_voltage, ax_temp = axes[2]

    line_colors = segment_colors(len(results.segments))

    for network in results.segments[0].analyses.vehicle.networks:
        for b_i, source in enumerate(network.sources):
            if not isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack):
                continue

            battery = source

            for i, segment in enumerate(results.segments):
                time = segment.conditions.frames.inertial.time[:, 0] / Units.min
                battery_conditions = segment.conditions.energy.sources[battery.tag]

                pack_power   = battery_conditions.outputs.power.electrical[:, 0]
                pack_energy  = battery_conditions.energy[:, 0]
                pack_SOC     = battery_conditions.state_of_charge[:, 0]
                pack_volts   = battery_conditions.voltage_under_load[:, 0]
                pack_current = battery_conditions.current_draw[:, 0]
                pack_temp    = battery_conditions.temperature[:, 0]

                label = battery.tag if (b_i == 0 and i == 0) else None

                ax_soc.plot(time, pack_SOC, color=line_colors[i], marker=ps.markers[b_i],
                            linewidth=ps.line_width, label=label)
                ax_energy.plot(time, pack_energy / 1000 / Units.Wh, color=line_colors[i],
                               marker=ps.markers[b_i], linewidth=ps.line_width)
                ax_current.plot(time, pack_current / 1000, color=line_colors[i],
                                marker=ps.markers[b_i], linewidth=ps.line_width)
                ax_power.plot(time, pack_power / 1000, color=line_colors[i],
                              marker=ps.markers[b_i], linewidth=ps.line_width)
                ax_voltage.plot(time, pack_volts / 1000, color=line_colors[i],
                                marker=ps.markers[b_i], linewidth=ps.line_width)
                ax_temp.plot(time, pack_temp, color=line_colors[i],
                             marker=ps.markers[b_i], linewidth=ps.line_width)

    ax_soc.set_ylabel('SOC')
    ax_soc.set_ylim([0, 1.1])
    ax_energy.set_ylabel('Energy (kW-hr)')
    ax_current.set_ylabel('Current (kA)')
    ax_power.set_ylabel('Power (kW)')
    ax_voltage.set_ylabel('Voltage (kV)')
    ax_voltage.set_xlabel('Time (min)')
    ax_temp.set_ylabel(r'Temperature ($\degree$C)')
    ax_temp.set_xlabel('Time (min)')

    for ax in axes.flat:
        set_axes(ax)

    if show_legend:
        fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=4)

    fig.suptitle('Battery Pack Conditions')
    fig.tight_layout()
    fig.subplots_adjust(top=0.88)

    if save_figure:
        fig.savefig(save_filename + file_type)

    return fig
