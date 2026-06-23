# RCAIDE/Library/Plots/Performance/plot_battery_cell_conditions.py
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
def plot_battery_cell_conditions(results,
                                  save_figure=False,
                                  show_legend=True,
                                  save_filename="Battery_Cell_Conditions",
                                  file_type=".png",
                                  width=11, height=7):
    """
    Creates a six-panel plot showing battery cell-level conditions throughout flight.

    Parameters
    ----------
    results : Results
        RCAIDE results structure containing segment data and battery conditions

    save_figure : bool, optional
        Flag for saving the figure (default: False)

    show_legend : bool, optional
        Flag for displaying plot legend (default: True)

    save_filename : str, optional
        Base name of file for saved figure (default: "Battery_Cell_Conditions")

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
            if b_i > 0 and source.identical_sources:
                continue

            for i, segment in enumerate(results.segments):
                time = segment.conditions.frames.inertial.time[:, 0] / Units.min
                battery_conditions = segment.conditions.energy.sources[battery.tag]

                cell_power   = battery_conditions.cell.power[:, 0]
                cell_energy  = battery_conditions.cell.energy[:, 0]
                cell_SOC     = battery_conditions.cell.state_of_charge[:, 0]
                cell_volts   = battery_conditions.cell.voltage_under_load[:, 0]
                cell_current = battery_conditions.cell.current[:, 0]
                cell_temp    = battery_conditions.cell.temperature[:, 0]

                label = battery.tag if (i == 0) else None

                ax_soc.plot(time, cell_SOC, color=line_colors[i], marker=ps.markers[b_i],
                            linewidth=ps.line_width, label=label)
                ax_energy.plot(time, cell_energy / Units.Wh, color=line_colors[i],
                               marker=ps.markers[b_i], linewidth=ps.line_width)
                ax_current.plot(time, cell_current, color=line_colors[i],
                                marker=ps.markers[b_i], linewidth=ps.line_width)
                ax_power.plot(time, cell_power, color=line_colors[i],
                              marker=ps.markers[b_i], linewidth=ps.line_width)
                ax_voltage.plot(time, cell_volts, color=line_colors[i],
                                marker=ps.markers[b_i], linewidth=ps.line_width)
                ax_temp.plot(time, cell_temp, color=line_colors[i],
                             marker=ps.markers[b_i], linewidth=ps.line_width)

    ax_soc.set_ylabel('SOC')
    ax_soc.set_ylim([0, 1.1])
    ax_energy.set_ylabel('Energy (W-hr)')
    ax_current.set_ylabel('Current (A)')
    ax_power.set_ylabel('Power (W)')
    ax_voltage.set_ylabel('Voltage (V)')
    ax_voltage.set_xlabel('Time (min)')
    ax_temp.set_ylabel(r'Temperature ($\degree$C)')
    ax_temp.set_xlabel('Time (min)')

    for ax in axes.flat:
        set_axes(ax)

    if show_legend:
        fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=4)

    fig.suptitle('Battery Cell Conditions')
    fig.tight_layout()
    fig.subplots_adjust(top=0.88)

    if save_figure:
        fig.savefig(save_filename + file_type)

    return fig
