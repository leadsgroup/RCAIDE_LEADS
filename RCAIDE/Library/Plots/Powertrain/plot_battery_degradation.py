# RCAIDE/Library/Plots/Performance/plot_battery_degradation.py
#
#
# Created:  Jul 2023, M. Clarke
# Modified: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------
def plot_battery_degradation(results,
                            save_figure=False,
                            save_filename="Battery_Degradation",
                            file_type=".png",
                            width=11, height=7):
    """
    Creates a six-panel plot showing battery degradation metrics against charge
    throughput and time.

    Parameters
    ----------
    results : Results
        RCAIDE results structure containing segment data and battery degradation metrics

    save_figure : bool, optional
        Flag for saving the figure (default: False)

    save_filename : str, optional
        Base name of file for saved figure (default: "Battery_Degradation")

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

    fig = None

    for network in results.segments[0].analyses.vehicle.networks:
        for b_i, source in enumerate(network.sources):
            if not isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Batteries.Battery_Pack):
                continue

            battery = source
            if b_i > 0 and source.identical_sources:
                continue

            color = cm.tab10(b_i / max(len(network.sources), 1))

            fig, axes = plt.subplots(3, 2, figsize=(width, height))
            fig.canvas.manager.set_window_title(save_filename + '_' + battery.tag)

            num_segs          = len(results.segments)
            time_hrs          = np.zeros(num_segs)
            capacity_fade     = np.zeros(num_segs)
            resistance_growth = np.zeros(num_segs)
            cycle_day         = np.zeros(num_segs)
            charge_throughput = np.zeros(num_segs)

            for i, segment in enumerate(results.segments):
                time_hrs[i]          = segment.conditions.frames.inertial.time[-1, 0] / Units.hour
                battery_conditions   = segment.conditions.energy.sources[battery.tag]
                module_tag           = list(battery.modules)[0].tag
                module_conditions    = battery_conditions[module_tag]
                cycle_day[i]         = module_conditions.cell.cycle_in_day
                capacity_fade[i]     = module_conditions.cell.capacity_fade_factor
                resistance_growth[i] = module_conditions.cell.resistance_growth_factor
                charge_throughput[i] = module_conditions.cell.charge_throughput[-1, 0]

            lw = ps.line_width

            axes[0, 0].plot(charge_throughput, capacity_fade, color=color, marker=ps.markers[0], linewidth=lw, label=battery.tag)
            axes[0, 0].set_ylabel('$E/E_0$')
            axes[0, 0].set_xlabel('Ah')
            set_axes(axes[0, 0])

            axes[1, 0].plot(time_hrs, capacity_fade, color=color, marker=ps.markers[0], linewidth=lw)
            axes[1, 0].set_ylabel('$E/E_0$')
            axes[1, 0].set_xlabel('Time (hrs)')
            set_axes(axes[1, 0])

            axes[2, 0].plot(cycle_day, capacity_fade, color=color, marker=ps.markers[0], linewidth=lw)
            axes[2, 0].set_ylabel('$E/E_0$')
            axes[2, 0].set_xlabel('Time (days)')
            set_axes(axes[2, 0])

            axes[0, 1].plot(charge_throughput, resistance_growth, color=color, marker=ps.markers[0], linewidth=lw)
            axes[0, 1].set_ylabel('$R/R_0$')
            axes[0, 1].set_xlabel('Ah')
            set_axes(axes[0, 1])

            axes[1, 1].plot(time_hrs, resistance_growth, color=color, marker=ps.markers[0], linewidth=lw)
            axes[1, 1].set_ylabel('$R/R_0$')
            axes[1, 1].set_xlabel('Time (hrs)')
            set_axes(axes[1, 1])

            axes[2, 1].plot(cycle_day, resistance_growth, color=color, marker=ps.markers[0], linewidth=lw)
            axes[2, 1].set_ylabel('$R/R_0$')
            axes[2, 1].set_xlabel('Time (days)')
            set_axes(axes[2, 1])

            fig.suptitle('Battery Cell Degradation: ' + battery.tag)
            fig.tight_layout()

            if save_figure:
                fig.savefig(save_filename + '_' + battery.tag + file_type)

    return fig
