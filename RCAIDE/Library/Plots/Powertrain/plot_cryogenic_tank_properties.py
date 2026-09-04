# RCAIDE/Library/Plots/Powertrain/plot_cryogenic_tank_properties.py
#
#
# Created: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import RCAIDE
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.lines import Line2D
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------
def plot_cryogenic_tank_properties(results,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Cryogenic_Tank_Properties" ,
                             file_type = ".png",
                             width = 11, height = 7):
    """
    Creates one six-panel figure per Cryogenic_Tank source (mass, temperature,
    volume, pressure, vent rate, and boil-off rate), so tanks with very
    different sizes don't get crowded onto shared axes. Each property's axis
    limits are shared across every tank's figure for direct comparison. Line
    color encodes chronological mission progress (see colorbar); solid lines
    are the liquid fuel, dashed lines are the ullage gas.
    """

    # get plotting style
    ps = plot_style()

    parameters = {
        'axes.labelsize': ps.axis_font_size,
        'xtick.labelsize': ps.axis_font_size,
        'ytick.labelsize': ps.axis_font_size,
        'axes.titlesize': ps.title_font_size
    }
    plt.rcParams.update(parameters)

    n_segments  = len(results.segments)
    line_colors = cm.viridis(np.linspace(0, 0.9, n_segments))

    # Collect every tank source once, and every field, per tank, per segment.
    tanks = []
    for network in results.segments[0].analyses.vehicle.networks:
        for source in network.sources:
            if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Cryogenic_Tank):
                tanks.append(source)

    if not tanks:
        return []

    fields = ['fuel_mass', 'ullage_mass', 'fuel_temperature', 'ullage_temperature',
              'fuel_volume', 'ullage_volume', 'pressure', 'vent_rate', 'boil_off_flow_rate']

    data = {tank.tag: {field: [] for field in fields + ['time']} for tank in tanks}
    for segment in results.segments:
        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        for tank in tanks:
            tc = segment.conditions.energy.sources[tank.tag]
            data[tank.tag]['time'].append(time)
            data[tank.tag]['fuel_mass'].append(tc.fuel_mass[:, 0])
            data[tank.tag]['ullage_mass'].append(tc.ullage_mass[:, 0])
            data[tank.tag]['fuel_temperature'].append(tc.fuel_temperature[:, 0])
            data[tank.tag]['ullage_temperature'].append(tc.ullage_temperature[:, 0])
            data[tank.tag]['fuel_volume'].append(tc.fuel_volume[:, 0] / Units.gallons)
            data[tank.tag]['ullage_volume'].append(tc.ullage_volume[:, 0] / Units.gallons)
            data[tank.tag]['pressure'].append(tc.pressure[:, 0])
            data[tank.tag]['vent_rate'].append(tc.vent_rate[:, 0])
            data[tank.tag]['boil_off_flow_rate'].append(tc.boil_off_flow_rate[:, 0])

    # Shared y-limits per property, across all tanks, so tank-to-tank comparisons are fair.
    def limits(field):
        vals = np.concatenate([np.concatenate(data[tank.tag][field]) for tank in tanks])
        pad  = 0.05 * (vals.max() - vals.min() if vals.max() > vals.min() else max(abs(vals.max()), 1.0))
        return vals.min() - pad, vals.max() + pad

    mass_lim   = (min(limits('fuel_mass')[0], limits('ullage_mass')[0]),
                  max(limits('fuel_mass')[1], limits('ullage_mass')[1]))
    temp_lim   = (min(limits('fuel_temperature')[0], limits('ullage_temperature')[0]),
                  max(limits('fuel_temperature')[1], limits('ullage_temperature')[1]))
    vol_lim    = (min(limits('fuel_volume')[0], limits('ullage_volume')[0]),
                  max(limits('fuel_volume')[1], limits('ullage_volume')[1]))
    press_lim  = limits('pressure')
    vent_lim   = limits('vent_rate')
    boil_lim   = limits('boil_off_flow_rate')

    style_handles = [Line2D([0], [0], color='black', linestyle='-',  label='Fuel'),
                      Line2D([0], [0], color='black', linestyle='--', label='Ullage')]

    figures = []
    for tank in tanks:
        fig = plt.figure(f"{save_filename}_{tank.tag}")
        fig.set_size_inches(width, height)

        axis_1 = plt.subplot(3, 2, 1)  # Mass
        axis_2 = plt.subplot(3, 2, 2)  # Temperatures
        axis_3 = plt.subplot(3, 2, 3)  # Volumes
        axis_4 = plt.subplot(3, 2, 4)  # Pressure
        axis_5 = plt.subplot(3, 2, 5)  # Venting
        axis_6 = plt.subplot(3, 2, 6)  # Boiling

        d = data[tank.tag]
        for i in range(n_segments):
            time = d['time'][i]

            axis_1.plot(time, d['fuel_mass'][i],   color=line_colors[i], linewidth=ps.line_width)
            axis_1.plot(time, d['ullage_mass'][i], color=line_colors[i], linewidth=ps.line_width, linestyle='--')

            axis_2.plot(time, d['fuel_temperature'][i],   color=line_colors[i], linewidth=ps.line_width)
            axis_2.plot(time, d['ullage_temperature'][i], color=line_colors[i], linewidth=ps.line_width, linestyle='--')

            axis_3.plot(time, d['fuel_volume'][i],   color=line_colors[i], linewidth=ps.line_width)
            axis_3.plot(time, d['ullage_volume'][i], color=line_colors[i], linewidth=ps.line_width, linestyle='--')

            axis_4.plot(time, d['pressure'][i],           color=line_colors[i], linewidth=ps.line_width)
            axis_5.plot(time, d['vent_rate'][i],           color=line_colors[i], linewidth=ps.line_width)
            axis_6.plot(time, d['boil_off_flow_rate'][i], color=line_colors[i], linewidth=ps.line_width)

        axis_1.set_ylabel("Mass (kg)");                    axis_1.set_ylim(mass_lim)
        axis_2.set_ylabel("Temp. (K)");                    axis_2.set_ylim(temp_lim)
        axis_3.set_ylabel("Volume (gal)");                 axis_3.set_ylim(vol_lim)
        axis_4.set_ylabel("Pressure (Pa)");                axis_4.set_ylim(press_lim)
        axis_5.set_ylabel(r"$\dot{m}_{vent}$ (kg/s)");     axis_5.set_ylim(vent_lim)
        axis_6.set_ylabel(r"$\dot{m}_{boiloff}$ (kg/s)");  axis_6.set_ylim(boil_lim)

        for ax in [axis_1, axis_2, axis_3, axis_4, axis_5, axis_6]:
            ax.set_xlabel("Time (min)")
            set_axes(ax)

        if show_legend:
            fig.legend(handles=style_handles, loc='upper right', ncol=2, bbox_to_anchor=(0.98, 0.985))

        sm = cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(vmin=0, vmax=1))
        sm.set_array([])
        cbar = fig.colorbar(sm, ax=[axis_1, axis_2, axis_3, axis_4, axis_5, axis_6], pad=0.02, fraction=0.03)
        cbar.set_label("Mission Progress")

        fig.suptitle(f"Cryogenic Tank Properties -- {tank.tag}".replace("_", " "))

        if save_figure:
            fig.savefig(f"{save_filename}_{tank.tag}{file_type}")

        figures.append(fig)

    return figures
