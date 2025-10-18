## @ingroup Library-Plots-Energy
# RCAIDE/Library/Plots/Energy/plot_powertrain_power.py
#
# Created:  Oct 2025, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
from RCAIDE.Framework.Core import Units
from RCAIDE.Library.Plots.Common import set_axes, plot_style
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------
## @ingroup Library-Plots-Performance-Energy
def plot_powertrain_power(results,
                          save_figure=False,
                          show_legend=True,
                          save_filename="Powertrain_Power",
                          file_type=".png",
                          width=11, height=7):
    """
    Creates independent plots (6 separate figures) for:
      - Distributors
      - Propulsors
      - Modulators
      - Converters
      - Sources 
      - Systems 

    Returns
    -------
    figs : dict
        Dict of figures keyed by category: 'distributors','propulsors','modulators','converters','sources','systems'
    """
    # get plotting style
    ps = plot_style()

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)

    # color map across segments
    line_colors = cm.inferno(np.linspace(0, 0.9, len(results.segments)))

    def _get_power_arr(bucket):
        # try common field names in priority order
        for path in [
            ("power_draw",),                         # distributors, systems (sometimes)
            ("power",),                              # propulsors, generic
            ("outputs", "power"),
            ("outputs", "dc_real_power"),           
            ("inputs", "power"),
            ("inputs", "ac_real_power"),            
        ]:
            try:
                val = bucket
                for p in path:
                    val = getattr(val, p)
                # ensure numpy array (N,1) if possible
                arr = np.asarray(val)
                if arr.ndim == 1:
                    arr = arr.reshape(-1, 1)
                return arr
            except Exception:
                pass
        # last resort: zeros matching time length
        return np.zeros_like(results.segments[0].conditions.frames.inertial.time[:, 0:1])

    # label helper: make "nice" LaTeX label
    def _label_from_name(name):
        # use LaTeX \mathrm with explicit small spaces between tokens
        return r'$P_{\mathrm{' + name.replace("_", r"\;") + '}}$'

    figs = {}

    # ---------------------------- Distributors ----------------------------
    fig = plt.figure(save_filename + "_Distributors")
    axis_1 = plt.subplot(1, 1, 1)
    fig.set_size_inches(width, height)

    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        dist_dict = results.segments[i].conditions.energy.distributors
        for j, tag in enumerate(dist_dict.keys()):
            distributor = dist_dict[tag]
            power = _get_power_arr(distributor)[:, 0]
            name = getattr(distributor, 'name', tag)
            if i == 0:
                axis_1.plot(time, power, color=line_colors[i], marker=ps.markers[j],
                            linewidth=ps.line_width, label=_label_from_name(name))
            else:
                axis_1.plot(time, power, color=line_colors[i], marker=ps.markers[j],
                            linewidth=ps.line_width)
    set_axes(axis_1)
    axis_1.set_xlabel('Time (mins)')
    axis_1.set_ylabel('Distributors Power (W)')
    if show_legend:
        fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=5)
    fig.tight_layout()
    fig.subplots_adjust(top=0.8)
    fig.suptitle('Distributors Power Consumption')
    if save_figure:
        plt.savefig(save_filename + "_Distributors" + file_type)
    figs['distributors'] = fig

    # ---------------------------- Propulsors ----------------------------
    fig = plt.figure(save_filename + "_Propulsors")
    axis_1 = plt.subplot(1, 1, 1)
    fig.set_size_inches(width, height)

    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        dist_dict = results.segments[i].conditions.energy.propulsors
        for j, tag in enumerate(dist_dict.keys()):
            propulsor = dist_dict[tag]
            # propulsors typically keep "power"
            if hasattr(propulsor, 'power'):
                power = np.asarray(propulsor.power)[:, 0]
            else:
                power = _get_power_arr(propulsor)[:, 0]
            name = getattr(propulsor, 'name', tag)
            if i == 0:
                axis_1.plot(time, power, color=line_colors[i], marker=ps.markers[j],
                            linewidth=ps.line_width, label=_label_from_name(name))
            else:
                axis_1.plot(time, power, color=line_colors[i], marker=ps.markers[j],
                            linewidth=ps.line_width)
    set_axes(axis_1)
    axis_1.set_xlabel('Time (mins)')
    axis_1.set_ylabel('Propulsors Power (W)')
    if show_legend:
        fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=5)
    fig.tight_layout()
    fig.subplots_adjust(top=0.8)
    fig.suptitle('Propulsors Power Consumption')
    if save_figure:
        plt.savefig(save_filename + "_Propulsors" + file_type)
    figs['propulsors'] = fig

    # ---------------------------- Modulators ----------------------------
    fig = plt.figure(save_filename + "_Modulators")
    axis_1 = plt.subplot(1, 1, 1)
    fig.set_size_inches(width, height)

    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        dist_dict = results.segments[i].conditions.energy.modulators
        for j, tag in enumerate(dist_dict.keys()):
            modulator = dist_dict[tag]
            power = _get_power_arr(modulator)
            name = getattr(modulator, 'name', tag)
            if i == 0:
                axis_1.plot(time, power, color=line_colors[i], marker=ps.markers[j],
                            linewidth=ps.line_width, label=_label_from_name(name))
            else:
                axis_1.plot(time, power, color=line_colors[i], marker=ps.markers[j],
                            linewidth=ps.line_width)
    set_axes(axis_1)
    axis_1.set_xlabel('Time (mins)')
    axis_1.set_ylabel('Modulators Power (W)')
    if show_legend:
        fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=5)
    fig.tight_layout()
    fig.subplots_adjust(top=0.8)
    fig.suptitle('Modulators Power Consumption')
    if save_figure:
        plt.savefig(save_filename + "_Modulators" + file_type)
    figs['modulators'] = fig

    # ---------------------------- Converters ----------------------------
    fig = plt.figure(save_filename + "_Converters")
    axis_1 = plt.subplot(1, 1, 1)
    fig.set_size_inches(width, height)

    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        dist_dict = results.segments[i].conditions.energy.converters
        for j, tag in enumerate(dist_dict.keys()):
            converter = dist_dict[tag]
            power = _get_power_arr(converter)[:, 0]
            name = getattr(converter, 'name', tag)
            if i == 0:
                axis_1.plot(time, power, color=line_colors[i], marker=ps.markers[j],
                            linewidth=ps.line_width, label=_label_from_name(name))
            else:
                axis_1.plot(time, power, color=line_colors[i], marker=ps.markers[j],
                            linewidth=ps.line_width)
    set_axes(axis_1)
    axis_1.set_xlabel('Time (mins)')
    axis_1.set_ylabel('Converters Power (W)')
    if show_legend:
        fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=5)
    fig.tight_layout()
    fig.subplots_adjust(top=0.8)
    fig.suptitle('Converters Power Consumption')
    if save_figure:
        plt.savefig(save_filename + "_Converters" + file_type)
    figs['converters'] = fig

    # ---------------------------- Sources ----------------------------
    fig = plt.figure(save_filename + "_Sources")
    axis_1 = plt.subplot(1, 1, 1)
    fig.set_size_inches(width, height)

    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        dist_dict = results.segments[i].conditions.energy.sources
        for j, tag in enumerate(dist_dict.keys()):
            source = dist_dict[tag]
            power = _get_power_arr(source)[:, 0]
            name = getattr(source, 'name', tag)
            if i == 0:
                axis_1.plot(time, power, color=line_colors[i], marker=ps.markers[j],
                            linewidth=ps.line_width, label=_label_from_name(name))
            else:
                axis_1.plot(time, power, color=line_colors[i], marker=ps.markers[j],
                            linewidth=ps.line_width)
    set_axes(axis_1)
    axis_1.set_xlabel('Time (mins)')
    axis_1.set_ylabel('Sources Power (W)')
    if show_legend:
        fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=5)
    fig.tight_layout()
    fig.subplots_adjust(top=0.8)
    fig.suptitle('Sources Power Consumption')
    if save_figure:
        plt.savefig(save_filename + "_Sources" + file_type)
    figs['sources'] = fig

    # ---------------------------- Systems ----------------------------
    fig = plt.figure(save_filename + "_Systems")
    axis_1 = plt.subplot(1, 1, 1)
    fig.set_size_inches(width, height)

    for i in range(len(results.segments)):
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        dist_dict = results.segments[i].conditions.energy.systems
        for j, tag in enumerate(dist_dict.keys()):
            system = dist_dict[tag]
            power = _get_power_arr(system)[:, 0]
            name = getattr(system, 'name', tag)
            if i == 0:
                axis_1.plot(time, power, color=line_colors[i], marker=ps.markers[j],
                            linewidth=ps.line_width, label=_label_from_name(name))
            else:
                axis_1.plot(time, power, color=line_colors[i], marker=ps.markers[j],
                            linewidth=ps.line_width)
    set_axes(axis_1)
    axis_1.set_xlabel('Time (mins)')
    axis_1.set_ylabel('Systems Power (W)')
    if show_legend:
        fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=5)
    fig.tight_layout()
    fig.subplots_adjust(top=0.8)
    fig.suptitle('Systems Power Consumption')
    if save_figure:
        plt.savefig(save_filename + "_Systems" + file_type)
    figs['systems'] = fig

    return figs