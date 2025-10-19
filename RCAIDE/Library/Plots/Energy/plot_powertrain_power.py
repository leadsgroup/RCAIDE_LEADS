# RCAIDE/Library/Plots/Energy/plot_powertrain_power.py
#
# Created:  Oct 2025, M. Guidotti

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

def plot_powertrain_power(results,
                          save_figure = False,
                          show_legend = True,
                          save_filename = "Powertrain_Power",
                          file_type = ".png",
                          width = 14, height = 9):

    # get plotting style 
    ps = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters) 
    
    # color map across segments (color by segment)
    line_colors = cm.inferno(np.linspace(0,0.9,len(results.segments)))     

    figs = {}

    # -------------------------------------------------- Propulsors net_dot--------------------------------------------------
    fig_prop = plt.figure(save_filename + "_Propulsors")
    fig_prop.set_size_inches(width,height)   
    ax_prop = plt.subplot(1,1,1)

    for network in results.segments[0].analyses.energy.vehicle.networks:  
        for p_i, propulsor in enumerate(network.propulsors):
            for i in range(len(results.segments)):  
                time  = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min      
                power = results.segments[i].conditions.energy.propulsors[propulsor.tag].power.propulsive[:,0]
                if i == 0:
                    ax_prop.plot(time, power, color=line_colors[i], marker=ps.markers[p_i],
                                 markersize=ps.marker_size, linewidth=ps.line_width, label=propulsor.tag)
                else:
                    ax_prop.plot(time, power, color=line_colors[i], marker=ps.markers[p_i],
                                 markersize=ps.marker_size, linewidth=ps.line_width)
    ax_prop.set_xlabel('Time (mins)')
    ax_prop.set_ylabel('Propulsors Power (W)')
    set_axes(ax_prop)
    if show_legend:
        fig_prop.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=4)
    fig_prop.tight_layout()
    fig_prop.subplots_adjust(top=0.85)
    fig_prop.suptitle('Propulsors Power')
    if save_figure:
        plt.savefig(save_filename + "_Propulsors" + file_type)
    figs['propulsors'] = fig_prop

    # -------------------------------------------------- Modulators net_dot--------------------------------------------------
    fig_mod = plt.figure(save_filename + "_Modulators")
    fig_mod.set_size_inches(width,height)   
    ax_mod  = plt.subplot(1,1,1)

    for network in results.segments[0].analyses.energy.vehicle.networks:  
        for m_i, modulator in enumerate(network.modulators):
            for i in range(len(results.segments)):  
                time  = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min      
                power = results.segments[i].conditions.energy.modulators[modulator.tag].power.electrical[:,0]
                if i == 0:
                    ax_mod.plot(time, power, color=line_colors[i], marker=ps.markers[m_i],
                                markersize=ps.marker_size, linewidth=ps.line_width, label=modulator.tag)
                else:
                    ax_mod.plot(time, power, color=line_colors[i], marker=ps.markers[m_i],
                                markersize=ps.marker_size, linewidth=ps.line_width)
    ax_mod.set_xlabel('Time (mins)')
    ax_mod.set_ylabel('Modulators Power (W)')
    set_axes(ax_mod)
    if show_legend:
        fig_mod.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=4)
    fig_mod.tight_layout()
    fig_mod.subplots_adjust(top=0.85)
    fig_mod.suptitle('Modulators Power')
    if save_figure:
        plt.savefig(save_filename + "_Modulators" + file_type)
    figs['modulators'] = fig_mod

    # -------------------------------------------------- Converters net_dot--------------------------------------------------
    fig_conv = plt.figure(save_filename + "_Converters")
    fig_conv.set_size_inches(width,height)   
    ax_conv = plt.subplot(1,1,1)

    for network in results.segments[0].analyses.energy.vehicle.networks:  
        for c_i, converter_tag in enumerate(network.non_propulsive_converters):
            for i in range(len(results.segments)):  
                time  = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min      
                power = results.segments[i].conditions.energy.converters[converter_tag].power.electrical[:,0]
                if i == 0:
                    ax_conv.plot(time, power, color=line_colors[i], marker=ps.markers[c_i],
                                 markersize=ps.marker_size, linewidth=ps.line_width, label=converter_tag)
                else:
                    ax_conv.plot(time, power, color=line_colors[i], marker=ps.markers[c_i],
                                 markersize=ps.marker_size, linewidth=ps.line_width)
    ax_conv.set_xlabel('Time (mins)')
    ax_conv.set_ylabel('Converters Power (W)')
    set_axes(ax_conv)
    if show_legend:
        fig_conv.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=4)
    fig_conv.tight_layout()
    fig_conv.subplots_adjust(top=0.85)
    fig_conv.suptitle('Converters Power')
    if save_figure:
        plt.savefig(save_filename + "_Converters" + file_type)
    figs['converters'] = fig_conv

    # -------------------------------------------------- Sources net_dot--------------------------------------------------
    fig_src = plt.figure(save_filename + "_Sources")
    fig_src.set_size_inches(width,height)   
    ax_src  = plt.subplot(1,1,1)

    for network in results.segments[0].analyses.energy.vehicle.networks:  
        for s_i, source in enumerate(network.sources):
            for i in range(len(results.segments)):  
                time  = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min      
                if isinstance(source, RCAIDE.Library.Components.Powertrain.Sources.Fuel_Tanks.Fuel_Tank):
                    power = results.segments[i].conditions.energy.sources[source.tag].power.chemical[:,0]
                else:
                    power = results.segments[i].conditions.energy.sources[source.tag].power.electrical[:,0]
                if i == 0:
                    ax_src.plot(time, power, color=line_colors[i], marker=ps.markers[s_i],
                                markersize=ps.marker_size, linewidth=ps.line_width, label=source.tag)
                else:
                    ax_src.plot(time, power, color=line_colors[i], marker=ps.markers[s_i],
                                markersize=ps.marker_size, linewidth=ps.line_width)
    ax_src.set_xlabel('Time (mins)')
    ax_src.set_ylabel('Sources Power (W)')
    set_axes(ax_src)
    if show_legend:
        fig_src.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=4)
    fig_src.tight_layout()
    fig_src.subplots_adjust(top=0.85)
    fig_src.suptitle('Sources Power')
    if save_figure:
        plt.savefig(save_filename + "_Sources" + file_type)
    figs['sources'] = fig_src

    # -------------------------------------------------- Systems --------------------------------------------------
    fig_sys = plt.figure(save_filename + "_Systems")
    fig_sys.set_size_inches(width,height)   
    ax_sys  = plt.subplot(1,1,1)

    for network in results.segments[0].analyses.energy.vehicle.networks:  
        for y_i, system in enumerate(network.systems):
            for i in range(len(results.segments)):  
                time  = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min      
                power = results.segments[i].conditions.energy.systems[system.tag].power.electrical[:,0]
                if i == 0:
                    ax_sys.plot(time, power, color=line_colors[i], marker=ps.markers[y_i],
                                markersize=ps.marker_size, linewidth=ps.line_width, label=system.tag)
                else:
                    ax_sys.plot(time, power, color=line_colors[i], marker=ps.markers[y_i],
                                markersize=ps.marker_size, linewidth=ps.line_width)
    ax_sys.set_xlabel('Time (mins)')
    ax_sys.set_ylabel('Systems Power (W)')
    set_axes(ax_sys)
    if show_legend:
        fig_sys.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol=4)
    fig_sys.tight_layout()
    fig_sys.subplots_adjust(top=0.85)
    fig_sys.suptitle('Systems Power')
    if save_figure:
        plt.savefig(save_filename + "_Systems" + file_type)
    figs['systems'] = fig_sys

    return figs