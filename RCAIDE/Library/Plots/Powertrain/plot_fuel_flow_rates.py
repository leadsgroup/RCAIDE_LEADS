# RCAIDE/Library/Plots/Performance/plot_fuel_flow_rates.py
# 
# 
# Created:  Jul 2023, M. Clarke

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
def plot_fuel_flow_rates(results,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Fuel_Flow_Rates" ,
                             file_type = ".png",
                             width = 11, height = 9):

    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
     
    line_colors   = segment_colors(len(results.segments))      
     
    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height)

    # --- Create 3 subplots ---
    axis_prop   = plt.subplot(3,1,1)
    axis_conv   = plt.subplot(3,1,2)
    axis_fuel   = plt.subplot(3,1,3)

    axis_prop.set_ylabel(r'Fuel Flow Rate (kg/s)')
    axis_fuel.set_ylabel(r'Fuel Flow Rate (kg/s)')
    axis_conv.set_ylabel(r'Fuel Flow Rate (kg/s)')

    axis_prop.set_xlabel(r'Time (min)')
    axis_fuel.set_xlabel(r'Time (min)')
    axis_conv.set_xlabel(r'Time (min)')


    set_axes(axis_prop)
    set_axes(axis_fuel)
    set_axes(axis_conv)

    converter_markers = ['o','s','^','d','v','>','<','p','h','x','+','*']

    for i in range(len(results.segments)): 

        time     = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min  
        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ') 

        for network in results.segments[i].analyses.vehicle.networks: 

            # ---------------- PROPULSORS ----------------
            for j , propulsor in enumerate(network.propulsors):

                propulsor_flow_rate = results.segments[i].conditions.energy.propulsors[propulsor.tag].fuel_mass_flow_rate[:,0]  

                label = propulsor.tag if i==0 else None

                axis_prop.plot(time,
                               propulsor_flow_rate,
                               color = line_colors[i],
                               marker = ps.markers[j],
                               linewidth = ps.line_width,
                               label = label)

            # ---------------- DISTRIBUTORS ----------------
            for j, distributor in enumerate(network.distributors):

                distributor_flow_rate = results.segments[i].conditions.energy.distributors[distributor.tag].fuel_mass_flow_rate

                label = distributor.tag if i==0 else None

                axis_fuel.plot(time,
                               distributor_flow_rate,
                               color = line_colors[i],
                               marker = ps.markers[j],
                               linewidth = ps.line_width,
                               label = label)

                # ---------------- CONVERTERS ----------------
                for converter_group in distributor.assigned_converters:
                    for m, converter_tag in enumerate(converter_group):

                        converter_flow_rate = results.segments[i].conditions.energy.converters[converter_tag].fuel_mass_flow_rate[:,0]

                        marker_style = converter_markers[m % len(converter_markers)]
                        label = converter_tag if i==0 and j==0 else None

                        axis_conv.plot(time,
                                       converter_flow_rate,
                                       color = line_colors[i],
                                       marker = marker_style,
                                       linewidth = ps.line_width,
                                       label = label)

    if show_legend:
        axis_prop.legend(fontsize=ps.legend_font_size)
        axis_fuel.legend(fontsize=ps.legend_font_size)
        axis_conv.legend(fontsize=ps.legend_font_size)

    fig.tight_layout()

    # fig.suptitle('Fuel Flow Rates')
    
    if save_figure:
        plt.savefig(save_filename + file_type)

    return fig
