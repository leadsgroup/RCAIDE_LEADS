# RCAIDE/Library/Plots/Emissions/plot_gCO2e_emissions
#
#
# Created:  Jul 2024, M. Clarke

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
def plot_CO2e_emissions(results,
                    save_figure = False,
                    save_filename = "CO2e_Emissions" ,
                    show_legend = True,
                    file_type = ".png",
                    width = 8, height = 5):
    """
    Generate a plot showing cumulative gCO2e emissions over the mission.

    Parameters
    ----------
    results : Data
        Mission results data structure containing:
        results.segments[i].conditions.emissions with fields:
            - cumulative_gCO2e : array
                Cumulative GWP-weighted CO2 equivalent emissions [g CO2e]

    save_figure : bool, optional
        Save figure to file if True, default False

    save_filename : str, optional
        Name for saved figure file, default "CO2e_Emissions"

    show_legend : bool, optional
        Show legend on the plot if True, default True   

    file_type : str, optional
        File extension for saved figure, default ".png"

    width : float, optional
        Figure width in inches, default 8

    height : float, optional
        Figure height in inches, default 5

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure showing cumulative gCO2e over the full mission timeline
    """ 
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
     
    # get line colors for plots 
    line_colors   = cm.inferno(np.linspace(0,0.9,len(results.segments)))     
     
    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height)
     
    for i in range(len(results.segments)): 
        time = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        gCO2e   = results.segments[i].conditions.emissions.cumulative_gCO2e[:, 0] / 1E6  
                       
        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ')
        axis_1 = plt.subplot(1,1,1) 
        axis_1.plot(time, gCO2e, color = line_colors[i], marker = ps.markers[0],markersize = ps.marker_size, linewidth = ps.line_width, label = segment_name)            
        axis_1.set_ylim([0, max(gCO2e)*1.1])
        axis_1.set_ylabel(r'$CO_2e$ (Metric Tons)')    
        axis_1.set_xlabel('Time (mins)') 
        set_axes(axis_1)   

    if show_legend:
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4)  
    
    # Adjusting the sub-plots for legend
    fig.tight_layout() 
    fig.subplots_adjust(top=0.8) 
    
    # set title of plot 
    title_text    = 'Cumulative CO2e Emissions'      
    fig.suptitle(title_text)    
    
    if save_figure:
        fig.savefig(save_filename   + file_type) 
    return fig
