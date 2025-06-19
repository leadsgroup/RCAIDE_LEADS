# RCAIDE/Library/Plots/Emissions/plot_emissions
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
## @ingroup Library-Plots-Performance-Emissions 
def plot_emissions(results,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Emissions" ,
                             file_type = ".png",
                             width = 11, height = 5):
    """
    Generate plots showing CO2-equivalent emissions and emission indexes for various fuel species over mission segments.

    Parameters
    ----------
    results : Data
        Mission results data structure containing:
        results.segments[i].conditions.emissions.total with fields:
            - CO2 : array
                Carbon dioxide emissions [kg]
            - NOx : array
                Nitrogen oxide emissions [kg]
            - H2O : array
                Water vapor emissions [kg]
            - Contrails : array
                Contrail formation impact [kg CO2e]
            - Soot : array
                Particulate emissions [kg]
            - SO2 : array
                Sulfur dioxide emissions [kg]

    save_figure : bool, optional
        Save figure to file if True, default False

    show_legend : bool, optional
        Display segment legend if True, default True

    save_filename : str, optional
        Name for saved figure file, default "CO2e_Emissions"

    file_type : str, optional
        File extension for saved figure, default ".png"

    width : float, optional
        Figure width in inches, default 11

    height : float, optional
        Figure height in inches, default 7

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure showing stacked emissions contributions

    Notes
    -----
    Creates a stacked area plot showing:
        - Individual contributions from each emission type
        - Cumulative total CO2-equivalent impact
        - Breakdown by mission segment
        - Time history of emissions

    Different emission types are distinguished by fill colors
    and segments use different shades from the inferno colormap.

    **Definitions**

    'CO2-equivalent (CO2e)'
        Combined climate impact normalized to CO2
    
    'Global Warming Potential (GWP)'
        Relative impact factor for different emissions
    
    'Contrail Impact'
        Climate forcing from aviation-induced cloudiness
    """
 
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
      
    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height)  

    line_colors   = cm.inferno(np.linspace(0,0.9,len(results.segments)))
    
    cum_y0  = 0
    cum_y1  = 0 
    cum_y1_0 = 0  
    
    for i in range(len(results.segments)): 
        time                = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min 
        emissions_CO2       = results.segments[i].conditions.emissions.mass.CO2[:, 0] /1E3  
        emissions_CO        = results.segments[i].conditions.emissions.mass.CO[:, 0]  /1E3  
        emissions_NOx       = results.segments[i].conditions.emissions.mass.NOx[:, 0] /1E3  
        emissions_H2O       = results.segments[i].conditions.emissions.mass.H2O[:, 0] /1E3   
        EI_CO2              = results.segments[i].conditions.emissions.index.CO2[:, 0] 
        EI_CO               = results.segments[i].conditions.emissions.index.CO[:, 0] 
        EI_NOx              = results.segments[i].conditions.emissions.index.NOx[:, 0] 
        EI_H2O              = results.segments[i].conditions.emissions.index.H2O[:, 0] 
        EI_SO2              = results.segments[i].conditions.emissions.index.SO2[:, 0]        

        cum_y0 = np.zeros_like(emissions_CO2)  
        cum_y1 = cum_y1_0 + emissions_CO2 + emissions_NOx  + emissions_CO +  emissions_H2O  #+ emissions_Contrails +  emissions_Soot +  emissions_SO2    

        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ') 
        axis_1 = plt.subplot(1,2,1)    
        axis_1.fill_between(time, cum_y0, cum_y1, where=(cum_y0 < cum_y1), color= line_colors[i],  interpolate=True, label = segment_name)   
        cum_y1_0 = cum_y1[-1]  
        axis_1.set_ylabel(r'CO2e Emissions (Metric Tons)') 
        axis_1.set_xlabel(r'Time (mins)') 
        set_axes(axis_1)
         
        axis_2 = plt.subplot(1,2,2)
        if i == 0: 
            axis_2.plot(time, EI_CO2, color = line_colors[i], marker = ps.markers[0],markersize = ps.marker_size, linewidth = ps.line_width, label = r"$CO_2$") 
            axis_2.plot(time, EI_CO , color = line_colors[i], marker = ps.markers[1],markersize = ps.marker_size, linewidth = ps.line_width, label = r"$CO$" ) 
            axis_2.plot(time, EI_NOx, color = line_colors[i], marker = ps.markers[2],markersize = ps.marker_size, linewidth = ps.line_width, label = r"$NO_x$") 
            axis_2.plot(time, EI_H2O, color = line_colors[i], marker = ps.markers[3],markersize = ps.marker_size, linewidth = ps.line_width, label = r"$H_2O$") 
            axis_2.plot(time, EI_SO2, color = line_colors[i], marker = ps.markers[4],markersize = ps.marker_size, linewidth = ps.line_width, label = r"$SO_2$")
        else:

            axis_2.semilogy(time, EI_CO2, color = line_colors[i], marker = ps.markers[0],markersize = ps.marker_size, linewidth = ps.line_width) 
            axis_2.semilogy(time, EI_CO , color = line_colors[i], marker = ps.markers[1],markersize = ps.marker_size, linewidth = ps.line_width)
            axis_2.semilogy(time, EI_NOx, color = line_colors[i], marker = ps.markers[2],markersize = ps.marker_size, linewidth = ps.line_width) 
            axis_2.semilogy(time, EI_H2O, color = line_colors[i], marker = ps.markers[3],markersize = ps.marker_size, linewidth = ps.line_width) 
            axis_2.semilogy(time, EI_SO2, color = line_colors[i], marker = ps.markers[4],markersize = ps.marker_size, linewidth = ps.line_width)             
    
        axis_2.set_ylabel(r'Emission Index') 
        axis_2.set_xlabel(r'Time (mins)') 
        axis_2.minorticks_on()
        axis_2.grid(which='major', linestyle='-', linewidth=0.5, color='grey')
        axis_2.grid(which='minor', linestyle=':', linewidth=0.5, color='grey')
        axis_2.grid(True)         
                
    if show_legend:
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 6) 
        leg.set_title('Flight Segment', prop={'size': ps.legend_font_size, 'weight': 'heavy'})    
    
    # Adjusting the sub-plots for legend 
    
    # set title of plot 
    title_text    = 'Emissions'     
    fig.tight_layout() 
    fig.suptitle(title_text)
    
    fig.subplots_adjust(top=0.7)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return fig 