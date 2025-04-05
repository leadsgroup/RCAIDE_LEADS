# RCAIDE/Library/Plots/Energy/plot_battery_ragone_diagram.py
# 
# 
# Created:  Jul 2023, M. Clarke

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
def plot_battery_ragone_diagram(battery,
                          save_figure   = False, 
                          save_filename = "Ragone_Plot",
                          file_type     =  ".png",
                          width = 11, height = 7):
    """
    Creates a Ragone plot showing the relationship between specific power and specific energy of a battery.

    Parameters
    ----------
    battery : Battery
        RCAIDE battery object containing Ragone characteristics
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    save_filename : str, optional
        Name of file for saved figure (default: "Ragone_Plot")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 11)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure handle containing the generated Ragone plot

    Notes
    -----
    The Ragone plot is a performance map showing the tradeoff between specific power 
    and specific energy in energy storage devices. The plot uses logarithmic scales 
    to display the wide range of values.
    
    **Major Assumptions**
    
    * Battery characteristics follow the Ragone relationship:
        P = const_1 * 10^(E * const_2)
    * Specific energy range is defined by upper and lower bounds
    * Values are converted to standard units (kW/kg and Wh/kg)

    **Definitions**
    
    'Specific Power'
        Power output per unit mass (kW/kg)
    'Specific Energy'
        Energy storage capacity per unit mass (Wh/kg)
    'Ragone Plot'
        Performance visualization showing the tradeoff between power and energy density
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
        
    axis_1 = plt.subplot(1,1,1)
    esp_plot = np.linspace(battery.cell.ragone.lower_bound, battery.cell.ragone.upper_bound,50)
    psp_plot = battery.cell.ragone.const_1*10**(esp_plot*battery.cell.ragone.const_2)
    axis_1.plot(esp_plot/(Units.Wh/Units.kg),psp_plot/(Units.kW/Units.kg), color = 'black', marker = ps.markers[0], linewidth = ps.line_width, label= battery.tag) 
    axis_1.set_ylabel('Specific Power (kW/kg)')
    axis_1.set_xlabel('Specific Energy (W-h/kg)')
    set_axes(axis_1)       
     
    # set title of plot 
    title_text    = 'Battery Ragone Plot'      
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   
    return  fig 