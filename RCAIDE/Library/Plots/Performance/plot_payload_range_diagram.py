# RCAIDE/Library/Methods/Plots/Performance/plot_payload_range_diagram.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
from RCAIDE.Framework.Core import Units , Data   
from RCAIDE.Library.Plots.Common import set_axes, plot_style    
from RCAIDE.Library.Plots import *

# python imports 
from matplotlib import pyplot as plt
 
# ----------------------------------------------------------------------
#  Plot Payload Range Diagram
# ----------------------------------------------------------------------  
def plot_payload_range_diagram(payload_range, 
                               save_figure = False,
                               show_legend = True,
                               save_filename = "Payload_Range_Diagram",
                               file_type = ".png",
                               width = 11, height = 7):
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
   
    fig  = plt.figure(save_filename) 
    fig.set_size_inches(width,height)  
    axis_1 = fig.add_subplot(1,2,1)
    axis_1.plot(payload_range.range /Units.nmi,payload_range.payload/Units.lbm  ,color = 'k', linewidth = ps.line_width )
    axis_1.set_xlabel('Range (nautical miles)')
    axis_1.set_ylabel('Payload (lbs)') 
    set_axes(axis_1) 

    axis_2 = fig.add_subplot(1,2,2)
    axis_2.plot(payload_range.range /Units.nmi,payload_range.oew_plus_payload/Units.lbm ,color = 'k', linewidth = ps.line_width )
    axis_2.set_xlabel('Range (nautical miles)')
    axis_2.set_ylabel('OEW + Payload (lbs)') 
    set_axes(axis_2)
    
    fig.tight_layout() 
     
    # set title of plot 
    title_text    = 'Payload Range'   
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename  + file_type)   
    return fig     