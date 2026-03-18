# RCAIDE/Library/Plots/Performance/plot_payload_range_diagram.py
# 
# 
# Created: Feb 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports 
from RCAIDE.Framework.Core import Units   
from RCAIDE.Library.Plots.Common import set_axes, plot_style     
 
# Pacakge imports  
from matplotlib import pyplot as plt 

# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------   
def plot_payload_range_diagram(results,
                               save_figure               = False,
                               show_legend               = True,
                               save_filename             = "Payload_Range_Diagram",
                               file_type                 = ".png",  
                               width                     = 11,
                               height                    = 6):
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
       
     
    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height) 
     
    axis_1 = fig.add_subplot(1,2,1)
    axis_1.plot(results.range /Units.nmi,results.payload/Units.lbm  ,color = 'k', linewidth = ps.line_width )
    axis_1.set_xlabel('Range (nautical miles)')
    axis_1.set_ylabel('Payload (lbs)') 
    set_axes(axis_1) 

    axis_2 = fig.add_subplot(1,2,2)
    axis_2.plot(results.range /Units.nmi,results.oew_plus_payload/Units.lbm ,color = 'k', linewidth = ps.line_width )
    axis_2.set_xlabel('Range (nautical miles)')
    axis_2.set_ylabel('OEW + Payload (lbs)') 
    set_axes(axis_2) 
    fig.tight_layout()   
    
    # Adjusting the sub-plots for legend
    fig.tight_layout()   
    if save_figure:
        fig.savefig(save_filename   + file_type)  
    return fig 