# RCAIDE/Library/Plots/Performance/plot_payload_range_diagram.py
# 
# 
# Created: Feb 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import Units , Data  
from RCAIDE.Library.Plots.Common import set_axes, plot_style     
 
# Pacakge imports 
import numpy as np
from matplotlib import pyplot as plt 

def plot_payload_range_diagram(payload_range):
 
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
    
    if fuel_name ==  None: 
        fig  = plt.figure( vehicle.tag + ' Fuel_Payload_Range_Diagram')
    else:
        fig  = plt.figure(vehicle.tag + ' Fuel_Payload_Range_Diagram for ' + fuel_name)
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
 
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)

    fig  = plt.figure('Electric_Payload_Range_Diagram')
    axis = fig.add_subplot(1,1,1)        
    axis.plot(payload_range.range /Units.nmi, payload_range.payload,color = 'k', linewidth = ps.line_width )
    axis.set_xlabel('Range (nautical miles)')
    axis.set_ylabel('Payload (kg)')
    axis.set_title('Payload Range Diagram')
    set_axes(axis) 
    fig.tight_layout()