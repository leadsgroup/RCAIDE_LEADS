
## @ingroup Library-Plots-Energy
# RCAIDE/Library/Plots/Energy/plot_l.py
# 
# 
# Created:  

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
def plot_liquid_hydrogen_tank_properties(results,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Liquid_Hydrogen_Tank_Properties" ,
                             file_type = ".png", 
                             width = 11, height = 7):

    # get plotting style 
    ps = plot_style()  

    parameters = {
        'axes.labelsize': ps.axis_font_size,
        'xtick.labelsize': ps.axis_font_size,
        'ytick.labelsize': ps.axis_font_size,
        'axes.titlesize': ps.title_font_size
    }
    plt.rcParams.update(parameters)
     
    # get line colors for plots 
    line_colors = cm.viridis(np.linspace(0,0.9,len(results.segments)))      
         
    fig = plt.figure(save_filename)
    fig.set_size_inches(width, height) 

    axis_1 = plt.subplot(3,2,1)  # Mass
    axis_2 = plt.subplot(3,2,2)  # Temperatures
    axis_3 = plt.subplot(3,2,3)  # Volumes
    axis_4 = plt.subplot(3,2,4)  # Pressure
    axis_5 = plt.subplot(3,2,5)  # Venting
    axis_6 = plt.subplot(3,2,6)  # Boiling
    
    for i, segment in enumerate(results.segments): 
        time = segment.conditions.frames.inertial.time[:, 0] / Units.min 

        for network in segment.analyses.vehicle.networks: 
            for fuel_line in network.fuel_lines:
                for fuel_tank in fuel_line.fuel_tanks:

                    tank_conditions = segment.conditions.energy.fuel_lines[fuel_line.tag].fuel_tanks[fuel_tank.tag]

                    # Extract variables
                    tank_fuel_mass     = tank_conditions.fuel_mass[:, 0]
                    ullage_mass        = tank_conditions.ullage_mass[:, 0]
                    fuel_temp          = tank_conditions.fuel_temperature[:, 0]
                    ullage_temp        = tank_conditions.ullage_temperature[:, 0]
                    fuel_volume        = tank_conditions.fuel_volume[:, 0]/Units.gallons
                    ullage_volume      = tank_conditions.ullage_volume[:, 0]/Units.gallons
                    pressure           = tank_conditions.pressure[:, 0]
                    vent_rate          = tank_conditions.vent_rate[:, 0]
                    boil_off_rate      = tank_conditions.boil_off_rate[:,0]
            
                    segment_tag  =  results.segments[i].tag
                    segment_name = segment_tag.replace('_', ' ')
                    
                    tank_label = f"{fuel_tank.tag}".replace("_"," ")

                    # --- Masses 
                    axis_1.plot(time, tank_fuel_mass, color=line_colors[i], linewidth=ps.line_width, label=f"{tank_label} fuel")
                    axis_1.plot(time, ullage_mass, color=line_colors[i], linestyle="--", linewidth=ps.line_width, label=f"{tank_label} ullage")

                    # --- Temperatures
                    axis_2.plot(time, fuel_temp, color=line_colors[i], linewidth=ps.line_width, label=f"{tank_label} fuel")
                    axis_2.plot(time, ullage_temp, color=line_colors[i],linestyle="--", linewidth=ps.line_width, label=f"{tank_label} ullage")

                    # --- Volumes
                    axis_3.plot(time, fuel_volume, color=line_colors[i], linewidth=ps.line_width, label=f"{tank_label} fuel")
                    axis_3.plot(time, ullage_volume, color=line_colors[i],  linestyle="--", linewidth=ps.line_width, label=f"{tank_label} ullage")

                    # --- Pressure
                    axis_4.plot(time, pressure, color=line_colors[i], linewidth=ps.line_width, label=f"{tank_label}")

                    # --- Venting Rate
                    axis_5.plot(time, vent_rate, color=line_colors[i], linewidth=ps.line_width, label=f"{tank_label}")
                    
                    # --- Boil Off Rate
                    axis_6.plot(time, boil_off_rate, color=line_colors[i], linewidth=ps.line_width, label=f"{tank_label}")

    # Axis labels
    axis_1.set_ylabel("Mass (kg)")
    axis_2.set_ylabel("Temp. (K)")
    axis_3.set_ylabel("Volume (gal)")
    axis_4.set_ylabel("Pressure (Pa)")
    axis_5.set_ylabel(r"$\dot{m}_{vent}$ (kg/s)")
    axis_6.set_ylabel(r"$\dot{m}_{boiloff}$ (kg/s)")

    for ax in [axis_1, axis_2, axis_3, axis_4, axis_5, axis_6]:
        ax.set_xlabel("Time (min)") 
        if show_legend:
            ax.legend(loc='upper center')
        set_axes(ax)
        
    # Adjust layout
    fig.tight_layout() 
    fig.subplots_adjust(top=0.9)

    # Title
    fig.suptitle("Liquid Hydrogen Tank Properties")

    if save_figure:
        plt.savefig(save_filename + file_type)

    return fig