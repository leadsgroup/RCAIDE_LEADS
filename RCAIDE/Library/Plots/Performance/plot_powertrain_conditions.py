# RCAIDE/Library/Plots/Performance/plot_powertrain_conditions.py
# 
# 
# Created:  Jul 2023, M. Clarke

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
def plot_powertrain_conditions(results,
                                save_figure = False,
                                show_legend = True,
                                save_filename = "Powertain_Power_Flow",
                                file_type = ".png",
                                width = 11, height = 7):
    """
    Creates a six-panel plot showing various powertrain component power consumption.

    Parameters
    ----------
    results : Results
        RCAIDE results structure containing segment data and powertrain conditions
        
    save_figure : bool, optional
        Flag for saving the figure (default: False)
        
    show_legend : bool, optional
        Flag for displaying plot legend (default: True)
        
    save_filename : str, optional
        Base name of file for saved figure (default: "Powertain_Power_Conditions")
        
    file_type : str, optional
        File extension for saved figure (default: ".png")
        
    width : float, optional
        Figure width in inches (default: 11)
        
    height : float, optional
        Figure height in inches (default: 7)

    Returns
    -------
    fig : matplotlib.figure.Figure
        Figure handle containing the generated plots

    Notes
    -----
    The function creates a 3x2 subplot containing:
        1. propulsive power vs time
        2. chemical  energy vs time
        3. electrical current vs time
        4. thermal  power vs time
        5. hydraulic power vstime
        6. pneumatic power vs time
    
    Each segment is plotted with a different color from the inferno colormap. 
    
    **Definitions** 
    """ 
    
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
     
    power_types = ['propulsive','chemical','electrical','thermal','hydraulic','pneumatic', 'mechanical']
      
    fig_1 = plt.figure(power_types[0] + '_' + save_filename)
    fig_1.set_size_inches(width,height)
    axis_1_1 = fig_1.add_subplot(1,2,1)  
    axis_1_2 = fig_1.add_subplot(1,2,2)

    fig_2 = plt.figure(power_types[1] + '_' + save_filename)
    fig_2.set_size_inches(width,height)
    axis_2_1 = fig_2.add_subplot(1,2,1)  
    axis_2_2 = fig_2.add_subplot(1,2,2)
    

    fig_3 = plt.figure(power_types[2] + '_' + save_filename)
    fig_3.set_size_inches(width,height)
    axis_3_1 = fig_3.add_subplot(1,2,1)  
    axis_3_2 = fig_3.add_subplot(1,2,2)
    

    fig_4 = plt.figure(power_types[3] + '_' +  save_filename)
    fig_4.set_size_inches(width,height)
    axis_4_1 = fig_4.add_subplot(1,2,1)  
    axis_4_2 = fig_4.add_subplot(1,2,2)
    

    fig_5 = plt.figure(power_types[4] + '_' +  save_filename)
    fig_5.set_size_inches(width,height)
    axis_5_1 = fig_5.add_subplot(1,2,1)  
    axis_5_2 = fig_5.add_subplot(1,2,2)
    

    fig_6 = plt.figure(power_types[5] + '_' + save_filename)
    fig_6.set_size_inches(width,height)
    axis_6_1 = fig_6.add_subplot(1,2,1)  
    axis_6_2 = fig_6.add_subplot(1,2,2)
    

    fig_7 = plt.figure(power_types[6] + '_' + save_filename)
    fig_7.set_size_inches(width,height)
    axis_7_1 = fig_7.add_subplot(1,2,1)  
    axis_7_2 = fig_7.add_subplot(1,2,2)
    
        
    axes = [[axis_1_1,axis_1_2],[axis_2_1,axis_2_2],
            [axis_3_1,axis_3_2],[axis_4_1,axis_4_2],
            [axis_5_1,axis_5_2],[axis_6_1,axis_6_2],
            [axis_7_1,axis_7_2]]
    
    
    # get line colors for plots 
    line_colors   = cm.inferno(np.linspace(0,0.9,len(results.segments)))      
     
       
    for i in range(len(results.segments)):  
        time                = results.segments[i].conditions.frames.inertial.time[:,0] / Units.min
        energy_conditions   = results.segments[i].state.conditions.energy 

        for network in  results.segments[i].analyses.vehicle.networks:  
            for p_i,propulsor in enumerate(network.propulsors):
                for input_power_key in energy_conditions.propulsors[propulsor.tag].inputs.power.keys():
                    p_idx = power_types.index(input_power_key)
                    input_power  = energy_conditions.propulsors[propulsor.tag].inputs.power[input_power_key][:,0]
                    plot_power(propulsor, time,input_power,input_power_key,axes[p_idx][0],ps,line_colors,0,i)
                for output_power_key in energy_conditions.propulsors[propulsor.tag].outputs.power.keys():
                    p_idx = power_types.index(output_power_key)
                    output_power = energy_conditions.propulsors[propulsor.tag].outputs.power[output_power_key][:,0]
                    plot_power(propulsor,time,output_power,output_power_key,axes[p_idx][1],ps,line_colors,0,i) 
        
            for c_i,converter in enumerate(network.converters):
                for input_power_key in energy_conditions.converters[converter.tag].inputs.power.keys():
                    p_idx = power_types.index(input_power_key)
                    input_power = energy_conditions.converters[converter.tag].inputs.power[input_power_key][:,0]
                    plot_power(converter, time,input_power,input_power_key,axes[p_idx][0],ps,line_colors,1,i)
                for output_power_key in energy_conditions.converters[converter.tag].outputs.power.keys():
                    p_idx = power_types.index(output_power_key)
                    output_power = energy_conditions.converters[converter.tag].outputs.power[output_power_key][:,0]
                    plot_power(converter,time,output_power,output_power_key,axes[p_idx][1],ps,line_colors,1,i) 
    
            for m_i,modulator in enumerate(network.modulators):       
                for input_power_key in energy_conditions.modulators[modulator.tag].inputs.power.keys():
                    p_idx = power_types.index(input_power_key)
                    input_power = energy_conditions.modulators[modulator.tag].inputs.power[input_power_key][:,0]
                    plot_power(modulator, time,input_power,input_power_key,axes[p_idx][0],ps,line_colors,2,i)
                for output_power_key in energy_conditions.modulators[modulator.tag].outputs.power.keys():
                    p_idx = power_types.index(output_power_key)
                    output_power = energy_conditions.modulators[modulator.tag].outputs.power[output_power_key][:,0]
                    plot_power(modulator,time,output_power,output_power_key,axes[p_idx][1],ps,line_colors,2,i)  
    
            for s_i,source in enumerate(network.sources):       
                for input_power_key in energy_conditions.sources[source.tag].inputs.power.keys():
                    p_idx = power_types.index(input_power_key)
                    input_power = energy_conditions.sources[source.tag].inputs.power[input_power_key][:,0]
                    plot_power(source, time,input_power,input_power_key,axes[p_idx][0],ps,line_colors,3,i) 
                for output_power_key in energy_conditions.sources[source.tag].outputs.power.keys(): 
                    p_idx = power_types.index(output_power_key)
                    output_power = energy_conditions.sources[source.tag].outputs.power[output_power_key][:,0]
                    plot_power(source,time,output_power,output_power_key,axes[p_idx][1],ps,line_colors,3,i)   
    
            for sy_i, system in enumerate(network.systems):     
                for input_power_key in energy_conditions.systems[system.tag].inputs.power.keys():
                    p_idx = power_types.index(input_power_key)
                    input_power = energy_conditions.systems[system.tag].inputs.power[input_power_key][:,0]
                    plot_power(system, time,input_power,input_power_key,axes[p_idx][0],ps,line_colors,4,i) 
                for output_power_key in energy_conditions.systems[system.tag].outputs.power.keys():  
                    p_idx = power_types.index(output_power_key)
                    output_power = energy_conditions.systems[system.tag].outputs.power[output_power_key][:,0]
                    plot_power(system,time,output_power,output_power_key,axes[p_idx][1],ps,line_colors,4,i)                      
                   
                
    for ax_i in range(len(axes)):
        axes[ax_i][0].set_ylabel(r'$P_{'+ power_types[ax_i] + '}$ In  (MW)')
        axes[ax_i][1].set_ylabel(r'$P_{'+ power_types[ax_i] + '}$ Out (MW)')
        set_axes(axes[ax_i][0])                            
        set_axes(axes[ax_i][1])
        
        ymin0, ymax0 =  axes[ax_i][0].get_ylim()
        ymin1, ymax1 =  axes[ax_i][1].get_ylim()
        
        ymin = np.minimum(ymin0,ymin1)
        ymax = np.maximum(ymax0,ymax1)
        
        axes[ax_i][0].set_ylim(ymin,ymax) 
        axes[ax_i][1].set_ylim(ymin,ymax)        
        
         
        if show_legend: 
            leg_1 =  fig_1.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
            leg_2 =  fig_2.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
            leg_3 =  fig_3.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
            leg_4 =  fig_4.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
            leg_5 =  fig_5.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
            leg_6 =  fig_6.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4)
            leg_7 =  fig_7.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4)
    
            leg_1.set_title('Powertrain Components', prop={'size': ps.legend_font_size, 'weight': 'heavy'}) 
            leg_2.set_title('Powertrain Components', prop={'size': ps.legend_font_size, 'weight': 'heavy'})
            leg_3.set_title('Powertrain Components', prop={'size': ps.legend_font_size, 'weight': 'heavy'})
            leg_4.set_title('Powertrain Components', prop={'size': ps.legend_font_size, 'weight': 'heavy'})
            leg_5.set_title('Powertrain Components', prop={'size': ps.legend_font_size, 'weight': 'heavy'})
            leg_6.set_title('Powertrain Components', prop={'size': ps.legend_font_size, 'weight': 'heavy'})  
            leg_7.set_title('Powertrain Components', prop={'size': ps.legend_font_size, 'weight': 'heavy'})  
             
             
    fig_1.tight_layout() 
    fig_2.tight_layout()
    fig_3.tight_layout()
    fig_4.tight_layout()
    fig_5.tight_layout()
    fig_6.tight_layout() 
    fig_7.tight_layout() 
    fig_1.subplots_adjust(top=0.8)
    fig_2.subplots_adjust(top=0.8)
    fig_3.subplots_adjust(top=0.8)
    fig_4.subplots_adjust(top=0.8)
    fig_5.subplots_adjust(top=0.8)
    fig_6.subplots_adjust(top=0.8) 
    fig_7.subplots_adjust(top=0.8) 
            
    # set title of plot        
    fig_1.suptitle(power_types[0], +  'power' ) 
    fig_2.suptitle(power_types[1], +  'power' ) 
    fig_3.suptitle(power_types[2], +  'power' ) 
    fig_4.suptitle(power_types[3], +  'power' ) 
    fig_5.suptitle(power_types[4], +  'power' ) 
    fig_6.suptitle(power_types[5], +  'power' ) 
    fig_7.suptitle(power_types[6], +  'power' ) 
                               
    if save_figure:
        fig_1.savefig(save_filename + file_type)  
        fig_2.savefig(save_filename + file_type)
        fig_3.savefig(save_filename + file_type)
        fig_4.savefig(save_filename + file_type)
        fig_5.savefig(save_filename + file_type)
        fig_6.savefig(save_filename + file_type) 
        fig_7.savefig(save_filename + file_type)  
    return fig_1, fig_2, fig_3, fig_4, fig_5, fig_6, fig_7

def plot_power(component,time,power,power_type,axis,ps,line_colors,c_i,i):
    power_MW = power / 1E6 
    if i == 0:
        axis.plot(time, power_MW, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width, label =component.tag)            
    else:
        axis.plot(time, power_MW, color = line_colors[i], marker = ps.markers[c_i], linewidth = ps.line_width)
         
    
    return 