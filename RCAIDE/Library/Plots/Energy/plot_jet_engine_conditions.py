# RCAIDE/Library/Plots/Energy/plot_jet_engine_conditions.py
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

import matplotlib.colors
import matplotlib.colors as colors  
# ----------------------------------------------------------------------------------------------------------------------
#  PLOTS
# ----------------------------------------------------------------------------------------------------------------------   
## @ingroup Library-Plots-Performance-Energy
def plot_jet_engine_conditions(results,
                             save_figure = False,
                             show_legend = True,
                             save_filename = "Jet_Engine_Velocities" ,
                             file_type = ".png",
                             width = 11, height = 7):
 
    # get plotting style 
    ps      = plot_style()  

    parameters = {'axes.labelsize': ps.axis_font_size,
                  'xtick.labelsize': ps.axis_font_size,
                  'ytick.labelsize': ps.axis_font_size,
                  'axes.titlesize': ps.title_font_size}
    plt.rcParams.update(parameters)
       
    
    elapsed_time,core_T, core_P, core_M, fan_T,  fan_P, fan_M =  get_jet_engine_performance_data(results)
    
    min_T_level = 200
    max_T_level = 2000
    
    min_P_level = np.minimum(np.min(fan_P), np.min(core_P)) / 1E6
    max_P_level = np.maximum(np.max(core_P), np.max(fan_P)) / 1E6
     
    # Create a meshgrid    
    non_dim_fan  =  np.linspace(0, 1,len(fan_T[0, :]))
    non_dim_core =  np.linspace(0, 1,len(core_T[0, :]))
    t_fan, loc_fan = np.meshgrid(elapsed_time[:, 0], non_dim_fan) 
    t_core, loc_core = np.meshgrid(elapsed_time[:, 0], non_dim_core)    
    cmap =  'viridis'
    # contour map bounds settings 
    T_levels   = np.linspace(min_T_level,max_T_level,10)  
    T_lvls     = np.linspace(min_T_level,max_T_level,5)
    P_levels   = np.linspace(min_P_level,max_P_level,10)
    P_lvls     = np.linspace(min_P_level,max_P_level,5) 
     
    # define figures 
    fig   = plt.figure(save_filename)
    fig.set_size_inches(width,height)
     
    # power 
    axis_1 = plt.subplot(3,2,1)
    axis_1.set_title(r'Core')
    axis_1.set_ylabel(r'% location')
    CS_1  = axis_1.contourf(t_core.T, loc_core.T ,core_P/ 1E6 ,P_levels,cmap =  cmap,extend='both')  
     
    axis_2 = plt.subplot(3,2,2)
    axis_2.set_title(r'Fan')
    axis_2.set_ylabel(r'% location')
    CS_2 = axis_2.contourf(t_fan.T, loc_fan.T ,fan_P/ 1E6  ,P_levels,cmap = cmap,extend='both') 
    cbar  = fig.colorbar(CS_2, ax=axis_2, ticks=P_lvls, format='%.2f')    
    cbar.ax.set_ylabel(r'$P_t$ (MPa)', rotation =  90)      

    axis_3 = plt.subplot(3,2,3) 
    axis_3.set_ylabel(r'% location')
    CS_3 = axis_3.contourf(t_core.T, loc_core.T ,core_T ,T_levels,cmap =  cmap,extend='both')  

    axis_4 = plt.subplot(3,2,4)
    axis_4.set_ylabel(r'% location')
    CS_4  = axis_4.contourf(t_fan.T, loc_fan.T ,fan_T ,T_levels,cmap = cmap,extend='both') 
    cbar  = fig.colorbar(CS_4, ax=axis_4, ticks=T_lvls) 
    cbar.ax.set_ylabel(r'$T_t$ (K)', rotation =  90)   

    axis_5 = plt.subplot(3,2,5) 
    axis_5.set_ylabel(r'$V_{exit}$ (m/s)')
    axis_5.set_xlabel(r'Time (min)') 
    set_axes(axis_5)

    axis_6 = plt.subplot(3,2,6)  
    axis_6.set_ylabel(r'$V_{exit}$ (m/s)')
    axis_6.set_xlabel(r'Time (min)') 
    set_axes(axis_6)
    

    # get line colors for plots 
    line_colors   = cm.inferno(np.linspace(0,0.9,len(results.segments))) 
    
    for i in range(len(results.segments)): 
        time     = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min  
        segment_tag  =  results.segments[i].tag
        segment_name = segment_tag.replace('_', ' ') 
                         
        for network in results.segments[i].analyses.energy.vehicle.networks:

            propulsor_tag = list(network.propulsors.keys())[0]
            propulsor  =  network.propulsors[propulsor_tag]
             
            core_nozzle   = propulsor.core_nozzle
            core_velocity = results.segments[i].conditions.energy.converters[core_nozzle.tag].outputs.velocity[:,0]  
            axis_5.plot(time, core_velocity, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width, label = segment_name)   
            axis_5.plot(time, core_velocity, color = line_colors[i], marker = ps.markers[0], linewidth = ps.line_width)  
                
            if 'fan_nozzle' in  propulsor:
                fan_nozzle    = propulsor.fan_nozzle
                fan_velocity = results.segments[i].conditions.energy.converters[fan_nozzle.tag].outputs.velocity[:,0] 
                axis_6.plot(time, fan_velocity, color = line_colors[i], marker = ps.markers[1], linewidth = ps.line_width) 
        
    
    if show_legend:
        leg =  fig.legend(bbox_to_anchor=(0.5, 0.95), loc='upper center', ncol = 4) 
        leg.set_title('Propulsor', prop={'size': ps.legend_font_size, 'weight': 'heavy'})    
    
    # Adjusting the sub-plots for legend 
    fig.tight_layout()
    fig.subplots_adjust(top=0.8)
    
    # set title of plot 
    title_text    = 'Engine Conditions'      
    fig.suptitle(title_text)
    
    if save_figure:
        plt.savefig(save_filename + file_type)   


    return fig 


def get_jet_engine_performance_data(results):
    
    # determine what compoments are on the propulsor
    n_c = 0
    n_f = 0     
    for network in results.segments[0].analyses.energy.vehicle.networks:
        propulsor_tag = list(network.propulsors.keys())[0]
        propulsor  =  network.propulsors[propulsor_tag] 
        if 'ram' in propulsor:
            n_c += 1
            n_f += 1
        if 'inlet_nozzle' in propulsor:     
            n_c += 1
            n_f += 1
        if 'fan' in propulsor:
            n_c += 1
            n_f += 1
        if 'low_pressure_compressor' in propulsor:
            n_c += 1
        if 'high_pressure_compressor' in propulsor:
            n_c += 1
        if 'combustor' in propulsor:
            n_c += 1
        if 'high_pressure_turbine' in propulsor:
            n_c += 1
        if 'low_pressure_turbine' in propulsor:
            n_c += 1
        if 'core_nozzle' in propulsor:
            n_c += 1
        if 'fan_nozzle' in propulsor:
            n_f += 1
        if 'afterburner' in propulsor:
            n_c += 1
                
    core_T = np.empty((0, n_c)) 
    core_P = np.empty((0, n_c))
    core_M = np.empty((0, n_c))
    fan_T  = np.empty((0, n_f)) 
    fan_P  = np.empty((0, n_f)) 
    fan_M  = np.empty((0, n_f)) 
    elapsed_time = np.empty((0, 1)) 
    for i in range(len(results.segments)):     
        elapsed_time = np.concatenate((elapsed_time, results.segments[i].conditions.frames.inertial.time / Units.min), axis=0)
        for network in results.segments[i].analyses.energy.vehicle.networks:
    
            propulsor_tag = list(network.propulsors.keys())[0]
            propulsor  =  network.propulsors[propulsor_tag]
             
            if 'ram' in propulsor: 
                ram             = propulsor.ram
                seg_core_T          = results.segments[i].conditions.energy.converters[ram.tag].outputs.stagnation_temperature 
                seg_core_P          = results.segments[i].conditions.energy.converters[ram.tag].outputs.stagnation_pressure 
                seg_core_M          = results.segments[i].conditions.energy.converters[ram.tag].outputs.mach_number 
                seg_fan_T           = results.segments[i].conditions.energy.converters[ram.tag].outputs.stagnation_temperature 
                seg_fan_P           = results.segments[i].conditions.energy.converters[ram.tag].outputs.stagnation_pressure 
                seg_fan_M           = results.segments[i].conditions.energy.converters[ram.tag].outputs.mach_number 
                
            if 'inlet_nozzle' in propulsor: 
                inlet_nozzle    = propulsor.inlet_nozzle
                seg_core_T          = np.concatenate((seg_core_T , results.segments[i].conditions.energy.converters[inlet_nozzle.tag].outputs.stagnation_temperature), axis=1)
                seg_core_P          = np.concatenate((seg_core_P , results.segments[i].conditions.energy.converters[inlet_nozzle.tag].outputs.stagnation_pressure), axis=1)
                seg_core_M          = np.concatenate((seg_core_M , results.segments[i].conditions.energy.converters[inlet_nozzle.tag].outputs.mach_number), axis=1)
                seg_fan_T           = np.concatenate((seg_fan_T  , results.segments[i].conditions.energy.converters[inlet_nozzle.tag].outputs.stagnation_temperature), axis=1)
                seg_fan_P           = np.concatenate((seg_fan_P  , results.segments[i].conditions.energy.converters[inlet_nozzle.tag].outputs.stagnation_pressure), axis=1)
                seg_fan_M           = np.concatenate((seg_fan_M  , results.segments[i].conditions.energy.converters[inlet_nozzle.tag].outputs.mach_number), axis=1)
                
            if 'fan' in propulsor: 
                fan   = propulsor.fan  
                seg_core_T          = np.concatenate((seg_core_T , results.segments[i].conditions.energy.converters[fan.tag].outputs.stagnation_temperature), axis=1)
                seg_core_P          = np.concatenate((seg_core_P , results.segments[i].conditions.energy.converters[fan.tag].outputs.stagnation_pressure), axis=1)
                seg_core_M          = np.concatenate((seg_core_M , results.segments[i].conditions.energy.converters[fan.tag].outputs.mach_number), axis=1)
                seg_fan_T           = np.concatenate((seg_fan_T  , results.segments[i].conditions.energy.converters[fan.tag].outputs.stagnation_temperature), axis=1)
                seg_fan_P           = np.concatenate((seg_fan_P  , results.segments[i].conditions.energy.converters[fan.tag].outputs.stagnation_pressure), axis=1)
                seg_fan_M           = np.concatenate((seg_fan_M  , results.segments[i].conditions.energy.converters[fan.tag].outputs.mach_number), axis=1)
                 
            if 'low_pressure_compressor' in propulsor: 
                lpc  = propulsor.low_pressure_compressor
                seg_core_T          = np.concatenate((seg_core_T , results.segments[i].conditions.energy.converters[lpc.tag].outputs.stagnation_temperature), axis=1)
                seg_core_P          = np.concatenate((seg_core_P , results.segments[i].conditions.energy.converters[lpc.tag].outputs.stagnation_pressure), axis=1)
                seg_core_M          = np.concatenate((seg_core_M , results.segments[i].conditions.energy.converters[lpc.tag].outputs.mach_number), axis=1)
                
            if 'high_pressure_compressor' in propulsor: 
                hpc  = propulsor.high_pressure_compressor
                seg_core_T          = np.concatenate((seg_core_T , results.segments[i].conditions.energy.converters[hpc.tag].outputs.stagnation_temperature), axis=1)
                seg_core_P          = np.concatenate((seg_core_P , results.segments[i].conditions.energy.converters[hpc.tag].outputs.stagnation_pressure), axis=1)
                seg_core_M          = np.concatenate((seg_core_M , results.segments[i].conditions.energy.converters[hpc.tag].outputs.mach_number), axis=1)
                
            if 'combustor' in propulsor: 
                combustor                 = propulsor.combustor
                seg_core_T          = np.concatenate((seg_core_T , results.segments[i].conditions.energy.converters[combustor.tag].outputs.stagnation_temperature), axis=1)
                seg_core_P          = np.concatenate((seg_core_P , results.segments[i].conditions.energy.converters[combustor.tag].outputs.stagnation_pressure), axis=1)
                seg_core_M          = np.concatenate((seg_core_M , results.segments[i].conditions.energy.converters[combustor.tag].outputs.mach_number), axis=1)
                
            if 'high_pressure_turbine' in propulsor: 
                hpt     = propulsor.high_pressure_turbine
                seg_core_T          = np.concatenate((seg_core_T , results.segments[i].conditions.energy.converters[hpt.tag].outputs.stagnation_temperature), axis=1)
                seg_core_P          = np.concatenate((seg_core_P , results.segments[i].conditions.energy.converters[hpt.tag].outputs.stagnation_pressure), axis=1)
                seg_core_M          = np.concatenate((seg_core_M , results.segments[i].conditions.energy.converters[hpt.tag].outputs.mach_number), axis=1)
                
            if 'low_pressure_turbine' in propulsor: 
                lpt      = propulsor.low_pressure_turbine 
                seg_core_T          = np.concatenate((seg_core_T , results.segments[i].conditions.energy.converters[lpt.tag].outputs.stagnation_temperature), axis=1)
                seg_core_P          = np.concatenate((seg_core_P , results.segments[i].conditions.energy.converters[lpt.tag].outputs.stagnation_pressure), axis=1)
                seg_core_M          = np.concatenate((seg_core_M , results.segments[i].conditions.energy.converters[lpt.tag].outputs.mach_number), axis=1)
                
            if 'afterburner' in propulsor: 
                afterburner         = propulsor.afterburner 
                seg_core_T          = np.concatenate((seg_core_T , results.segments[i].conditions.energy.converters[afterburner.tag].outputs.stagnation_temperature), axis=1)
                seg_core_P          = np.concatenate((seg_core_P , results.segments[i].conditions.energy.converters[afterburner.tag].outputs.stagnation_pressure), axis=1)
                seg_core_M          = np.concatenate((seg_core_M , results.segments[i].conditions.energy.converters[afterburner.tag].outputs.mach_number), axis=1) 
                
            if 'core_nozzle' in propulsor: 
                core_nozzle         = propulsor.core_nozzle
                seg_core_T          = np.concatenate((seg_core_T , results.segments[i].conditions.energy.converters[core_nozzle.tag].outputs.stagnation_temperature), axis=1)
                seg_core_P          = np.concatenate((seg_core_P , results.segments[i].conditions.energy.converters[core_nozzle.tag].outputs.stagnation_pressure), axis=1)
                seg_core_M          = np.concatenate((seg_core_M , results.segments[i].conditions.energy.converters[core_nozzle.tag].outputs.mach_number), axis=1)
                
            if 'fan_nozzle' in propulsor: 
                fan_nozzle         = propulsor.fan_nozzle         
                seg_fan_T          = np.concatenate((seg_fan_T , results.segments[i].conditions.energy.converters[fan_nozzle.tag].outputs.stagnation_temperature), axis=1)
                seg_fan_P          = np.concatenate((seg_fan_P , results.segments[i].conditions.energy.converters[fan_nozzle.tag].outputs.stagnation_pressure), axis=1)
                seg_fan_M          = np.concatenate((seg_fan_M , results.segments[i].conditions.energy.converters[fan_nozzle.tag].outputs.mach_number), axis=1)
                
            core_T          = np.concatenate((core_T,seg_core_T),axis=0)
            core_P          = np.concatenate((core_P,seg_core_P),axis=0)
            core_M          = np.concatenate((core_M,seg_core_M),axis=0)
            fan_T           = np.concatenate((fan_T ,seg_fan_T),axis=0)
            fan_P           = np.concatenate((fan_P ,seg_fan_P),axis=0)
            fan_M           = np.concatenate((fan_M ,seg_fan_M ),axis=0)
                
                        
                        
    return elapsed_time, core_T, core_P, core_M, fan_T,  fan_P, fan_M

# ------------------------------------------------------------------ 
# Truncate colormaps
# ------------------------------------------------------------------  
def truncate_colormap(cmap, minval=0.0, maxval=1.0, n=100):
    new_cmap = colors.LinearSegmentedColormap.from_list(
        'trunc({n},{a:.2f},{b:.2f})'.format(n=cmap.name, a=minval, b=maxval),
        cmap(np.linspace(minval, maxval, n)))
    return new_cmap
