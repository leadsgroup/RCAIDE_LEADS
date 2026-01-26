# RCAIDE imports  
import matplotlib.colors 
import matplotlib.colors as colors
from RCAIDE.Library.Plots import *
 
# Pacakge imports 
import numpy as np
from matplotlib import pyplot as plt

# ----------------------------------------------------------------------
#  Plot Aircraft Noise Certification Data  
# ----------------------------------------------------------------------  
def plot_noise_certification_contour( noise_data,
                                    noise_level       = None, 
                                    min_noise_level   = 45,  
                                    max_noise_level   = 105, 
                                    noise_scale_label = "Max. SPL [dbA]",
                                    save_figure       = False,
                                    show_figure       = True,
                                    save_filename     = "Certification_Noise", 
                                    colormap          = 'jet',
                                    file_type         = ".png",
                                    width             = 12, 
                                    height            = 6):
     
      
    fig = plt.figure(save_filename)
    fig.set_size_inches(width,height) 
    
    noise_levels   = np.linspace(min_noise_level,max_noise_level,7)  
    noise_cmap     = plt.get_cmap('turbo')
    noise_new_cmap = truncate_colormap(noise_cmap,0.0, 1.0) 

    noise_level = noise_data.certification_SPL_dBA_max           
    X           = noise_data.certification_microphone_locations[:,:,0]
    Y           = noise_data.certification_microphone_locations[:,:,1] 
    
    ap_noise_level = noise_data.approach_SPL_dBA_max           
    ap_X           = noise_data.approach_microphone_locations[:,:,0]
    ap_Y           = noise_data.approach_microphone_locations[:,:,1]
    ap_POS         = noise_data.approach_trajectory

    to_noise_level = noise_data.takeoff_SPL_dBA_max           
    to_X           = noise_data.takeoff_microphone_locations[:,:,0]
    to_Y           = noise_data.takeoff_microphone_locations[:,:,1]
    to_POS         = noise_data.takeoff_trajectory
    
    axis_0   = fig.add_subplot(2,2,1) 
    axis_0.set_xlabel('x [m]')
    axis_0.set_ylabel('altitude [m]')   
    axis_1   = fig.add_subplot(2,2,3) 
    axis_1.set_xlabel('x [m]')
    axis_1.set_ylabel('y [m]')   
    axis_2   = fig.add_subplot(2,2,2) 
    axis_2.set_xlabel('x [m]')
    axis_2.set_ylabel('y [m]')   
    axis_3   = fig.add_subplot(2,2,4) 
    axis_3.set_xlabel('x [m]')
    axis_3.set_ylabel('y [m]')
    
    # plot aircraft position
    axis_0.plot(ap_POS[:,0],-ap_POS[:,2], color = 'black', linestyle = '-' , marker = 'o', linewidth = 2, label= "Approach")
    axis_0.plot(to_POS[:,0],-to_POS[:,2], color = 'blue', linestyle = '-' , marker = 's', linewidth = 2, label= "Takeoff")
    axis_0.legend(loc='upper center') 

    # plot aircraft noise levels   
    CS_11    = axis_1.contourf(X,Y,noise_level ,noise_levels,cmap = noise_new_cmap,extend='both')  
    CS_12    = axis_1.contourf(X,-Y,noise_level ,noise_levels,cmap = noise_new_cmap,extend='both')
    CS_21    = axis_2.contourf(ap_X,ap_Y,ap_noise_level ,noise_levels,cmap = noise_new_cmap,extend='both')  
    CS_22    = axis_2.contourf(ap_X,-ap_Y,ap_noise_level ,noise_levels,cmap = noise_new_cmap,extend='both')
    CS_31    = axis_3.contourf(to_X,to_Y,to_noise_level ,noise_levels,cmap = noise_new_cmap,extend='both')  
    CS_32    = axis_3.contourf(to_X,-to_Y,to_noise_level ,noise_levels,cmap = noise_new_cmap,extend='both')
     
    cbar  = fig.colorbar(CS_11, ax=axis_1)        
    cbar.ax.set_ylabel(noise_scale_label, rotation =  90)

    cbar  = fig.colorbar(CS_11, ax=axis_2)        
    cbar.ax.set_ylabel(noise_scale_label, rotation =  90)

    cbar  = fig.colorbar(CS_11, ax=axis_3)        
    cbar.ax.set_ylabel(noise_scale_label, rotation =  90)     

    run_way_x_pts = np.linspace(0, 3000, 20)
    run_way_y_pts = run_way_x_pts * 0

    axis_1.plot(run_way_x_pts,run_way_y_pts, color = 'grey', linestyle = '-' , linewidth = 5 , alpha=0.5)
    axis_2.plot(run_way_x_pts,run_way_y_pts, color = 'grey', linestyle = '-' , linewidth = 5 , alpha=0.5)
    axis_3.plot(run_way_x_pts,run_way_y_pts, color = 'grey', linestyle = '-' , linewidth = 5 , alpha=0.5)
    
    set_axes(axis_1)
    set_axes(axis_2)
    set_axes(axis_3)

    axis_0.set_title('Flight Trajectory')        
    axis_1.set_title('Approach and Takeoff Noise')
    axis_2.set_title('Approach Noise')
    axis_3.set_title('Takeoff Noise')
    
    fig.tight_layout()  
    if save_figure: 
        figure_title  = save_filename
        plt.savefig(figure_title + file_type )   
    

# ------------------------------------------------------------------ 
# Truncate colormaps
# ------------------------------------------------------------------  
def truncate_colormap(cmap, minval=0.0, maxval=1.0, n=100):
    new_cmap = colors.LinearSegmentedColormap.from_list(
        'trunc({n},{a:.2f},{b:.2f})'.format(n=cmap.name, a=minval, b=maxval),
        cmap(np.linspace(minval, maxval, n)))
    return new_cmap
        