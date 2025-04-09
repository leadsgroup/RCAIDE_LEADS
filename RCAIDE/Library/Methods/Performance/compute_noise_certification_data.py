# RCAIDE/Library/Methods/Performance/compute_noise_certification_data.py
# 
# 
# Created:  Jul 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------

# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core import Units , Data   
import matplotlib.colors
import matplotlib.colors as colors
from RCAIDE.Library.Plots import *
 
# Pacakge imports 
import numpy as np
from matplotlib import pyplot as plt
import plotly.graph_objects as go
 
# ----------------------------------------------------------------------
#  Compute Aircraft Noise Certification Data  
# ----------------------------------------------------------------------  
def compute_noise_certification_data(approach_mission  = None,
                                     takeoff_mission   = None,
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
    """Calculates the noise at certification points as well as the noise contours of approach and takeoff.
    A combined approach-takeoff noisec contour is also created 
    """ 
            
    if approach_mission == None:
        raise AssertionError('Approach mission not specifed!')
    if takeoff_mission == None:
        raise AssertionError('Takeoff mission not specifed!')
     
    microphone_x_resolution                = 401 
    microphone_y_resolution                = 9  
    noise_times_steps                      = 51 
    number_of_microphone_in_stencil        = 1800
    
    # update weights analysis
    for segment in approach_mission.segments:
        if segment.analyses.noise == None:
            raise AssertionError('Noise analysis not specifed!')
        noise_analysis = segment.analyses.noise
        noise_analysis.settings.microphone_x_resolution                = microphone_x_resolution
        noise_analysis.settings.microphone_y_resolution                = microphone_y_resolution
        noise_analysis.settings.noise_times_steps                      = noise_times_steps
        noise_analysis.settings.number_of_microphone_in_stencil        = number_of_microphone_in_stencil
        noise_analysis.settings.microphone_min_y                       = 1E-6   
        noise_analysis.settings.microphone_max_y                       = 1800
        noise_analysis.settings.microphone_min_x                       = 1E-6   
        noise_analysis.settings.microphone_max_x                       = 8000
    
    # update weights analysis
    for segment in takeoff_mission.segments:
        if segment.analyses.noise == None:
            raise AssertionError('Noise analysis not specifed!')
        noise_analysis = segment.analyses.noise 
        noise_analysis.settings.microphone_x_resolution                = microphone_x_resolution
        noise_analysis.settings.microphone_y_resolution                = microphone_y_resolution
        noise_analysis.settings.noise_times_steps                      = noise_times_steps
        noise_analysis.settings.number_of_microphone_in_stencil        = number_of_microphone_in_stencil
        noise_analysis.settings.microphone_min_y                       = 1E-6   
        noise_analysis.settings.microphone_max_y                       = 1800
        noise_analysis.settings.microphone_min_x                       = -2000 + 1E-6  
        noise_analysis.settings.microphone_max_x                       = 6000
    
    # evaluate both missions
    approach_results = approach_mission.evaluate() 
    takeoff_results  = takeoff_mission.evaluate()
    
    # post process results
    noise_data  =  post_process_certification_noise_data(approach_results,takeoff_results)
    
    # plot diagram
    if show_figure:  
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
        
    return noise_data 
    
def post_process_certification_noise_data(approach_results,takeoff_results): 
   
    approach_noise_data   = post_process_noise_data(approach_results)
    takeoff_noise_data    = post_process_noise_data(takeoff_results) 
    
    # append approach noise                                
    approach_pos         = approach_noise_data.aircraft_position
    approach_pos[:,0]   -= 2000 
    
    # append takeoff noise  
    cert_SPL_dBA_max  = np.max(np.concatenate((approach_noise_data.SPL_dBA,takeoff_noise_data.SPL_dBA), axis = 0) ,axis = 0)                      
    cert_pos          = np.concatenate((approach_pos, takeoff_noise_data.aircraft_position), axis = 0)     
    cert_mic_locs     = takeoff_noise_data.microphone_locations 
     
    res = Data(
        certification_SPL_dBA_max         = cert_SPL_dBA_max, 
        certification_trajectory          = cert_pos,     
        certification_microphone_locations= cert_mic_locs,

        approach_SPL_dBA_max              = np.max(approach_noise_data.SPL_dBA,axis = 0),
        approach_SPL_dBA                  = approach_noise_data.SPL_dBA, 
        approach_time                     = approach_noise_data.time,   
        approach_trajectory               = approach_pos,     
        approach_microphone_locations     = takeoff_noise_data.microphone_locations, 

        takeoff_SPL_dBA_max               = np.max(takeoff_noise_data.SPL_dBA,axis = 0),
        takeoff_SPL_dBA                   = takeoff_noise_data.SPL_dBA, 
        takeoff_time                      = takeoff_noise_data.time,   
        takeoff_trajectory                = takeoff_noise_data.aircraft_position,     
        takeoff_microphone_locations      = takeoff_noise_data.microphone_locations     
    
    )
    
    print('Certification Noise')
    print('-----------------------------------------------')
    print('2000 m  Approach Noise   :', round(cert_SPL_dBA_max[0, 0], 2)) 
    print('6000 m  Flyover Noise    :', round(cert_SPL_dBA_max[-1, 0], 2))
    print('450  m  Sideline Noise   :', round(max(cert_SPL_dBA_max[:, 2]), 2))
    
    area_85_dbA =  len(np.where(cert_SPL_dBA_max.flatten()>85)[0]) *100 / len(cert_SPL_dBA_max.flatten()) 
    area_65_dbA =  len(np.where(cert_SPL_dBA_max.flatten()>65)[0]) *100 / len(cert_SPL_dBA_max.flatten())
    print('% Area > 85 dBA Threshold:', round(area_85_dbA, 2)) 
    print('% Area > 65 dBA Threshold:', round(area_65_dbA, 2))
    
    res.approach_noise_2000m = cert_SPL_dBA_max[0, 0] 
    res.flyover_noise_6000m  = cert_SPL_dBA_max[-1, 0] 
    res.sideline_noise_450m  = max(cert_SPL_dBA_max[:, 2])
    res.area_65_dbA = area_65_dbA
    res.area_85_dbA = area_85_dbA 
    
    return res


# ------------------------------------------------------------------ 
# Truncate colormaps
# ------------------------------------------------------------------  
def truncate_colormap(cmap, minval=0.0, maxval=1.0, n=100):
    new_cmap = colors.LinearSegmentedColormap.from_list(
        'trunc({n},{a:.2f},{b:.2f})'.format(n=cmap.name, a=minval, b=maxval),
        cmap(np.linspace(minval, maxval, n)))
    return new_cmap
