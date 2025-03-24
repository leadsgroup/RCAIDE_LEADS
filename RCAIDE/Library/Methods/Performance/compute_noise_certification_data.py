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
from RCAIDE.Library.Plots.Common import set_axes, plot_style    
from RCAIDE.Library.Plots import *
 
# Pacakge imports 
import numpy as np
from matplotlib import pyplot as plt
 
# ----------------------------------------------------------------------
#  Compute Aircraft Noise Certification Data  
# ----------------------------------------------------------------------  
def compute_noise_certification_data(approach_mission = None, takeoff_mission = None, plot_diagram = True):  
    """Calculates the noise at certification points as well as the noise contours of approach and takeoff.
    A combined approach-takeoff noisec contour is also created 
    """ 
            
    if approach_mission == None:
        raise AssertionError('Approach mission not specifed!')
    if takeoff_mission == None:
        raise AssertionError('Takeoff mission not specifed!')
    
    # update weights analysis
    for segment in approach_mission.segments:
        if segment.analyses.noise == None:
            raise AssertionError('Noise analysis not specifed!')
        noise_analysis = segment.analyses.noise
        noise_analysis.settings.microphone_x_resolution                = 10 #8
        noise_analysis.settings.microphone_y_resolution                = 3 #5 
        noise_analysis.settings.noise_times_steps                      = 20 #101 
        noise_analysis.settings.number_of_microphone_in_stencil        =  10 #30
        noise_analysis.settings.microphone_min_y                       = 0  
        noise_analysis.settings.microphone_max_y                       = 900
        noise_analysis.settings.microphone_min_x                       = 3500
        noise_analysis.settings.microphone_max_x                       = 1000 
    
    
    # update weights analysis
    for segment in takeoff_mission.segments:
        if segment.analyses.noise == None:
            raise AssertionError('Noise analysis not specifed!')
        noise_analysis = segment.analyses.noise 
        noise_analysis.settings.microphone_x_resolution                = 10 # 15
        noise_analysis.settings.microphone_y_resolution                = 3 # 5 
        noise_analysis.settings.noise_times_steps                      = 20 #101 
        noise_analysis.settings.number_of_microphone_in_stencil        = 10 #30
        noise_analysis.settings.microphone_min_y                       = 0  
        noise_analysis.settings.microphone_max_y                       = 900
        noise_analysis.settings.microphone_min_x                       = 7000
        noise_analysis.settings.microphone_max_x                       = 1000
    
    # evaluate both missions
    approach_results = approach_mission.evaluate()
    takeoff_results  = takeoff_mission.evaluate()
    
    # post process results
    noise_data  =  post_process_certification_noise_data(approach_results,takeoff_results)
    
    # plot diagram
    if plot_diagram:
        noise_data.coordinates 
        noise_data.coordinates
        
    return noise_data 
    
def post_process_certification_noise_data(approach_results,takeoff_results):
    
    res = Data()

    approach_noise_data   = post_process_noise_data(approach_results)
    takeoff_noise_data   = post_process_noise_data(takeoff_results)
        
    
    # cooridate points

    
    # shift approach coordinates back by 
    
    # approach contour
    
    # takeoff contour
    
    # combined contour 
    
    return res