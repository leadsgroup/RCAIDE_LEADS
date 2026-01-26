# RCAIDE/Methods/Aeroacoustics/Common/generate_microphone_locations.py
# 
# 
# Created:  Oct 2023, A. Molloy  

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports  
from RCAIDE.Framework.Core import Units 
from RCAIDE.Framework.Analyses.Geodesics.Geodesics import Calculate_Distance

# package imports 
from scipy.interpolate import griddata
import numpy as np 
 
# ---------------------------------------------------------------------------------------------------------------------- 
#  generate_terrain_microphone_locations
# ----------------------------------------------------------------------------------------------------------------------  
def generate_terrain_microphone_locations(settings):
    """This computes the absolute microphone/observer locations on a defined topography
            
    Assumptions: 
        topography_file is a text file obtained from https://topex.ucsd.edu/cgi-bin/get_data.cgi
    
    Source:
        N/A  

    Inputs:  
        topography_file                        - file of lattide, longitude and elevation points                                [-]
        origin_coordinates                  - coordinates of origin location                                              [degrees]
        destination_coordinates                - coordinates of destimation location                                            [degrees]
        microphone_x_resolution         - number of points on computational domain in latitudal direction                [-]
        microphone_y_resolution         - number of points on computational domain in  longitidinal direction            [-] 
        ground_microphone_x_stencil            - number of points in stencil in latitudal direction                             [-] 
        adjusted_cruise_distance                       - distance used to modify cruise to ensure desired range is met                  [-]
        ground_microphone_y_stencil            - number of points in stencil in in longitidinal direction                       [-] 
        
    Outputs: 
    topography_data.
        microphone_x_resolution         - number of points on computational domain in latitudal direction                [-] 
        microphone_y_resolution         - number of points on computational domain in  longitidinal direction            [-]
        ground_microphone_x_stencil            - number of points in stencil in latitudal direction                             [-]
        ground_microphone_y_stencil            - number of points in stencil in in longitidinal direction                       [-]       
        microphone_min_x                - x-location of start of computation domain                                      [meters]          
        microphone_max_x                - x-location of end of computation domain                                        [meters]  
        microphone_min_y                - y-location of start of computation domain                                      [meters]                
        microphone_max_y                - y-location of end of computation domain                                        [meters]  
        cartesian_micrphone_locations          - cartesian coordinates (x,y,z) of all microphones in domain                     [meters]       
        latitude_longitude_micrphone_locations - latitude-longitude and elevation coordinates of all microphones in domain      [deg,deg,m]   
    
    Properties Used:
        N/A       
    """     
    
    y_res = settings.microphone_y_resolution 
    x_res = settings.microphone_x_resolution 
    
    # extract data from file 
    data  = np.loadtxt(settings.topography_file) # settings.topography_file) CHANGED 10-15-2024
    Long  = data[:,0]
    Lat   = data[:,1]
    Elev  = data[:,2] 
    
    x_min_coord = np.min(Lat)
    x_max_coord = np.max(Lat)
    y_min_coord = np.min(Long)
    y_max_coord = np.max(Long) 

    if y_min_coord < 0: 
        y_min_coord = 360 + y_min_coord
    if y_max_coord< 0:
        y_max_coord=360 + y_max_coord 
    
    top_left_map_coords      = np.array([x_max_coord,y_min_coord])
    bottom_left_map_coords   = np.array([x_min_coord,y_min_coord])  
    bottom_right_map_coords  = np.array([x_min_coord,y_max_coord]) 
    
    x_dist_max = Calculate_Distance(top_left_map_coords,bottom_left_map_coords) * Units.kilometers
    y_dist_max = Calculate_Distance(bottom_right_map_coords,bottom_left_map_coords) * Units.kilometers
    
    [y_pts,x_pts]      = np.meshgrid(np.linspace(0,y_dist_max,y_res),np.linspace(0,x_dist_max,x_res))
    [long_deg,lat_deg] = np.meshgrid(np.linspace(np.min(Long),np.max(Long),y_res),np.linspace(np.min(Lat),np.max(Lat),x_res)) 
    z_deg              = griddata((Lat,Long), Elev, (lat_deg, long_deg), method='linear')        
    cartesian_pts      = np.dstack((np.dstack((x_pts[:,:,None],y_pts[:,:,None] )),z_deg[:,:,None])).reshape(x_res*y_res,3)
    lat_long_pts       = np.dstack((np.dstack((lat_deg[:,:,None],long_deg[:,:,None] )),z_deg[:,:,None])).reshape(x_res*y_res,3)  
    return cartesian_pts , lat_long_pts


