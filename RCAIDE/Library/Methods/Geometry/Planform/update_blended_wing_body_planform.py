# RCAIDE/Library/Methods/Geometry/Planform/update_blended_wing_body_planform.py
# 
# 
# Created:  Jul 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from RCAIDE.Framework.Core import Units , Data 
from RCAIDE.Library.Methods.Geometry.Airfoil import import_airfoil_geometry,  compute_naca_4series 

# python imports 
import numpy as np  
from copy import deepcopy
# ----------------------------------------------------------------------------------------------------------------------
# update_blended_wing_body_planform 
# ----------------------------------------------------------------------------------------------------------------------
def update_blended_wing_body_planform(bwb_wing):
    '''
    
    '''

    BWB_LOPA = bwb_wing.layout_of_passenger_accommodations    
      
    # Create lopa with offset included
    #first_segment = list(bwb_wing.segments.keys())[0]
    #cabin_offset  = bwb_wing.segments[first_segment.tag].cabin_offset 
    
    
    # get with of lopa (will be used to check that no wing segment are specified where fuselage segments should be and vice versa)
    #cabin_width = bwb_wing.width 
    
    # loop through wings segments and update (sweep) based of offset
    
    for segment , i in enumerate(bwb_wing.segments):  
        # import segment airfoil 
        if segment.airfoil != None: 
            if type(segment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                geometry = compute_naca_4series(segment.airfoil.NACA_4_Series_code)
            elif type(segment.airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil: 
                geometry = import_airfoil_geometry(segment.airfoil.coordinate_file)
        else:
            geometry = compute_naca_4series('0012')
    
        # nondimensionl front spar location coordinates 
        fs_nondim_x       = segment.structural.front_spar_percent_chord   
        rs_nondim_x       = segment.structural.rear_spar_percent_chord     
        fs_nondim_y_upper = np.interp1d(fs_nondim_x, geometry.x_upper_surface  ,geometry.y_upper_surface ) 
        rs_nondim_y_upper = np.interp1d(rs_nondim_x, geometry.x_upper_surface  ,geometry.y_upper_surface )     
        fs_nondim_y_lower = np.interp1d(fs_nondim_x, geometry.x_lower_surface   , geometry.y_lower_surface)     
        rs_nondim_y_lower = np.interp1d(rs_nondim_x, geometry.x_lower_surface   , geometry.y_lower_surface)
        
        cabin_start_percent_x  = segment.percent_chord_cabin_start 
        
        cc =  fuselage.cabin_coordinates
        cc_front = cc[:, 0:int(len(cc/2))]
        cc_rear  = cc[:, int(len(cc/2)):]
        
        segment_y = segment.percent_span_location * bwb_wing.spans.projected
        
        section_end_x_location     = np.interp1d(segment_y, cc_rear[0],cc_rear[0])
        section_start_x_location   = np.interp1d(segment_y , cc_front[0],cc_front[0])
        sectional_cabin_chord      = section_end_x_location =  section_start_x_location 
                             
        l =  sectional_cabin_chord / (rs_nondim_x-cabin_start_percent_x ) 
        dimensional_multiplier = l /(rs_nondim_x - rs_nondim_x)
        
        
        # update percent root chord values
        if i == 0: 
            bwb_wing.chords.root =  dimensional_multiplier
        else:
            segment.percent_root_chord = dimensional_multiplier / bwb_wing.chords.root

        # update span of previous segment         
        if i > 0:
            prev_seg = bwb_wing.segments[i-1]
        
        
     
    # return 
    