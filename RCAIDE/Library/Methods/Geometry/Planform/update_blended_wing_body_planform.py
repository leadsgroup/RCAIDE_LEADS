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
from RCAIDE.Library.Methods.Geometry.Planform.convert_sweep import convert_sweep_segments 

# python imports 
import numpy as np  
from copy import deepcopy
# ----------------------------------------------------------------------------------------------------------------------
# update_blended_wing_body_planform 
# ----------------------------------------------------------------------------------------------------------------------
def update_blended_wing_body_planform(bwb_wing):
    '''
    
    '''

    BWB_LOPA     = bwb_wing.layout_of_passenger_accommodations
    cabin_width  = BWB_LOPA.width
    
    new_bwb_wing = deepcopy(bwb_wing)
    new_bwb_wing.segments.clear()
    
    seg_tags = list(bwb_wing.segments.keys())
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # Step 1 loop through wing segments to check and make sure that the % location of the last 
    # BWB-fuselage segment is equal to the width of the lopa 
    # ----------------------------------------------------------------------------------------------------------------------
    # 1.1 get location of last BWB fuselage segment 
    max_cabin_seg_span = 0
    for segment , seg_i in enumerate(bwb_wing.segments):
        seg_span = segment.percent_span_location * bwb_wing.spans.projected
        if type(segment) == RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment: 
            max_cabin_seg_span =  np.maximum(max_cabin_seg_span,seg_span)
        
    add_final_cabin_seg = True             
    if cabin_width == max_cabin_seg_span:
        pass
    
     # 1.2 update segments on BWB 
    for segment , seg_i in enumerate(bwb_wing.segments):
        seg_span = segment.percent_span_location * bwb_wing.spans.projected 
        if (cabin_width > seg_span): 
            if type(segment) == RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment:
                segment.structural.rib = True
                new_bwb_wing.append(segment) 
            
            elif type(segment) == RCAIDE.Library.Components.Wings.Segments.Segment:
                new_segment = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()  
                new_segment.tag                    = segment.tag                          
                new_segment.taper                  = segment.taper                        
                new_segment.twist                  = segment.twist                        
                new_segment.percent_span_location  = segment.percent_span_location        
                new_segment.root_chord_percent     = segment.root_chord_percent           
                new_segment.dihedral_outboard      = segment.dihedral_outboard            
                new_segment.thickness_to_chord     = segment.thickness_to_chord           
                new_segment.sweeps.leading_edge    = segment.sweeps.leading_edge          
                new_segment.append_airfoil(segment.airfoil )      
                new_bwb_wing.append(segment)
            
        if  cabin_width < seg_span:
            if add_final_cabin_seg:
                cabin_wall_segment = bwb_wing.segments[seg_tags[i-1]]
                new_segment = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()  
                new_segment.structural.rib = True
                new_segment.tag                    = cabin_wall_segment.tag                          
                new_segment.taper                  = cabin_wall_segment.taper                        
                new_segment.twist                  = cabin_wall_segment.twist                        
                new_segment.percent_span_location  = max_cabin_seg_span / bwb_wing.spans.projected      
                new_segment.root_chord_percent     = cabin_wall_segment.root_chord_percent           
                new_segment.dihedral_outboard      = cabin_wall_segment.dihedral_outboard            
                new_segment.thickness_to_chord     = cabin_wall_segment.thickness_to_chord           
                new_segment.sweeps.leading_edge    = cabin_wall_segment.sweeps.leading_edge          
                new_segment.append_airfoil(segment.airfoil )      
                new_bwb_wing.append(segment)                    
                 
                add_final_cabin_seg = False  
            
            if type(segment) == RCAIDE.Library.Components.Wings.Segments.Segment:
                new_bwb_wing.append(segment)
                
            elif type(segment) == RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment: 
                new_segment = RCAIDE.Library.Components.Wings.Segments.Segment()   
                new_segment.tag                    = segment.tag                          
                new_segment.taper                  = segment.taper                        
                new_segment.twist                  = segment.twist                        
                new_segment.percent_span_location  = segment.percent_span_location        
                new_segment.root_chord_percent     = segment.root_chord_percent           
                new_segment.dihedral_outboard      = segment.dihedral_outboard            
                new_segment.thickness_to_chord     = segment.thickness_to_chord           
                new_segment.sweeps.leading_edge    = segment.sweeps.leading_edge          
                new_segment.append_airfoil(segment.airfoil )      
                new_bwb_wing.append(segment)                    
          
    # ----------------------------------------------------------------------------------------------------------------------    
    # Step 2: Update sweep and chord based on lopa 
    # ----------------------------------------------------------------------------------------------------------------------
 
    cc       = BWB_LOPA.cabin_area_coordinates
    cc_front = cc[:, 0:int(len(cc/2))]
    cc_rear  = cc[:, int(len(cc/2)):]
    
    inner_origin_x = 0
    outer_origin_x = 0
    for  i in range(len(new_bwb_wing.segments)-1):
        inner_segment = new_bwb_wing.segments[seg_tags[i]]
        outer_segment = new_bwb_wing.segments[seg_tags[i+1]]
        
        delta_y = (outer_segment.spans.projected - inner_segment.spans.projected) /2
        
        # import inner_segment airfoil 
        if inner_segment.airfoil != None: 
            if type(inner_segment.airfoil) == RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil:
                geometry = compute_naca_4series(inner_segment.airfoil.NACA_4_Series_code)
            elif type(inner_segment.airfoil) == RCAIDE.Library.Components.Airfoils.Airfoil: 
                geometry = import_airfoil_geometry(inner_segment.airfoil.coordinate_file)
        else:
            geometry = compute_naca_4series('0012')
    
        # nondimensionl front spar location coordinates 
        fs_nondim_x       = inner_segment.structural.front_spar_percent_chord   
        rs_nondim_x       = inner_segment.structural.rear_spar_percent_chord     
        fs_nondim_y_upper = np.interp1d(fs_nondim_x, geometry.x_upper_surface  ,geometry.y_upper_surface ) 
        rs_nondim_y_upper = np.interp1d(rs_nondim_x, geometry.x_upper_surface  ,geometry.y_upper_surface )     
        fs_nondim_y_lower = np.interp1d(fs_nondim_x, geometry.x_lower_surface   , geometry.y_lower_surface)     
        rs_nondim_y_lower = np.interp1d(rs_nondim_x, geometry.x_lower_surface   , geometry.y_lower_surface)
        
        
        # inner segment 
        inner_cabin_start_percent_x      = inner_segment.percent_chord_cabin_start 
        inner_seg_span                   = inner_segment.percent_span_location * new_bwb_wing.spans.projected 
        inner_section_end_x_location     = np.interp1d(inner_seg_span, cc_rear[0],cc_rear[0])
        inner_section_start_x_location   = np.interp1d(inner_seg_span , cc_front[0],cc_front[0])
        inner_sectional_cabin_chord      = inner_section_end_x_location -  inner_section_start_x_location 

        # inner segment 
        outer_cabin_start_percent_x      = outer_segment.percent_chord_cabin_start 
        outer_seg_span                   = outer_segment.percent_span_location * new_bwb_wing.spans.projected 
        outer_section_end_x_location     = np.interp1d(outer_seg_span, cc_rear[0],cc_rear[0])
        outer_section_start_x_location   = np.interp1d(outer_seg_span , cc_front[0],cc_front[0])
        outer_sectional_cabin_chord      = outer_section_end_x_location -  outer_section_start_x_location 
                             
        inner_l =  inner_sectional_cabin_chord / (rs_nondim_x-inner_cabin_start_percent_x ) 
        outer_l =  outer_sectional_cabin_chord / (rs_nondim_x-outer_cabin_start_percent_x )
        
        inner_chord = inner_l /(rs_nondim_x - rs_nondim_x)
        outer_chord = outer_l /(rs_nondim_x - rs_nondim_x)
        
        # update percent root chord values and the location of the lopa 
        if i == 0: 
            bwb_wing.chords.root       =  inner_chord
            delta_x_0                  =  inner_sectional_cabin_chord *inner_chord
            BWB_LOPA[:, 2]             += delta_x_0
            outer_origin_x             += delta_x_0
        segment.percent_root_chord = inner_chord / bwb_wing.chords.root
        
        # update sweep 
        outer_origin_x += outer_section_end_x_location - (outer_chord *outer_cabin_start_percent_x ) 
        sweep           = np.arctan(delta_y/(outer_origin_x-inner_origin_x)) 
        inner_segment.sweeps.leading_edge = sweep 
        inner_segment.sweeps.quarter_chord = convert_sweep_segments(sweep, inner_segment, outer_segment, new_bwb_wing, old_ref_chord_fraction=0.0, new_ref_chord_fraction=0.25) 
        inner_origin_x  = outer_origin_x 
         
    # ----------------------------------------------------------------------------------------------------------------------      
    # Step 3. Determine fuel tank capacity
    # ----------------------------------------------------------------------------------------------------------------------  
    
     
    return 
    