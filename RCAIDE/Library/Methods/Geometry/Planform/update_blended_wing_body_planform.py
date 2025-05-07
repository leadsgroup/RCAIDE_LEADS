# RCAIDE/Library/Methods/Geometry/Planform/update_blended_wing_body_planform.py
# 
# 
# Created:  Jul 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE 
from RCAIDE.Library.Methods.Geometry.Planform.convert_sweep import convert_sweep_segments 

# python imports 
import numpy as np  
from copy import deepcopy
from scipy.interpolate import interp1d
# ----------------------------------------------------------------------------------------------------------------------
# update_blended_wing_body_planform 
# ----------------------------------------------------------------------------------------------------------------------
def update_blended_wing_body_planform(bwb_wing):
    '''
    
    ''' 
    BWB_LOPA     = bwb_wing.layout_of_passenger_accommodations
    cabin_width  = BWB_LOPA.cabin_wdith / 2
    
    new_bwb_wing = deepcopy(bwb_wing)
    new_bwb_wing.segments.clear()
    
    seg_tags = list(bwb_wing.segments.keys())
    
    # ----------------------------------------------------------------------------------------------------------------------    
    # Step 1 loop through wing segments to check and make sure that the % location of the last 
    # BWB-fuselage segment is equal to the width of the lopa 
    # ----------------------------------------------------------------------------------------------------------------------
    # 1.1 get location of last BWB fuselage segment
    new_cabin_segs = 0
    max_cabin_seg_span = 0
    for seg_i , segment in enumerate(bwb_wing.segments):
        if isinstance(segment,RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment): 
            seg_span = segment.percent_span_location * bwb_wing.spans.projected / 2
            max_cabin_seg_span =  np.maximum(max_cabin_seg_span,seg_span)
        
    add_final_cabin_seg = True             
    if cabin_width == max_cabin_seg_span:
        pass
    
     # 1.2 update segments on BWB 
    for  seg_i , segment  in enumerate(bwb_wing.segments):
        seg_span = segment.percent_span_location * bwb_wing.spans.projected / 2
        if (cabin_width > seg_span): 
            if type(segment) == RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment:
                segment.structural.rib = True
                new_cabin_segs += 1
                new_bwb_wing.append_segment(segment) 
            
            elif type(segment) == RCAIDE.Library.Components.Wings.Segments.Segment:
                new_segment = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()  
                new_cabin_segs += 1
                new_segment.tag                    = 'fuselage_section'               
                new_segment.taper                  = segment.taper                        
                new_segment.twist                  = segment.twist                        
                new_segment.percent_span_location  = segment.percent_span_location        
                new_segment.root_chord_percent     = segment.root_chord_percent           
                new_segment.dihedral_outboard      = segment.dihedral_outboard            
                new_segment.thickness_to_chord     = segment.thickness_to_chord           
                new_segment.sweeps.leading_edge    = segment.sweeps.leading_edge          
                new_segment.append_airfoil(segment.airfoil )      
                new_bwb_wing.append_segment(new_segment)
            
        if  cabin_width < seg_span:
            if add_final_cabin_seg:
                cabin_wall_segment = bwb_wing.segments[seg_tags[seg_i-1]]
                new_segment = RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment()
                new_cabin_segs += 1  
                new_segment.structural.rib         = True
                new_segment.tag                    = "cabin_wall_section"                   
                new_segment.taper                  = cabin_wall_segment.taper                        
                new_segment.twist                  = cabin_wall_segment.twist                        
                new_segment.percent_span_location  = max_cabin_seg_span / bwb_wing.spans.projected      
                new_segment.root_chord_percent     = cabin_wall_segment.root_chord_percent           
                new_segment.dihedral_outboard      = cabin_wall_segment.dihedral_outboard            
                new_segment.thickness_to_chord     = cabin_wall_segment.thickness_to_chord           
                new_segment.sweeps.leading_edge    = cabin_wall_segment.sweeps.leading_edge          
                new_segment.append_airfoil(segment.airfoil )      
                new_bwb_wing.append_segment(new_segment)                    
                 
                add_final_cabin_seg = False  
            
            if type(segment) == RCAIDE.Library.Components.Wings.Segments.Segment:
                new_bwb_wing.append_segment(segment)
                
            elif type(segment) == RCAIDE.Library.Components.Wings.Segments.Blended_Wing_Body_Fuselage_Segment: 
                new_segment = RCAIDE.Library.Components.Wings.Segments.Segment()   
                new_segment.tag                    = 'wing_section'                    
                new_segment.taper                  = segment.taper                        
                new_segment.twist                  = segment.twist                        
                new_segment.percent_span_location  = segment.percent_span_location        
                new_segment.root_chord_percent     = segment.root_chord_percent           
                new_segment.dihedral_outboard      = segment.dihedral_outboard            
                new_segment.thickness_to_chord     = segment.thickness_to_chord           
                new_segment.sweeps.leading_edge    = segment.sweeps.leading_edge          
                new_segment.append_airfoil(segment.airfoil )      
                new_bwb_wing.append_segment(new_segment)                    
          
    # ----------------------------------------------------------------------------------------------------------------------    
    # Step 2: Update sweep and chord based on lopa. outer sweeps not updated
    # ----------------------------------------------------------------------------------------------------------------------
 
    cc       = BWB_LOPA.cabin_area_coordinates
    cc_front = cc[0:int(len(cc[0]/2)),:]
    cc_rear  = cc[int(len(cc[0]/2)):,:]
    
    cc_front = np.vstack((np.array([0, 0]), cc_front))
    cc_rear  = np.vstack(( cc_rear, np.array([cc_rear[1,0], 0])))
    
    inner_origin_x = 0
    outer_origin_x = 0
    new_seg_tags = list(new_bwb_wing.segments.keys())
    for  i in range(new_cabin_segs-1): 
        inner_segment = new_bwb_wing.segments[new_seg_tags[i]] 
        outer_segment = new_bwb_wing.segments[new_seg_tags[i+1]]  
    
        # nondimensionl front spar location coordinates  
        inner_rs_nondim_x       = inner_segment.structural.rear_spar_percent_chord  
        outer_rs_nondim_x       = outer_segment.structural.rear_spar_percent_chord   
        
        # inner segment 
        inner_cabin_start_percent_x      = inner_segment.percent_chord_cabin_start 
        inner_seg_span                   = inner_segment.percent_span_location * new_bwb_wing.spans.projected /2 
        inner_section_start_x_location   = interp1d(cc_front[:, 1], cc_front[:, 0]) # function representing leading edge of lopa 
        inner_section_end_x_location     = interp1d(cc_rear[:, 1], cc_rear[:, 0])   # function representing trailing edge of lopa 
        inner_sectional_cabin_chord      = inner_section_end_x_location(inner_seg_span) -  inner_section_start_x_location(inner_seg_span) # length of lopa at inner segment span percent location 

        # outer segment 
        outer_cabin_start_percent_x      = outer_segment.percent_chord_cabin_start 
        outer_seg_span                   = outer_segment.percent_span_location * new_bwb_wing.spans.projected /2 
        outer_section_end_x_location     = interp1d( cc_rear[:, 1],cc_rear[:, 0])  # function representing leading edge of lopa 
        outer_section_start_x_location   = interp1d(cc_front[:, 1],cc_front[:, 0]) # function representing trailing edge of lopa
        outer_sectional_cabin_chord      = outer_section_end_x_location(outer_seg_span) -  outer_section_start_x_location(outer_seg_span) # length of lopa at outer segment span percent location   
                             
        inner_chord =  (inner_sectional_cabin_chord / (inner_rs_nondim_x-inner_cabin_start_percent_x ))
        outer_chord =  (outer_sectional_cabin_chord / (outer_rs_nondim_x-outer_cabin_start_percent_x ))
         
        # update percent root chord values and the location of the lopa 
        if i == 0: 
            bwb_wing.chords.root   =  inner_chord 
            delta_x_0              =  inner_chord *inner_cabin_start_percent_x 
            outer_origin_x         += delta_x_0
            
        inner_segment.root_chord_percent = inner_chord / bwb_wing.chords.root
        outer_segment.root_chord_percent = outer_chord / bwb_wing.chords.root
        
        # update sweep 
        outer_origin_x  = delta_x_0 + outer_section_start_x_location(outer_seg_span) - (outer_chord *outer_cabin_start_percent_x ) 
        sweep           = np.pi /2 -  np.arctan((outer_seg_span-inner_seg_span)/(outer_origin_x-inner_origin_x))
        
        inner_segment.sweeps.leading_edge = sweep 
        inner_segment.sweeps.quarter_chord = convert_sweep_segments(sweep, inner_segment, outer_segment, new_bwb_wing, old_ref_chord_fraction=0.0, new_ref_chord_fraction=0.25) 
        inner_origin_x  = outer_origin_x 
     
    return 
    