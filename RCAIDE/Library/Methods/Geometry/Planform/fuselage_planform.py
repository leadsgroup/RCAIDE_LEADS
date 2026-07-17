# RCAIDE/Library/Methods/Geometry/Platform.py
# 
# 
# Created:  Apr 2023, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------
#  Methods
# ----------------------------------------------------------------------
def fuselage_planform(fuselage, circular_cross_section = True):
    """Calculates fuselage geometry values

    Assumptions:
    None

    Source:
    http://adg.stanford.edu/aa241/drag/wettedarea.html

    Inputs:
    fuselage.
      num_coach_seats       [-] 
      fineness.nose         [-]
      fineness.tail         [-] 
      width                 [m]
      heights.maximum       [m]

    Outputs:
    fuselage.
      lengths.nose          [m]
      lengths.tail          [m]
      lengths.cabin         [m]
      lengths.total         [m]
      areas.wetted          [m]
      areas.front_projected [m]
      effective_diameter    [m]

    Properties Used:
    N/A
    """ 
    fuselage_width  = fuselage.width  
    nose_length     = fuselage.fineness.nose * fuselage_width
    tail_length     = fuselage.fineness.tail  * fuselage_width 
    cabin_length    = fuselage.lengths.total -  nose_length - tail_length  
    fuselage_height = fuselage.heights.maximum 
    
    a     = fuselage_width/2.  # base semi-major axis  
    b     = fuselage_height/2. # base semi-minor axis  
    R     = (a-b)/(a+b) 

    side_projected_area  = 0 
    wetted_area          = 0   
    front_projected_area = 0 
    effective_diameter   = 0 
    
    if len(fuselage.segments) > 2:
        f_segs = list(fuselage.segments.keys())
        for i in range(len(fuselage.segments)-1):
            
            seg_1 = fuselage.segments[f_segs[i]]
            seg_2 = fuselage.segments[f_segs[i+1]]
              
            delta_x                = fuselage.lengths.total * (seg_2.percent_x_location -  seg_1.percent_x_location)
            side_projected_area   += ((seg_1.height +  seg_2.height ) / 2) * delta_x 

            area = truncated_elliptic_cone_lateral_area(seg_1.width/2, seg_1.height/2, seg_2.width/2, seg_2.height/2, delta_x)
            wetted_area += area

            A_1  = np.pi *  (seg_1.height / 2) *  (seg_1.width / 2)   
            A_2  = np.pi *  (seg_2.height / 2) *  (seg_2.width / 2)    
            front_projected_area  = np.maximum(front_projected_area,np.maximum(A_1,A_2)  )
    else:    
        side_projected_area  = fuselage.heights.maximum * fuselage.lengths.total  
        wetted_area          = np.pi*a*(a+ np.sqrt( fuselage.lengths.nose **2 +(a)**2)) + \
                               np.pi*a*(a+ np.sqrt( fuselage.lengths.tail**2 +(a)**2))+ \
                               np.pi * fuselage.width * ( fuselage.lengths.total - (fuselage.lengths.tail+ fuselage.lengths.nose))  
        front_projected_area = np.pi * a *  b
        
    effective_diameter             = ((fuselage_width/2)+(fuselage_height/2.))*(64.-3.*R**4)/(64.-16.*R**2)  
    fuselage.lengths.nose          = nose_length
    fuselage.lengths.tail          = tail_length
    fuselage.lengths.cabin         = cabin_length 
    fuselage.areas.wetted          = wetted_area
    fuselage.areas.front_projected = front_projected_area
    fuselage.areas.side_projected  = side_projected_area 
    fuselage.effective_diameter    = effective_diameter 

    return


def truncated_elliptic_cone_lateral_area(a, b, c, d, h):
    
    s_major = np.sqrt(h**2 + (a - c)**2)  # slant length in major axis direction
    s_minor = np.sqrt(h**2 + (b - d)**2)  # slant length in minor axis direction
    wetted_area = np.pi * ((a + c)/2) * s_major + np.pi * ((b + d)/2) * s_minor
    return wetted_area
