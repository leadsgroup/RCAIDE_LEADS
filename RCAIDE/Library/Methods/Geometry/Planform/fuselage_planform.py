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
     
    nose_fineness   = fuselage.fineness.nose
    tail_fineness   = fuselage.fineness.tail
    
    fuselage_width  = fuselage.width 
        
    nose_length     = nose_fineness * fuselage_width
    tail_length     = tail_fineness * fuselage_width 
    cabin_length    = fuselage.lengths.total -  nose_length - tail_length  
    
    fuselage_height =  fuselage.heights.maximum
    if fuselage.heights.maximum == 0:
        fuselage.heights.maximum = fuselage_width
        fuselage_height = fuselage_width 
    
    a     = fuselage_width/2.
    b     = fuselage_height/2. 
    R     = (a-b)/(a+b)   
        
    side_projected_area  = fuselage.heights.maximum * fuselage.lengths.total  
    wetted_area          = np.pi*a*(a+ np.sqrt( fuselage.lengths.nose **2 +(a)**2)) + \
                           np.pi*a*(a+ np.sqrt( fuselage.lengths.tail**2 +(a)**2))+ \
                           np.pi * fuselage.width * ( fuselage.lengths.total - (fuselage.lengths.tail+ fuselage.lengths.nose))  
    front_projected_area = np.pi * a *  b
    effective_diameter   = ((fuselage_width/2)+(fuselage_height/2.))*(64.-3.*R**4)/(64.-16.*R**2) 
    

    fuselage.lengths.nose          = nose_length
    fuselage.lengths.tail          = tail_length
    fuselage.lengths.cabin         = cabin_length 
    fuselage.areas.wetted          = wetted_area
    fuselage.areas.front_projected = front_projected_area
    fuselage.areas.side_projected  = side_projected_area 
    fuselage.effective_diameter    = effective_diameter 

    return 
