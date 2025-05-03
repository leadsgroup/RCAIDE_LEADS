

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np
from RCAIDE.Library.Methods.Geometry.LOPA.compute_layout_of_passenger_accommodations import  compute_layout_of_passenger_accommodations

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

    if circular_cross_section:
        fuselage_height = fuselage_width
    else: 
        fuselage_height = fuselage.heights.maximum
            
    wetted_area = 0.0 
    # model constant fuselage cross section as an ellipse
    # approximate circumference http://en.wikipedia.org/wiki/Ellipse#Circumference
    a = fuselage_width/2.
    b = fuselage_height/2.
    A = np.pi * a * b  # area
    R = (a-b)/(a+b) # effective radius
    C = np.pi*(a+b)*(1.+ ( 3*R**2 )/( 10+np.sqrt(4.-3.*R**2) )) # circumference
    
    wetted_area += C * cabin_length
    cross_section_area = A
    
    # approximate nose and tail wetted area
    # http://adg.stanford.edu/aa241/drag/wettedarea.html
    Deff = (a+b)*(64.-3.*R**4)/(64.-16.*R**2)
    wetted_area += 0.75*np.pi*Deff * (nose_length + tail_length)
    

    fuselage.lengths.nose          = nose_length
    fuselage.lengths.tail          = tail_length
    fuselage.lengths.cabin         = cabin_length 
    fuselage.areas.wetted          = wetted_area
    fuselage.areas.front_projected = cross_section_area
    fuselage.effective_diameter    = Deff 

    return 
