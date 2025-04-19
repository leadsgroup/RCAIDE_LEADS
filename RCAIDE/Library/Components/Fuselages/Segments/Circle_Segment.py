# RCAIDE/Library/Compoments/Fuselage/Circle_Segment.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports    
from .Segment import  Segment

# ---------------------------------------------------------------------------------------------------------------------- 
#  Segment
# ----------------------------------------------------------------------------------------------------------------------   
class Circle_Segment(Segment):
    """
    A component representing a circular cross-sectional segment of a fuselage. Segments are used 
    to define the shape and dimensions of the fuselage through a series of connected cross-sections.

    Attributes
    ----------
    tag : str
        Identifier for the segment, defaults to 'segment'
        
    prev : Component
        Link to the previous segment in the fuselage chain, defaults to None
        
    next : Component
        Link to the next segment in the fuselage chain, defaults to None
        
    percent_x_location : float
        Longitudinal position as percentage of fuselage length, defaults to 0
        
    percent_y_location : float
        Lateral position as percentage of fuselage width, defaults to 0
        
    percent_z_location : float
        Vertical position as percentage of fuselage height, defaults to 0
        
    height : float
        Vertical dimension of the segment cross-section, defaults to 0
        
    width : float
        Lateral dimension of the segment cross-section, defaults to 0
        
    radius : float
        Shape parameter controlling cross-section corner rounding, defaults to 2

    Notes
    -----
    Segments are used to build up the complete fuselage geometry through a series of 
    cross-sections. Each segment's position is defined as a percentage of the overall 
    fuselage dimensions, allowing for flexible scaling and positioning.

    **Major Assumptions**
    
    * Segments are connected in sequence to form a continuous surface
    * Cross-sections are symmetric about the vertical plane
    * Transitions between segments are smooth
    
    **Definitions**

    'Cross-section'
        The 2D shape formed by intersecting the fuselage with a plane perpendicular 
        to its longitudinal axis
        
    'Curvature'
        Parameter controlling the smoothness of transition between vertical and 
        horizontal surfaces at the corners of the cross-section

    See Also
    --------
    RCAIDE.Library.Components.Fuselages.Fuselage
        Parent container for fuselage segments
    """

    def __defaults__(self): 
        """
        Sets default values for the fuselage segment attributes.
        """
        self.tag                     = 'super_ellipse_segment' 
          