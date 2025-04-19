# RCAIDE/Library/Compoments/Nacelles/Segment.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from RCAIDE.Framework.Core import Container
from RCAIDE.Library.Components import Component  

# ---------------------------------------------------------------------------------------------------------------------- 
#  Segment
# ----------------------------------------------------------------------------------------------------------------------   
class Segment(Component):
    """
    A component representing a single cross-sectional segment of a nacelle. Used to build
    up complex nacelle geometries through a series of connected sections.

    Attributes
    ----------
    tag : str
        Unique identifier for the segment, defaults to 'segment'
        
    orientation_euler_angles : list
        Rotation angles [roll, pitch, yaw], defaults to [0.0, 0.0, 0.0]
        
    percent_x_location : float
        Longitudinal position as percentage of nacelle length, defaults to 0
        
    percent_y_location : float
        Lateral position as percentage of nacelle width, defaults to 0
        
    percent_z_location : float
        Vertical position as percentage of nacelle height, defaults to 0
        
    height : float
        Vertical dimension of the segment cross-section, defaults to 0
        
    width : float
        Lateral dimension of the segment cross-section, defaults to 0
        
    curvature : float
        Shape parameter controlling cross-section corner rounding using super-ellipse 
        formulation, defaults to 2

    Notes
    -----
    Segments are used to build up the complete nacelle geometry through a series of 
    cross-sections. Each segment's position is defined as a percentage of the overall 
    nacelle dimensions, allowing for flexible scaling and positioning.
    
    **Major Assumptions**
    
    * Segments are connected in sequence to form a continuous surface
    * Cross-sections are defined by height, width, and curvature parameters
    * Super-ellipse formulation used for cross-section shape
    
    **Definitions**

    'Super-ellipse'
        Mathematical curve used to create smooth, rounded rectangular shapes with
        controllable corner curvature

    See Also
    --------
    RCAIDE.Library.Components.Nacelles.Stack_Nacelle
        Nacelle type that uses multiple segments
    """

    def __defaults__(self): 
        """
        Sets default values for the nacelle segment attributes.
        """
        self.tag                      = 'segment' 
        self.orientation_euler_angles = [0.,0.,0.]  
        self.percent_x_location       = 0  
        self.percent_y_location       = 0
        self.percent_z_location       = 0 
        self.height                   = 0 
        self.width                    = 0 
        self.curvature                = 2 # super ellipse 
         
class Segment_Container(Container):
    """
    Container class for managing nacelle segments. Provides organization and 
    access methods for segment components.

    Notes
    -----
    This container is designed to hold and manage Segment objects that define 
    the nacelle shape. It inherits from the base Container class and provides 
    specialized functionality for nacelle segments.

    See Also
    --------
    RCAIDE.Framework.Core.Container
        Base container class providing core functionality
    RCAIDE.Library.Components.Nacelles.Segment
        The segment components stored in this container
    """     

    def get_children(self):
        """
        Returns a list of allowable child component types for the segment container.

        Returns
        -------
        list
            Empty list as segments do not contain child components
        """       
        return []