# RCAIDE/Library/Components/Powertrain/Energy/Sources/Fuel_Tanks/Segment.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from RCAIDE.Framework.Core import Data, Container
from RCAIDE.Library.Components import Component  

# ---------------------------------------------------------------------------------------------------------------------- 
#  Segment
# ----------------------------------------------------------------------------------------------------------------------   
class Segment(Component):
    """
    A component representing a generic super-ellipse cross-sectional segment of a fuel tank. Segments are used 
    to define the shape and dimensions of the fuel tank through a series of connected cross-sections.

    Attributes
    ----------
    tag : str
        Identifier for the segment, defaults to 'segment' 
        
    percent_x_location : float
        Longitudinal position as percentage of fuel tank length, defaults to 0
        
    percent_y_location : float
        Lateral position as percentage of fuel tank width, defaults to 0
        
    percent_z_location : float
        Vertical position as percentage of fuel tank height, defaults to 0
        
    height : float
        Vertical dimension of the segment cross-section, defaults to 0
        
    width : float
        Lateral dimension of the segment cross-section, defaults to 0
        
    curvature : float
        Shape parameter controlling cross-section corner rounding, defaults to 2

    Notes
    -----
    Segments are used to build up the complete fuel tank geometry through a series of 
    cross-sections. Each segment's position is defined as a percentage of the overall 
    fuel tank dimensions, allowing for flexible scaling and positioning.

    **Major Assumptions**
    
    * Segments are connected in sequence to form a continuous surface
    * Cross-sections are symmetric about the vertical plane
    * Transitions between segments are smooth
    
    **Definitions**

    'Cross-section'
        The 2D shape formed by intersecting the fuel tank with a plane perpendicular 
        to its longitudinal axis
        
    'Curvature'
        Parameter controlling the smoothness of transition between vertical and 
        horizontal surfaces at the corners of the cross-section

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Source.Fuel_Tanks.Fuel_Tank
        Parent container for fuel tank segments
    """

    def __defaults__(self): 
        """
        Sets default values for the fuel tank segment attributes.
        """
        self.tag                     = 'segment' 
        self.percent_x_location      = 0  
        self.percent_y_location      = 0
        self.percent_z_location      = 0 
        self.height                  = 0 
        self.width                   = 0 
        self.curvature               = 2
        self.radius                  = 0

        self.vsp_data                = Data()
        self.vsp_data.xsec_id        = ''       
        self.vsp_data.shape          = ''        
         
class Segment_Container(Container):
    """
    Container class for managing fuel tank segments. Provides organization and 
    access methods for segment components.

    Notes
    -----
    This container is designed to hold and manage Segment objects that define 
    the fuel tank shape. It inherits from the base Container class and provides 
    specialized functionality for fuel tank segments.

    See Also
    --------
    RCAIDE.Framework.Core.Container
        Base container class providing core functionality
    RCAIDE.Library.Components.Powertrain.Source.Fuel_Tanks.Fuel_Tank.Segments.Segment
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