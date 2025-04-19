# RCAIDE/Library/Compoments/Wings/Main_Wing.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from .Wing import Wing
from RCAIDE.Framework.Core import Container 
from RCAIDE.Library.Components.Wings.Segments.Segment import Segment
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_wing_moment_of_inertia import  compute_wing_moment_of_inertia

# ---------------------------------------------------------------------------------------------------------------------- 
#  Main_Wing
# ----------------------------------------------------------------------------------------------------------------------  
class Main_Wing(Wing):
    """
    A class representing the primary lifting surface of an aircraft.

    Attributes
    ----------
    tag : str
        Unique identifier for the main wing, defaults to 'main_wing'
        
    segments : Segment_Container
        Collection of wing segments defining the wing geometry, initialized empty

    Notes
    -----
    The main wing provides primary lift generation for the aircraft. It inherits all 
    geometric and aerodynamic functionality from the Wing class and adds specific 
    attributes for main wing operation.

    See Also
    --------
    RCAIDE.Library.Components.Wings.Wing
        Base wing class providing core functionality
    RCAIDE.Library.Components.Wings.Segments.Segment
        Wing segment components used to build the wing
    RCAIDE.Library.Components.Wings.Control_Surfaces
        Control surfaces that can be mounted on the wing
    """ 

    def __defaults__(self):
        """
        Sets default values for the main wing attributes.
        """
        self.tag      = 'main_wing'
        self.segments = Segment_Container()
         
    def moment_of_inertia(wing, center_of_gravity):
        """
        Computes the moment of inertia tensor for the main wing.

        Parameters
        ----------
        wing : Component
            Wing component data
        center_of_gravity : list
            Reference point coordinates for moment calculation

        Returns
        -------
        ndarray
            3x3 moment of inertia tensor
        """
        I = compute_wing_moment_of_inertia(wing, center_of_gravity) 
        return I 

class Segment_Container(Container):
    """
    Container class for managing wing segments. Provides organization and 
    access methods for segment components.

    Notes
    -----
    This container is designed to hold and manage Segment objects that define 
    the wing geometry. It inherits from the base Container class and provides 
    specialized functionality for wing segments.

    See Also
    --------
    RCAIDE.Library.Components.Wings.Segments.Segment
        The segment components stored in this container
    """     

    def get_children(self):
        """
        Returns a list of allowable child component types for the segment container.

        Returns
        -------
        list
            List containing the Segment class as the only allowable child type
        """       
        return [Segment] 
    
    def append(self, val):
        """
        Appends a segment to the container with automatic name handling.

        Parameters
        ----------
        val : Segment
            Wing segment to be added

        Notes
        -----
        Automatically modifies segment tags to avoid duplicates by appending 
        numbers to repeated tags
        """          
        # See if the component exists, if it does modify the name
        keys = self.keys()
        if val.tag in keys:
            string_of_keys = "".join(self.keys())
            n_comps = string_of_keys.count(val.tag)
            val.tag = val.tag + str(n_comps+1)    
            
        Container.append(self, val) 