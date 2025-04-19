# RCAIDE/Compoments/Nacelles/Stack_Nacelle.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core              import Data, Container  
from .Nacelle import Nacelle
  
# ---------------------------------------------------------------------------------------------------------------------- 
#  Stack_Nacelle
# ----------------------------------------------------------------------------------------------------------------------  
class Stack_Nacelle(Nacelle):
    """
    A nacelle design built up from a series of stacked cross-sectional segments, 
    allowing for complex, non-axisymmetric shapes.

    Attributes
    ----------
    tag : str
        Unique identifier for the nacelle component, defaults to 'stack_nacelle'
        
    segments : Container
        Collection of cross-sectional segments defining the nacelle shape, 
        initialized empty

    Notes
    -----
    The stack nacelle approach provides flexibility in designing complex nacelle 
    shapes through the use of multiple cross-sections. This is particularly useful for:
    
    * Non-axisymmetric designs
    * Complex inlet and nozzle geometries
    * Integration with adjacent structures
    
    **Major Assumptions**
    
    * Segments are ordered from front to back
    * Adjacent segments form smooth transitions
    * Cross-sections are defined using super-ellipse parameters
    
    **Definitions**

    'Stack'
        Series of cross-sectional segments arranged longitudinally to define 
        the nacelle shape
        
    'Cross-section'
        Individual segment defining the nacelle shape at a specific longitudinal station

    See Also
    --------
    RCAIDE.Library.Components.Nacelles.Nacelle
        Base nacelle class
    RCAIDE.Library.Components.Nacelles.Segment
        Individual segments used to build the stack
    RCAIDE.Library.Components.Nacelles.Body_of_Revolution_Nacelle
        Alternative nacelle design approach
    """
    
    def __defaults__(self):
        """
        Sets default values for the stack nacelle attributes.
        """      
        self.tag      = 'stack_nacelle'  
        self.segments = Container() 
        
    def append_segment(self, segment):
        """
        Adds a new segment to the nacelle's segment container.

        Parameters
        ----------
        segment : Data
            Cross-sectional segment to be added to the nacelle definition
        """
        # Assert database type
        if not isinstance(segment,RCAIDE.Library.Components.Nacelles.Segments.Segment):
            raise Exception('input component must be of type Segment')

        # Store data
        self.segments.append(segment)

        return  
