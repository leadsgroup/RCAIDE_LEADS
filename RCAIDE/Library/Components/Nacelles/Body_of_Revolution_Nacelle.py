# RCAIDE/Compoments/Nacelles/Body_of_Revolution_Nacelle.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core     import Data  
from .Nacelle                  import Nacelle 
 
# ---------------------------------------------------------------------------------------------------------------------- 
#  Body_of_Revolution_Nacelle
# ----------------------------------------------------------------------------------------------------------------------  
class Body_of_Revolution_Nacelle(Nacelle):
    """
    A nacelle design generated by rotating an airfoil profile around a central axis, 
    commonly used for podded engine installations.

    Attributes
    ----------
    tag : str
        Unique identifier for the nacelle component, defaults to 'body_of_revolution_nacelle'
        
    Airfoil : Data
        Container for airfoil profile data used to generate the nacelle shape, 
        defaults to empty Data()

    Notes
    -----
    A body of revolution nacelle is created by rotating an airfoil section about a 
    central axis, creating an axisymmetric shape. This approach is commonly used for:
    
    * Podded engine installations
    * Streamlined auxiliary power unit enclosures
    * External fuel tank designs
    
    **Major Assumptions**
    
    * Axisymmetric geometry
    * Smooth surface transitions
    * No significant flow separation
    
    **Definitions**

    'Body of Revolution'
        A three-dimensional shape created by rotating a two-dimensional profile 
        around a central axis
        
    'Highlight Radius'
        The radius at the forward-most point of the nacelle inlet

    See Also
    --------
    RCAIDE.Library.Components.Nacelles.Nacelle
        Base nacelle class
    RCAIDE.Library.Components.Nacelles.Stack_Nacelle
        Alternative nacelle design approach
    """
    
    def __defaults__(self):
        """
        Sets default values for the body of revolution nacelle attributes.
        """      
        self.tag     = 'body_of_revolution_nacelle' 
        self.Airfoil = None
    
    def append_airfoil(self, airfoil):
        """
        Adds an airfoil profile to the nacelle definition.

        Parameters
        ----------
        airfoil : Data
            Airfoil profile data to be used in generating the nacelle shape
        """
        # Assert database type
        if not isinstance(airfoil,RCAIDE.Library.Components.Airfoils.Airfoil):
            raise Exception('input component must be of type Airfoil')

        # Store data
        self.airfoil = airfoil

        return            