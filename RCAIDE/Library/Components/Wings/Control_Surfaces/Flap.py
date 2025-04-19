# RCAIDE/Compoments/Wings/Control_Surfaces/Flap.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from .Control_Surface import Control_Surface 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Flap
# ----------------------------------------------------------------------------------------------------------------------
class Flap(Control_Surface):
    """
    A class representing a flap control surface for high-lift generation during takeoff 
    and landing.

    Attributes
    ----------
    tag : str
        Unique identifier for the flap, defaults to 'flap'
        
    hinge_fraction : float
        Location of the hinge line as fraction of chord, defaults to 0.0
        
    sign_duplicate : float
        Sign convention for duplicate flap deflection, defaults to 1.0
        (synchronized deflection for lift generation)

    Notes
    -----
    The flap is a trailing edge high-lift device used to increase lift coefficient 
    at low speeds. It inherits basic control surface functionality from the 
    Control_Surface class and adds specific attributes for flap operation.
    
    **Definitions**

    'Hinge Fraction'
        The chordwise location of the flap hinge line, measured from the leading 
        edge as a fraction of local chord
        
    'Sign Duplicate'
        Determines whether paired flaps deflect in the same or opposite directions. 
        1.0 indicates synchronized deflection for lift generation

    See Also
    --------
    RCAIDE.Library.Components.Wings.Control_Surfaces.Control_Surface
        Base class providing common control surface functionality
    RCAIDE.Library.Components.Wings.Control_Surfaces.Slat
        Leading edge high-lift device
    """ 

    def __defaults__(self):
        """
        Sets default values for the flap attributes.
        
        Notes
        -----
        See Control_Surface.__defaults__ for additional inherited attributes.
        """
        self.tag            = 'flap'
        self.hinge_fraction = 0.0
        self.sign_duplicate = 1.0
 