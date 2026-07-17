# RCAIDE/Components/Wings/Control_Surfaces/Slat.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from .Control_Surface import Control_Surface 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Slat
# ----------------------------------------------------------------------------------------------------------------------
class Slat(Control_Surface):
    """
    A class representing a slat control surface for high-lift generation during takeoff 
    and landing.

    Attributes
    ----------
    tag : str
        Unique identifier for the slat, defaults to 'slat'
        
    hinge_fraction : float
        Location of the hinge line as fraction of chord, defaults to 1.0
        (leading edge device)
        
    sign_duplicate : float
        Sign convention for duplicate slat deflection, defaults to 1.0
        (synchronized deflection for lift generation)

    Notes
    -----
    The slat is a leading edge high-lift device used to increase maximum lift coefficient 
    and stall angle at low speeds. It inherits basic control surface functionality from 
    the Control_Surface class and adds specific attributes for slat operation.
    
    **Definitions**

    'Hinge Fraction'
        The chordwise location of the slat hinge line, measured from the leading 
        edge as a fraction of local chord. Set to 1.0 for leading edge device
        
    'Sign Duplicate'
        Determines whether paired slats deflect in the same or opposite directions. 
        1.0 indicates synchronized deflection for lift generation

    See Also
    --------
    RCAIDE.Library.Components.Wings.Control_Surfaces.Control_Surface
        Base class providing common control surface functionality
    RCAIDE.Library.Components.Wings.Control_Surfaces.Flap
        Trailing edge high-lift device
    """ 

    def __defaults__(self):
        """
        Sets default values for the slat attributes.
        
        Notes
        -----
        See Control_Surface.__defaults__ for additional inherited attributes.
        """
        self.tag            = 'slat'
        self.hinge_fraction = 1.0
        self.sign_duplicate = 1.0
        
        pass 