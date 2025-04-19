# RCAIDE/Compoments/Wings/Control_Surfaces/Rudder.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from .Control_Surface import Control_Surface 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Rudder
# ----------------------------------------------------------------------------------------------------------------------
class Rudder(Control_Surface):
    """
    A class representing a rudder control surface for aircraft directional control.

    Attributes
    ----------
    tag : str
        Unique identifier for the rudder, defaults to 'rudder'
        
    hinge_fraction : float
        Location of the hinge line as fraction of chord, defaults to 0.0
        
    sign_duplicate : float
        Sign convention for duplicate rudder deflection, defaults to 1.0
        (typically only one rudder per aircraft)

    Notes
    -----
    The rudder is a trailing edge control surface mounted on the vertical tail, used 
    primarily for yaw control. It inherits basic control surface functionality from 
    the Control_Surface class and adds specific attributes for rudder operation.
    
    **Definitions**

    'Hinge Fraction'
        The chordwise location of the rudder hinge line, measured from the leading 
        edge as a fraction of local chord
        
    'Sign Duplicate'
        Determines whether paired rudders (if present) deflect in the same or 
        opposite directions. Typically 1.0 as most aircraft have a single rudder

    See Also
    --------
    RCAIDE.Library.Components.Wings.Control_Surfaces.Control_Surface
        Base class providing common control surface functionality
    RCAIDE.Library.Components.Wings.Vertical_Tail_All_Moving
        Alternative yaw control using all-moving vertical tail
    """ 

    def __defaults__(self):
        """
        Sets default values for the rudder attributes.
        
        Notes
        -----
        See Control_Surface.__defaults__ for additional inherited attributes.
        """
        self.tag            = 'rudder'
        self.hinge_fraction = 0.0
        self.sign_duplicate = 1.0
        
        pass 