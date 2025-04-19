# RCAIDE/Compoments/Wings/Control_Surfaces/Elevator.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from .Control_Surface import Control_Surface 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Elevator
# ----------------------------------------------------------------------------------------------------------------------
class Elevator(Control_Surface):
    """
    A class representing an elevator control surface for aircraft pitch control.

    Attributes
    ----------
    tag : str
        Unique identifier for the elevator, defaults to 'elevator'
        
    hinge_fraction : float
        Location of the hinge line as fraction of chord, defaults to 0.0
        
    sign_duplicate : float
        Sign convention for duplicate elevator deflection, defaults to 1.0
        (synchronized deflection for pitch control)

    Notes
    -----
    The elevator is a trailing edge control surface used primarily for pitch control. 
    It inherits basic control surface functionality from the Control_Surface class 
    and adds specific attributes for elevator operation.
    
    **Definitions**

    'Hinge Fraction'
        The chordwise location of the elevator hinge line, measured from the leading 
        edge as a fraction of local chord
        
    'Sign Duplicate'
        Determines whether paired elevators deflect in the same or opposite directions. 
        1.0 indicates synchronized deflection for pitch control

    See Also
    --------
    RCAIDE.Library.Components.Wings.Control_Surfaces.Control_Surface
        Base class providing common control surface functionality
    RCAIDE.Library.Components.Wings.Control_Surfaces.Stabilator
        Alternative pitch control using all-moving horizontal tail
    """ 

    def __defaults__(self):
        """
        Sets default values for the elevator attributes.
        
        Notes
        -----
        See Control_Surface.__defaults__ for additional inherited attributes.
        """
        self.tag            = 'elevator'
        self.hinge_fraction = 0.0
        self.sign_duplicate = 1.0
        
        pass 