# RCAIDE/Compoments/Wings/Control_Surfaces/Aileron.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from .Control_Surface import Control_Surface 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Aileron
# ----------------------------------------------------------------------------------------------------------------------  
class Aileron(Control_Surface):
    """
    A class representing an aileron control surface for aircraft roll control.

    Attributes
    ----------
    tag : str
        Unique identifier for the aileron, defaults to 'aileron'
        
    hinge_fraction : float
        Location of the hinge line as fraction of chord, defaults to 0.0
        
    sign_duplicate : float
        Sign convention for duplicate aileron deflection, defaults to -1.0
        (opposite deflection for roll control)

    Notes
    -----
    The aileron is a trailing edge control surface used primarily for roll control. 
    It inherits basic control surface functionality from the Control_Surface class 
    and adds specific attributes for aileron operation.
    
    **Definitions**

    'Hinge Fraction'
        The chordwise location of the aileron hinge line, measured from the leading 
        edge as a fraction of local chord
        
    'Sign Duplicate'
        Determines whether paired ailerons deflect in the same or opposite directions. 
        -1.0 indicates opposite deflection for roll control

    See Also
    --------
    RCAIDE.Library.Components.Wings.Control_Surfaces.Control_Surface
        Base class providing common control surface functionality
    RCAIDE.Library.Components.Wings.Control_Surfaces.Flap
        Similar trailing edge control surface used for high lift
    """ 

    def __defaults__(self):
        """
        Sets default values for the aileron attributes.
        
        Notes
        -----
        See Control_Surface.__defaults__ for additional inherited attributes.
        """
        self.tag            = 'aileron'  
        self.hinge_fraction = 0.0
        self.sign_duplicate = -1.0
        
        pass 