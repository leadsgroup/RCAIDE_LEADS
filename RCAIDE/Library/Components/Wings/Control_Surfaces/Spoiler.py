# RCAIDE/Compoments/Wings/Control_Surfaces/Spoiler.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from .Control_Surface import Control_Surface 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Spoiler
# ----------------------------------------------------------------------------------------------------------------------
class Spoiler(Control_Surface):
    """
    A class representing a spoiler control surface for drag generation/lift reduction during descent
    and landing.

    Attributes
    ----------
    tag : str
        Unique identifier for the spoiler, defaults to 'spoiler' 

    Notes
    -----
    The spoiler is an upper surface device used as an airbrake to decrease the
    lift coefficient and increase drag. It inherits basic control surface functionality from 
    the Control_Surface class and adds specific attributes for spoiler operation. It is not modeled
    in the aerodynamics module, rather correlations from a NASA TR are used to predict aerodynamic
    coefficient modifications

    See Also
    --------
    RCAIDE.Library.Components.Wings.Control_Surfaces.Control_Surface
        Base class providing common control surface functionality
    RCAIDE.Library.Components.Wings.Control_Surfaces.Flap
        Trailing edge high-lift device
    """ 

    def __defaults__(self):
        """
        Sets default values for the spoiler attributes.
        
        Notes
        -----
        See Control_Surface.__defaults__ for additional inherited attributes.
        """
        self.tag            = 'spoiler' 
        
        pass
      


    