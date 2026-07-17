# RCAIDE/Library/Components/Wings/Horizontal_Tail.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from .Wing import Wing  

# ---------------------------------------------------------------------------------------------------------------------- 
#  Horizontal_Tail
# ----------------------------------------------------------------------------------------------------------------------    
class Horizontal_Tail(Wing):
    """
    A class representing a horizontal stabilizer surface for aircraft pitch control 
    and longitudinal stability.

    Attributes
    ----------
    tag : str
        Unique identifier for the horizontal tail, defaults to 'horizontal_tail'

    Notes
    -----
    The horizontal tail provides pitch stability and control. It inherits all geometric 
    and aerodynamic functionality from the Wing class and adds specific attributes for 
    horizontal tail operation. 

    See Also
    --------
    RCAIDE.Library.Components.Wings.Wing
        Base wing class providing core functionality
    RCAIDE.Library.Components.Wings.Stabilator
        All-moving horizontal tail variant
    RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator
        Control surface typically mounted on horizontal tail
    """ 

    def __defaults__(self):
        """
        Sets default values for the horizontal tail attributes.
        """
        self.tag = 'horizontal_tail'
    