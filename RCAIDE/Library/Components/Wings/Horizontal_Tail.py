# RCAIDE/Library/Compoments/Wings/Horizontal_Tail.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from .Wing import Wing 
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_wing_moment_of_inertia import  compute_wing_moment_of_inertia

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
    
    def moment_of_inertia(wing, center_of_gravity):
        """
        Computes the moment of inertia tensor for the horizontal tail.

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
    