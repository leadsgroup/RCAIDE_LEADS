# RCAIDE/Library/Compoments/Wings/Stabilator.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from .Horizontal_Tail    import Horizontal_Tail
from .All_Moving_Surface import All_Moving_Surface
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_wing_moment_of_inertia import  compute_wing_moment_of_inertia

# ---------------------------------------------------------------------------------------------------------------------- 
#  Stabilator
# ----------------------------------------------------------------------------------------------------------------------     
class Stabilator(Horizontal_Tail, All_Moving_Surface):
    """
    A class representing an all-moving horizontal tail (stabilator) that provides 
    pitch control and longitudinal stability.

    Attributes
    ----------
    tag : str
        Unique identifier for the stabilator, defaults to 'stabilator'
        
    sign_duplicate : float
        Sign convention for duplicate stabilator deflection, defaults to 1.0
        (synchronized deflection for pitch control)

    Notes
    -----
    The stabilator combines the functions of a horizontal stabilizer and elevator 
    by pivoting as a complete surface. It inherits from both Horizontal_Tail and 
    All_Moving_Surface to combine their functionality.

    See Also
    --------
    RCAIDE.Library.Components.Wings.Horizontal_Tail
        Base class providing horizontal tail functionality
    RCAIDE.Library.Components.Wings.All_Moving_Surface
        Base class providing pivoting surface functionality
    RCAIDE.Library.Components.Wings.Control_Surfaces.Elevator
        Alternative pitch control using conventional elevator
    """ 

    def __defaults__(self):
        """
        Sets default values for the stabilator attributes.
        
        Notes
        -----
        See All_Moving_Surface.__defaults__ and Wing.__defaults__ for 
        additional inherited attributes.
        """
        self.tag            = 'stabilator'
        self.sign_duplicate = 1.0

    def moment_of_inertia(wing, center_of_gravity):
        """
        Computes the moment of inertia tensor for the stabilator.

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