# RCAIDE/Library/Compoments/Wings/Vertical_Tail_All_Moving.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from .Vertical_Tail      import Vertical_Tail
from .All_Moving_Surface import All_Moving_Surface
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_wing_moment_of_inertia import  compute_wing_moment_of_inertia

# ---------------------------------------------------------------------------------------------------------------------- 
#  Vertical_Tail_All_Moving
# ----------------------------------------------------------------------------------------------------------------------     
class Vertical_Tail_All_Moving(Vertical_Tail, All_Moving_Surface):
    """
    A class representing an all-moving vertical tail that provides directional control 
    and stability.

    Attributes
    ----------
    tag : str
        Unique identifier for the surface, defaults to 'vertical_tail_all_moving'
        
    sign_duplicate : float
        Sign convention for duplicate surface deflection, defaults to -1.0
        (opposite deflection for yaw control)

    Notes
    -----
    The all-moving vertical tail combines the functions of a vertical stabilizer and 
    rudder by pivoting as a complete surface. It inherits from both Vertical_Tail and 
    All_Moving_Surface to combine their functionality.

    See Also
    --------
    RCAIDE.Library.Components.Wings.Vertical_Tail
        Base class providing vertical tail functionality
    RCAIDE.Library.Components.Wings.All_Moving_Surface
        Base class providing pivoting surface functionality
    RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder
        Alternative yaw control using conventional rudder
    """ 

    def __defaults__(self):
        """
        Sets default values for the all-moving vertical tail attributes.
        
        Notes
        -----
        See All_Moving_Surface.__defaults__ and Wing.__defaults__ for 
        additional inherited attributes.
        """
        self.tag            = 'vertical_tail_all_moving'
        self.sign_duplicate = -1.0   
    
    def make_x_z_reflection(self):
        """
        Creates a reflected copy of the vertical tail over the x-z plane.

        Returns
        -------
        Component
            Reflected vertical tail with appropriate deflection sign

        Notes
        -----
        * Used when vertical tail's symmetric attribute is True
        * Deflection is reflected according to sign_duplicate convention
        * Should be called after setting control surface deflections
        """       
        wing                  = super().make_x_z_reflection()
        wing.deflection      *= -1*self.sign_duplicate
        wing.hinge_vector[1] *= -1
        return wing
    
    def moment_of_inertia(wing, center_of_gravity):
        """
        Computes the moment of inertia tensor for the vertical tail.

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
        
