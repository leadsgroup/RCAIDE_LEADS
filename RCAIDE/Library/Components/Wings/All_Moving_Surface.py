# RCAIDE/Library/Compoments/Wings/All_Moving_Surface.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports     
from RCAIDE.Library.Components     import Component 
from RCAIDE.Framework.Core         import DataOrdered 
import numpy as np
from RCAIDE.Library.Methods.Mass_Properties.Moment_of_Inertia.compute_wing_moment_of_inertia import  compute_wing_moment_of_inertia

# ---------------------------------------------------------------------------------------------------------------------- 
#  All_Moving_Surface
# ----------------------------------------------------------------------------------------------------------------------  
class All_Moving_Surface(Component):
    """
    A base class for control surfaces that pivot as complete aerodynamic surfaces. This class 
    provides common functionality for surfaces like stabilators and all-moving vertical tails.

    Attributes
    ----------
    tag : str
        Unique identifier for the surface, defaults to 'All_Moving_Surface_Data_Object'
        
    sign_duplicate : float
        Sign convention for duplicate surface deflection, defaults to 1.0
        
    hinge_fraction : float
        Location of the hinge line as fraction of chord, defaults to 0.25
        
    deflection : float
        Surface deflection angle, defaults to 0.0
        
    segments : Container
        Collection of surface segments, initialized empty
        
    use_constant_hinge_fraction : bool
        Flag to use constant chord fraction for hinge line, defaults to False
        
    hinge_vector : ndarray
        Vector defining hinge line orientation in body frame, defaults to [0.0, 0.0, 0.0]

    Notes
    -----
    This class is designed to be inherited alongside a Wing-derived class to create 
    specific all-moving control surfaces. It provides:
    
    * Hinge line definition options
    * Deflection handling
    * Moment of inertia calculations
    
    **Definitions**

    'Hinge Vector'
        Three-dimensional vector defining the orientation of the surface's hinge line 
        in the body frame. Default [0,0,0] indicates hinge normal to root chord
        
    'Hinge Fraction'
        Location of hinge line as fraction of chord when using constant fraction mode

    See Also
    --------
    RCAIDE.Library.Components.Wings.Stabilator
        All-moving horizontal stabilizer implementation
    RCAIDE.Library.Components.Wings.Vertical_Tail_All_Moving
        All-moving vertical stabilizer implementation
    """ 

    def __defaults__(self):
        """
        Sets default values for the all-moving surface attributes.
        """
        self.tag                         = 'All_Moving_Surface_Data_Object' 
        self.sign_duplicate              = 1.0
        self.hinge_fraction              = 0.25
        self.deflection                  = 0.0   
        self.segments                    = Container()   
        
        self.use_constant_hinge_fraction = False
        self.hinge_vector                = np.array([0.,0.,0.])

    def moment_of_inertia(wing, center_of_gravity):
        """
        Computes the moment of inertia tensor for the all-moving surface.

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
