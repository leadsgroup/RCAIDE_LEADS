# RCAIDE/Library/Components/Wings/Vertical_Tail.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports   
from .Wing import Wing 
from copy import deepcopy 
# ---------------------------------------------------------------------------------------------------------------------- 
#  Vertical_Tail
# ----------------------------------------------------------------------------------------------------------------------  
class Vertical_Tail(Wing):
    """
    A class representing a vertical stabilizer surface for aircraft directional control 
    and stability.

    Attributes
    ----------
    tag : str
        Unique identifier for the vertical tail, defaults to 'vertical_tail'
        
    vertical : bool
        Flag indicating vertical orientation, defaults to True
        
    symmetric : bool
        Flag indicating if tail is symmetric about x-z plane, defaults to False

    Notes
    -----
    The vertical tail provides directional stability and serves as a mounting surface 
    for the rudder. It inherits all geometric and aerodynamic functionality from the 
    Wing class and adds specific attributes for vertical tail operation.

    See Also
    --------
    RCAIDE.Library.Components.Wings.Wing
        Base wing class providing core functionality
    RCAIDE.Library.Components.Wings.Control_Surfaces.Rudder
        Control surface typically mounted on vertical tail
    RCAIDE.Library.Components.Wings.Vertical_Tail_All_Moving
        All-moving vertical tail variant
    """ 

    def __defaults__(self):
        """
        Sets default values for the vertical tail attributes.
        """
        self.tag                = 'vertical_tail'
        self.vertical           = True
        self.xz_plane_symmetric = False
        
    def make_x_z_reflection(self):
        """
        Creates a reflected copy of the vertical tail over the x-z plane.

        Returns
        -------
        Component
            Reflected vertical tail with appropriate sign conventions

        Notes
        -----
        * Used when vertical tail's xz_plane_symmetric attribute is True
        * Reflects dihedral angles and origin location
        * Control surface deflections are reflected according to sign_duplicate
        * Should be called after setting control surface deflections

        WARNING: this uses deepcopy to achieve its purpose. If this copies too many unwanted 
        attributes, it is recommended that the user should write their own code, taking 
        after the form of this function.
        
        It is also recommended that the user call this function after they set control surface
        or all moving surface deflections. This way the deflection is also properly reflected 
        to the other side
        """    
        wing = deepcopy(self)
        wing.dihedral     *= -1
        wing.origin[0][1] *= -1
        
        for segment in wing.segments:
            segment.dihedral_outboard *= -1
            
        for cs in wing.control_surfaces:
            cs.deflection *= -1*cs.sign_duplicate
                
        return wing