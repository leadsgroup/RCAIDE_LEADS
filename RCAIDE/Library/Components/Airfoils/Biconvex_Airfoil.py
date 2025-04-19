# RCAIDE/Library/Compoments/Airfoils/Biconvex_Airfoil.py
# 
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports     
from .Airfoil import Airfoil
 
# ---------------------------------------------------------------------------------------------------------------------- 
#  Airfoil
# ---------------------------------------------------------------------------------------------------------------------- 
 
class Biconvex_Airfoil(Airfoil):
    """
    Class for generating Bi-convex airfoil geometries.

    Attributes
    ----------
    tag : str
        Identifier for the airfoil (default: 'biconvex') 
    
    See Also
    --------
    RCAIDE.Library.Components.Airfoils.Airfoil
        Base airfoil class
    """
    
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """        
        self.tag                   = 'biconvex' 
        return 
