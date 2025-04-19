# RCAIDE/Library/Compoments/Airfoils/NACA_4_Series_Airfoil.py
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
 
class NACA_4_Series_Airfoil(Airfoil):
    """
    Class for generating NACA 4-series airfoil geometries.

    Attributes
    ----------
    tag : str
        Identifier for the airfoil (default: 'NACA_4_Series')
        
    NACA_4_Series_code : str
        Four-digit NACA designation (default: '0012')
        
    Notes
    -----
    Inherits all attributes from the base Airfoil class. The geometry is generated
    based on the NACA 4-series equations using the specified four-digit code.
    
    The NACA 4-series airfoils are defined by four digits that specify the camber,
    position of maximum camber, and thickness. For example, in a NACA 2412:
    - First digit (2) specifies maximum camber in percentage of chord
    - Second digit (4) specifies position of maximum camber in tenths of chord
    - Last two digits (12) specify maximum thickness in percentage of chord
    
    'NACA Code Format'
        MPXX where:
        - M is the maximum camber in percentage of chord
        - P is the position of maximum camber in tenths of chord
        - XX is the maximum thickness in percentage of chord
    
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
        self.tag                   = 'NACA_4_Series'
        self.NACA_4_Series_code    = '0012'
        return 
