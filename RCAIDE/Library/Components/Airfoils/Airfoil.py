# RCAIDE/Library/Compoments/Airfoils/Airfoil.py
# 
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports     
from RCAIDE.Library.Components   import Component 
 
# ---------------------------------------------------------------------------------------------------------------------- 
#  Airfoil
# ---------------------------------------------------------------------------------------------------------------------- 
 
class Airfoil(Component):
    """
    Base class for defining airfoil geometries and their aerodynamic characteristics

    Attributes
    ----------
    tag : str
        Identifier for the airfoil (default: 'Airfoil')
        
    coordinate_file : str, optional
        Path to file containing airfoil coordinates (default: None)
        
    geometry : array_like, optional
        Array containing x,y coordinates defining the airfoil shape (default: None)
        
    polar_files : list of str, optional
        Paths to files containing aerodynamic polar data (default: None)
        
    polars : dict, optional
        Dictionary containing aerodynamic coefficient data (default: None)
        
    prev : Airfoil, optional
        Reference to previous airfoil in a wing (default: None)
        
    next : Airfoil, optional
        Reference to next airfoil in a wing (default: None)
        
    number_of_points : int
        Number of points used to discretize the airfoil geometry (default: 201)

    Notes
    -----
    The Airfoil class serves as a base class for more specific airfoil implementations.
    It provides basic structure for storing geometry and aerodynamic data, which can be
    loaded from files or generated programmatically.
    
    **Definitions**
    
    'Polar Data'
        Aerodynamic coefficient data (typically cl, cd, cm) as a function of angle of attack

    See Also
    --------
    RCAIDE.Library.Components.Airfoils.NACA_4_Series_Airfoil
        Implementation for NACA 4-series airfoils
    """
    
    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """         
        
        self.tag                        = 'airfoil' 
        self.coordinate_file            = None     
        self.geometry                   = None
        self.polar_files                = None
        self.polars                     = None
        self.prev                       = None
        self.next                       = None
        self.number_of_points           = 201
       