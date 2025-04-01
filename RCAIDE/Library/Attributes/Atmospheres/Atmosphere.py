# RCAIDE/Library/Attributes/Atmospheres/Atmosphere.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
 
from RCAIDE.Framework.Core import Data

# ---------------------------------------------------------------------------------------------------------------------- 
#  Industrial_Costs Class
# ----------------------------------------------------------------------------------------------------------------------   
class Atmosphere(Data):
    """
    Base class for atmospheric models providing a framework for defining atmospheric properties.

    Attributes
    ----------
    tag : str
        Identifier for the atmospheric model type
    composition : Data
        Container for atmospheric composition information
            - gas : float
                Mass fraction of gas in the atmosphere (default: 1.0)

    Notes
    -----
    This class serves as a template for specific atmospheric implementations (e.g., Earth's atmosphere)
    and provides basic structure for atmospheric composition data.

    **Definitions**

    'Constant-property atmosphere'
        An atmospheric model where properties remain constant and do not vary with altitude or other parameters

    See Also
    --------
    RCAIDE.Library.Attributes.Atmospheres.Earth : Earth-specific atmospheric models
    """

    def __defaults__(self):
        """This sets the default values. 
    
        Assumptions:
            None
        
        Source:
            None 
        """          
        self.tag = 'Constant-property atmosphere'
        self.composition           = Data()
        self.composition.gas       = 1.0
