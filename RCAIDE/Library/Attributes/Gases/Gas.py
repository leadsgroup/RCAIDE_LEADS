# RCAIDE/Library/Attributes/Gases/Gas.py
# 
#
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------  
#  Imports
# ----------------------------------------------------------------------------------------------------------------------  

from RCAIDE.Framework.Core import Data 

# ----------------------------------------------------------------------------------------------------------------------  
#  Gas Class
# ----------------------------------------------------------------------------------------------------------------------   
class Gas(Data):
    """
    Base class for all gas implementations in RCAIDE. Provides fundamental gas properties and 
    composition tracking.

    Attributes
    ----------
    tag : str
        Identifier for the gas type ('gas' by default)
    molecular_mass : float
        Molecular mass of the gas in kg/kmol
    gas_specific_constant : float
        Specific gas constant in m²/s²-K
    composition : Data
        Chemical composition container for tracking gas components
            - gas : float
                Default mass fraction of primary gas component (1.0 by default)

    Notes
    -----
    This class serves as the parent class for all specific gas implementations.
    The default values represent a generic gas and should be overridden by child classes
    with specific gas properties.
    
    **Definitions**
    
    'Molecular Mass'
        The mass of one mole of the gas molecules in kg/kmol
    
    'Specific Gas Constant'
        The individual gas constant for the specific gas, calculated as the universal 
        gas constant divided by the molecular mass
    
    'Mass Fraction'
        The ratio of the mass of a component to the total mass of the gas mixture
    """

    def __defaults__(self):
        """This sets the default values. 
    
        Assumptions:
            None
        
        Source:
            None
        """    
        self.tag                   ='gas'
        self.molecular_mass        = 0.0    
        self.gas_specific_constant = 0.0              
        self.composition           = Data()
        self.composition.gas       = 1.0
