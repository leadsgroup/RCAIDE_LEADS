# RCAIDE/Library/Attributes/Gases/CO2.py
# 
# Created:  Mar 2024, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------  
#  Imports
# ----------------------------------------------------------------------------------------------------------------------  
 
from .Gas import Gas  

# ----------------------------------------------------------------------------------------------------------------------  
# CO2 Class
# ----------------------------------------------------------------------------------------------------------------------   
class CO2(Gas):
    """
    A class representing carbon dioxide gas and its thermodynamic properties.

    Attributes
    ----------
    tag : str
        Identifier for the gas type ('CO2')
    molecular_mass : float
        Molecular mass of CO2 in kg/kmol
    gas_specific_constant : float
        Specific gas constant in m²/s²-K
    composition : Container
        Chemical composition of the gas
            - CO2 : float
                Mass fraction of carbon dioxide (1.0 for pure CO2)

    Notes
    -----
    This class implements basic thermodynamic properties for carbon dioxide gas.
    All properties are for pure CO2 at standard conditions.
    
    **Definitions**
    
    'Specific Gas Constant'
        The individual gas constant for CO2, equal to the universal gas constant divided 
        by the molecular mass of CO2
    
    'Molecular Mass'
        The mass of one mole of CO2 molecules
    """
    def __defaults__(self):
        """This sets the default values.
        
            Assumptions:
                None
            
            Source:
                None
        """            
        self.tag                   ='CO2'
        self.molecular_mass        = 44.01           # kg/kmol
        self.gas_specific_constant = 188.9                       # m^2/s^2-K, specific gas constant
        self.composition.CO2       = 1.0
    