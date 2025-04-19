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
    """Default class for all gases.
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
