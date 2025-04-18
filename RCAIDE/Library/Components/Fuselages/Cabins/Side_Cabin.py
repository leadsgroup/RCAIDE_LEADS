# RCAIDE/Compoments/Fuselages/Cabins/Side_Cabin.py
# 
# Created:  Mar 2024, M. Clarke 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE 
from .Cabin        import Cabin 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Side_Cabin
# ---------------------------------------------------------------------------------------------------------------------- 
class Side_Cabin(Cabin): 
    
    def __defaults__(self):
        """
        Sets default values for all fuselage attributes.
        """      
        
        self.tag                                    = 'side_cabin'    
         