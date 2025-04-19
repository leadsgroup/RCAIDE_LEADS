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
   """Generic class for carbon dioxide 
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
 