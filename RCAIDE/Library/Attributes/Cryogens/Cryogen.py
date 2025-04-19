# RCAIDE/Library/Attributes/Cryogens/Cryogens.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from RCAIDE.Framework.Core import Data

# ---------------------------------------------------------------------------------------------------------------------- 
# Cryogen
# ----------------------------------------------------------------------------------------------------------------------  
class Cryogen(Data):
    """Defauly cryogenic liquid class 
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """    
        self.tag                       = 'Cryogen'
        self.density                   = 0.0                       # kg/m^3
        self.specific_energy           = 0.0                       # MJ/kg
        self.energy_density            = 0.0                       # MJ/m^3
        self.temperatures              = Data()
        self.temperatures.freeze       = 0.0                       # K
        self.temperatures.boiling      = 0.0                       # K