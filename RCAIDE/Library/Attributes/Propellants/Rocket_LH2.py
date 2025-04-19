# RCAIDE/Library/Attributes/LH2.py
# 
# 
# Created:  Sep 2023, M. Clarke
# Modified: 
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from .Propellant import Propellant
from RCAIDE.Framework.Core import Units

# ----------------------------------------------------------------------------------------------------------------------
#  Rocket_LH2
# ----------------------------------------------------------------------------------------------------------------------  
class Rocket_LH2(Propellant):
    """LH2 Rocket Fuel Class
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            Assujmes an O/F ratio 5.50 
        
        Source:
            Sutton, Rocket Propulsion Elements Using CEA
        """    
        self.tag                         = 'Rocket_LH2'
        self.molecular_weight            = 12.644                             # [kg/kmol]
        self.isentropic_expansion_factor = 1.145
        self.combustion_temperature      = 3331.0*Units.kelvin                # [K]                      
        self.gas_specific_constant       = (8314.45986/self.molecular_weight)*Units['J/(kg*K)'] # [J/(kg-K)]