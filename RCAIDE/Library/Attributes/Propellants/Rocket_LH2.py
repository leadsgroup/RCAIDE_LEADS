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
    """
    A class representing liquid hydrogen (LH2) fuel properties specifically for rocket 
    propulsion applications. Optimized for high-performance rocket engines with 
    oxygen as oxidizer.

    Attributes
    ----------
    tag : str
        Identifier for the propellant ('Rocket_LH2')
    molecular_weight : float
        Molecular weight in kg/kmol (12.644)
    isentropic_expansion_factor : float
        Ratio of specific heats (1.145)
    combustion_temperature : float
        Adiabatic flame temperature in K (3331.0)
    gas_specific_constant : float
        Specific gas constant in J/(kg*K) (8314.45986/molecular_weight)

    Notes
    -----
    This class implements properties for rocket-grade liquid hydrogen, focusing on 
    parameters relevant to high-performance rocket engine applications. Properties 
    are optimized for use with liquid oxygen in bipropellant rocket engines.

    **Definitions**
    
    'Isentropic Expansion Factor'
        Ratio of specific heats (cp/cv) for exhaust products
    
    'Combustion Temperature'
        Theoretical maximum temperature achieved in the combustion chamber
    
    'Gas Specific Constant'
        Individual gas constant for combustion products

    **Major Assumptions**
        * Properties are for rocket engine operating conditions
        * O/F ratio of 5.50 (oxygen to fuel mass ratio)

    References
    ----------
    [1] Sutton, G. P., & Biblarz, O. (2017). Rocket Propulsion Elements. John Wiley & Sons Inc. 
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