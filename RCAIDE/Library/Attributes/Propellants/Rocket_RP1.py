# RCAIDE/Library/Attributes/RP1.py
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
#  RP1
# ---------------------------------------------------------------------------------------------------------------------- 
class Rocket_RP1(Propellant):
    """
    A class representing Rocket Propellant-1 (RP-1) properties for rocket propulsion 
    applications. 

    Attributes
    ----------
    tag : str
        Identifier for the propellant ('Rocket_RP1')
    molecular_weight : float
        Molecular weight of combustion products in kg/kmol (22.193)
    isentropic_expansion_factor : float
        Ratio of specific heats (1.1505)
    combustion_temperature : float
        Adiabatic flame temperature in K (3545.69)
    gas_specific_constant : float
        Specific gas constant for combustion products in J/(kg*K) (8314.45986/molecular_weight)

    Notes
    -----
    This class implements properties for rocket-grade RP-1, a highly refined kerosene 
    fuel specifically designed for rocket propulsion. Properties are optimized for use 
    with liquid oxygen in bipropellant rocket engines.

    **Definitions**
    
    'Isentropic Expansion Factor'
        Ratio of specific heats (cp/cv) for exhaust products
    
    'Combustion Temperature'
        Theoretical maximum temperature achieved in the combustion chamber
    
    'Gas Specific Constant'
        Individual gas constant for combustion products mixture

    **Major Assumptions**
        * Properties are for rocket engine operating conditions
        * O/F ratio of 2.27 (oxygen to fuel mass ratio)
        * Uniform mixture properties

    References
    ----------
    [1] Sutton, G. P., & Biblarz, O. (2017). Rocket Propulsion Elements. John Wiley & Sons Inc. 
    """

    def __defaults__(self):
        """This sets the default values. 
    
        Assumptions:
            Assumes an O/F ratio 2.27 
        
        Source:
            Sutton, Rocket Propulsion Elements
        """    
        self.tag                         = 'Rocket_RP1'
        self.molecular_weight            = 22.193 # [kg/kmol]
        self.isentropic_expansion_factor = 1.1505
        self.combustion_temperature      = 3545.69*Units.kelvin             #[k]
        self.gas_specific_constant       = 8314.45986/self.molecular_weight*Units['J/(kg*K)']  # [J/(Kg-K)]