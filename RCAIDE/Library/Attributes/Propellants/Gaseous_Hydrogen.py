# RCAIDE/Library/Attributes/Propellants/Gaseous_Hydrogen.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from .Propellant import Propellant  
from RCAIDE.Framework.Core import Data

# ---------------------------------------------------------------------------------------------------------------------- 
#  Gaseous_Hydrogen Class
# ----------------------------------------------------------------------------------------------------------------------  
class Gaseous_Hydrogen(Propellant):
    """
    A class representing gaseous hydrogen (H2) fuel properties for propulsion applications.

    Attributes
    ----------
    tag : str
        Identifier for the propellant ('hydrogen_gas')
    reactant : str
        Oxidizer used for combustion ('O2')
    specific_energy : float
        Specific energy content in J/kg (141.86e6)
    energy_density : float
        Energy density in J/m³ (5591.13e6)
    max_mass_fraction : Data
        Maximum fuel-to-oxidizer mass ratios
            - Air : float
                Maximum mass fraction with air (0.013197)
            - O2 : float
                Maximum mass fraction with pure oxygen (0.0630)
    molecular_mass : float
        Molar mass in kg/kmol (2.016)
    gas_constant : float
        Specific gas constant in J/kg-K (4124.0)
    pressure : float
        Storage pressure in Pa (700e5)
    temperature : float
        Reference temperature in K (293.0)
    compressibility_factor : float
        Gas compressibility factor (1.4699)
    density : float
        Gas density at reference conditions in kg/m³ (39.4116)

    Notes
    -----
    This class implements properties for gaseous hydrogen fuel at high pressure 
    storage conditions. Properties account for real gas behavior through the 
    compressibility factor.

    **Definitions**
    
    'Compressibility Factor'
        Correction factor for real gas behavior deviation from ideal gas law
    
    'Specific Energy'
        Energy content per unit mass, significantly higher than hydrocarbon fuels
    
    'Max Mass Fraction'
        Maximum fuel fraction for stoichiometric combustion with different oxidizers

    **Major Assumptions**
        * Real gas behavior is accounted for via compressibility factor
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """    
        self.tag                        = 'hydrogen_gas'
        self.reactant                   = 'O2'
        self.specific_energy            = 141.86e6                          # J/kg
        self.energy_density             = 5.3E6                             # J/m^3
        self.max_mass_fraction          = Data({'Air' : 0.013197, 'O2' : 0.0630})  # kg propellant / kg oxidizer
        self.lower_heating_value        = 120E6                              # J/kg
        self.stoichiometric_fuel_to_air = 0.0291 
        self.temperatures.autoignition  = 808                              # [K]
        self.molecular_mass             = 2.016                             # kg/kmol
        self.gas_constant               = 4124.0                            # J/kg-K              
        self.pressure                   = 700e5                             # Pa
        self.temperature                = 293.0                             # K
        self.compressibility_factor     = 1.4699                            # compressibility factor
        self.density                    = 39.4116                           # kg/m^3

        self.stoichiometric_fuel_air_ratio = 0         # [-] Stoichiometric Fuel to Air ratio
        self.heat_of_vaporization          = 0         # [J/kg] Heat of vaporization at standard conditions
        self.temperature                   = 0         # [K] Temperature of fuel
        self.pressure                      = 0         # [Pa] Pressure of fuel
        self.fuel_surrogate_S1             = {} # [-] Mole fractions of fuel surrogate species
        self.kinetic_mechanism             = '' # [-] Kinetic mechanism for fuel surrogate species
        self.oxidizer                      = ''
