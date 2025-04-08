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
    """Gaseous hydrogen class
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            None
        """    
        self.tag                       = 'hydrogen_gas'
        self.reactant                  = 'O2'
        self.specific_energy           = 141.86e6                           # J/kg
        self.energy_density            = 5591.13e6                          # J/m^3
        self.max_mass_fraction         = Data({'Air' : 0.013197, 'O2' : 0.0630})  # kg propellant / kg oxidizer 

        # gas properties 
        self.molecular_mass            = 2.016                             # kg/kmol
        self.gas_constant              = 4124.0                            # J/kg-K              
        self.pressure                  = 700e5                             # Pa
        self.temperature               = 293.0                             # K
        self.compressibility_factor    = 1.4699                            # compressibility factor
        self.density                   = 39.4116                           # kg/m^3

        self.stoichiometric_fuel_air_ratio = 0         # [-] Stoichiometric Fuel to Air ratio
        self.heat_of_vaporization          = 0         # [J/kg] Heat of vaporization at standard conditions
        self.temperature                   = 0         # [K] Temperature of fuel
        self.pressure                      = 0         # [Pa] Pressure of fuel
        self.fuel_surrogate_S1             = {} # [-] Mole fractions of fuel surrogate species
        self.kinetic_mechanism             = '' # [-] Kinetic mechanism for fuel surrogate species
        self.oxidizer                      = ''
