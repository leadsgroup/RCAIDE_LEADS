# RCAIDE/Library/Attributes/Liquid_Hydrogen.py
# 
# 
# Created:  Sep 2023, M. Clarke
# Modified: 
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from .Propellant import Propellant 

# ----------------------------------------------------------------------------------------------------------------------
#  Liquid Hydrogen
# ----------------------------------------------------------------------------------------------------------------------  
class Liquid_Hydrogen(Propellant):
    """Liquid hydrogen fuel class 
    """

    def __defaults__(self):
        """This sets the default values.

        Assumptions:
            None

        Source: 
            http://arc.uta.edu/publications/td_files/Kristen%20Roberts%20MS.pdf 
        """ 
        
        self.tag                        = 'Liquid_H2' 
        self.reactant                   = 'O2' 
        self.density                    = 70.85                            # [kg/m^3]
        self.specific_energy            = 141.86e6                         # [J/kg] 
        self.energy_density             = 8491.0e6                         # [J/m^3] 
        self.temperatures.autoignition  = 845.15                           # [K]

        self.stoichiometric_fuel_air_ratio = 0.029411         # [-] Stoichiometric Fuel to Air ratio
        self.heat_of_vaporization          = 0         # [J/kg] Heat of vaporization at standard conditions
        self.temperature                   = 0         # [K] Temperature of fuel
        self.pressure                      = 0         # [Pa] Pressure of fuel
        self.fuel_surrogate_S1             = {} # [-] Mole fractions of fuel surrogate species
        self.kinetic_mechanism             = '' # [-] Kinetic mechanism for fuel surrogate species
        self.oxidizer                      = ''