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
        self.stoichiometric_fuel_to_air = 0.029411
        self.temperatures.autoignition  = 845.15                           # [K]