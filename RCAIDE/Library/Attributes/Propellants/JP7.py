# RCAIDE/Library/Attributes/Propellants/JP7.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from .Propellant import Propellant 

# ---------------------------------------------------------------------------------------------------------------------- 
#  JP7 Propellant Class
# ----------------------------------------------------------------------------------------------------------------------  
class JP7(Propellant):
    """JP7 Jet fuel class
    """

    def __defaults__(self):
        """This sets the default values.

        Assumptions:
            None

        Source: 
            http://arc.uta.edu/publications/td_files/Kristen%20Roberts%20MS.pdf 
        """    
        self.tag                        = 'JP7'
        self.reactant                   = 'O2'
        self.density                    = 803.0                          # kg/m^3 (15 C, 1 atm)
        self.specific_energy            = 43.50e6                        # J/kg
        self.energy_density             = 34930.5e6                      # J/m^3
        self.stoichiometric_fuel_to_air = 0.0674            

        # critical temperatures
        self.temperatures.flash        = 333.15                 # K
        self.temperatures.autoignition = 555.15                 # K
        self.temperatures.freeze       = 514.15                 # K