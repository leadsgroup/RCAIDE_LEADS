# RCAIDE/Library/Attributes/Propellants/Methane.py
#  
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from .Propellant import Propellant 

# ---------------------------------------------------------------------------------------------------------------------- 
# Methane Propellant Class
# ----------------------------------------------------------------------------------------------------------------------  
class Methane(Propellant):
    """Methane class propellant  
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            Density at -162C, 1 atm
        
        Source:  
        """    
        self.tag                       = 'Methane'
        self.reactant                  = 'O2'
        self.density                   = 422.6                            # kg/m^3 (15 C, 1 atm)
        self.specific_energy           = 5.34e7                           # J/kg
        self.energy_density            = 2.26e10                          # J/m^3
        self.lower_heating_value       = 5.0e7                            # J/kg  
        
        self.stoichiometric_fuel_air_ratio = 0         # [-] Stoichiometric Fuel to Air ratio
        self.heat_of_vaporization          = 0         # [J/kg] Heat of vaporization at standard conditions
        self.temperature                   = 0         # [K] Temperature of fuel
        self.pressure                      = 0         # [Pa] Pressure of fuel
        self.fuel_surrogate_S1             = {} # [-] Mole fractions of fuel surrogate species
        self.kinetic_mechanism             = '' # [-] Kinetic mechanism for fuel surrogate species
        self.oxidizer                      = ''      
        
        self.global_warming_potential_100.CO2       = 1     # CO2e/kg  
        self.global_warming_potential_100.H2O       = 0.06  # CO2e/kg  
        self.global_warming_potential_100.SO2       = -226  # CO2e/kg  
        self.global_warming_potential_100.NOx       = 52    # CO2e/kg  
        self.global_warming_potential_100.Soot      = 1166  # CO2e/kg    
        self.global_warming_potential_100.Contrails = 11    # kg/CO2e/km          