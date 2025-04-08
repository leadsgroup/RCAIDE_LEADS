# RCAIDE/Library/Attributes/Propellants/Liquid_Petroleum_Gas.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from .Propellant import Propellant   

# ---------------------------------------------------------------------------------------------------------------------- 
#  Gaseous_Hydrogen Class
# ----------------------------------------------------------------------------------------------------------------------   
class Liquid_Petroleum_Gas(Propellant):
    """Liquid petroleum gas fuel class,
    """

    def __defaults__(self):
        """This sets the default values. 
    
    Assumptions:
        None
    
    Source:
        None
        """    
        self.tag             = 'Liquid_Petroleum_Gas'
        self.reactant        = 'O2'
        self.density         = 509.26                           # kg/m^3 
        self.specific_energy = 43.1e6                           # J/kg
        self.energy_density  = 21949.1e6                        # J/m^3
        
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