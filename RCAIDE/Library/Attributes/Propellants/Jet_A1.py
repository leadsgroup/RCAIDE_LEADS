# RCAIDE/Library/Attributes/Propellants/Jet_A1.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from .Propellant import Propellant
from RCAIDE.Framework.Core import Data  

# ---------------------------------------------------------------------------------------------------------------------- 
#  Jet_A1 Propellant Class
# ----------------------------------------------------------------------------------------------------------------------  
class Jet_A1(Propellant):
    """Jet A1 class propellant  
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            lower_heating_value: Boehm et al, Lower Heating Value of Jet Fuel From Hydrocarbon Class Concentration Data
            and Thermo-Chemical Reference Data: An Uncertainty Quantification
            
            emission indices: NASA's Engine Performance Program (NEPP) and 
    
        """    
        self.tag                       = 'Jet A1'
        self.reactant                  = 'O2'
        self.density                   = 804.0                            # kg/m^3 (15 C, 1 atm)
        self.specific_energy           = 43.15e6                          # J/kg
        self.energy_density            = 34692.6e6                        # J/m^3
        self.lower_heating_value       = 43.24e6                          # J/kg 
        self.max_mass_fraction         = Data({'Air' : 0.0633, 'O2' : 0.3022})  # kg propellant / kg oxidizer
        self.temperatures.flash        = 311.15                           # K
        self.temperatures.autoignition = 483.15                           # K
        self.temperatures.freeze       = 226.15                           # K
        self.temperatures.boiling      = 0.0                              # K

        self.stoichiometric_fuel_air_ratio = 0.068          # [-] Stoichiometric Fuel to Air ratio
        self.heat_of_vaporization          = 360000         # [J/kg] Heat of vaporization at standard conditions
        self.temperature                   = 298.15         # [K] Temperature of fuel
        self.pressure                      = 101325         # [Pa] Pressure of fuel
        self.fuel_surrogate_S1             = {'NC12H26':0.404, 'IC8H18':0.295, 'TMBENZ' : 0.073,'NPBENZ':0.228, 'C10H8':0.02} # [-] Mole fractions of fuel surrogate species
        self.kinetic_mechanism             = 'Fuel.yaml' # [-] Kinetic mechanism for fuel surrogate species
        self.oxidizer                      = 'Air.yaml'

        # critical temperatures   
        self.temperatures.flash           = 311.15                 # K
        self.temperatures.autoignition    = 483.15                 # K
        self.temperatures.freeze          = 233.15                 # K
        self.temperatures.boiling         = 0.0                    # K  
        
        self.emission_indices.Production  = 0.4656   # kg/kg Greet 
        self.emission_indices.CO2         = 3.16    # kg/kg  fuel
        self.emission_indices.H2O         = 1.23    # kg/kg  fuel 
        self.emission_indices.SO2         = 0.0012  # kg/kg  fuel
        self.emission_indices.NOx         = 0.01514 # kg/kg  fuel
        self.emission_indices.Soot        = 0.0012  # kg/kg  fuel

        self.global_warming_potential_100.CO2       = 1     # CO2e/kg  
        self.global_warming_potential_100.H2O       = 0.06  # CO2e/kg  
        self.global_warming_potential_100.SO2       = -226  # CO2e/kg  
        self.global_warming_potential_100.NOx       = 52    # CO2e/kg  
        self.global_warming_potential_100.Soot      = 1166  # CO2e/kg    
        self.global_warming_potential_100.Contrails = 11 # kg/CO2e/km  