# RCAIDE/Library/Attributes/Propellants/Jet_A1.py
# 
#
# Created:  Mar 2024, M. Clarke
# Updated:  Mar 2025, M. Guidotti

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from .Propellant import Propellant
from RCAIDE.Framework.Core import Data  
import numpy as np
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
        self.tag                                    = 'Jet A1'
        self.reactant                               = 'O2'
        self.density                                = 804.0     # [kg/m^3] (15 C, 1 atm)
        self.specific_energy                        = 43.15e6   # [J/kg]
        self.energy_density                         = 34692.6e6 # [J/m^3]
        self.lower_heating_value                    = 43.24e6   # [J/kg] 
        self.heat_of_vaporization                   = 360000    # [J/kg] Heat of vaporization at standard conditions
        self.max_mass_fraction                      = Data({'Air' : 0.0633, 'O2' : 0.3022}) # [kg propellant / kg oxidizer] 
        self.stoichiometric_fuel_air_ratio          = 0.068     # [-] Stoichiometric Fuel to Air ratio
        self.temperature                            = 298.15    # [K] Temperature of fuel
        self.pressure                               = 101325    # [Pa] Pressure of fuel
        self.nuc_fac                                = 1e-5      # [-] Nucleation scaling factor
        self.sg_fac                                 = 0.25      # [-] Surface growth scaling factor
        self.ox_fac                                 = 0.3       # [-] Oxidation scaling factor
        self.L                                      = 5         # [-] ???
        self.M_mech                                 = 0.0       # [-] ???
        self.PAH_species                            = np.array([152.19, 154.2078, 166.22, 178.23, 202.25])    # [g/mol] ??? [kmol/m**3] Concentration of PAH species
        self.radii                                  = np.array([3.5e-10, 3.5e-10, 4.5e-10, 4.5e-10, 3.5e-10]) # [m] Radii of PAH species 
        self.mu_matrix                              = np.array([[76.095, 76.59612792, 79.44795013, 82.09195478, 86.84242044],
                                                                [76.59612792, 77.1039,     79.99437164, 82.67548454, 87.49570791],
                                                                [79.44795013, 79.99437164, 83.11      , 86.00781129, 91.23672212],
                                                                [82.09195478, 82.67548454, 86.00781129, 89.115     , 94.74089965],
                                                                [86.84242044, 87.49570791, 91.23672212, 94.74089965, 101.125]]) # [???]
        self.n_C_matrix                             = np.array([[24, 24, 25, 26, 28],
                                                                [24, 24, 25, 26, 28],
                                                                [25, 25, 26, 27, 29],
                                                                [26, 26, 27, 28, 30],
                                                                [28, 28, 29, 30, 32]]) # [???]

        self.fuel_surrogate_S1                      = {'NC12H26':0.404, 'IC8H18':0.295, 'TMBENZ' : 0.073,'NPBENZ':0.228, 'C10H8':0.02}
        self.fuel_surrogate_S2                      = {'NC12H26':0.303, 'MCYC6':0.485, 'XYLENE' : 0.212, 'C10H8':0.02}
        self.fuel_surrogate_S3                      = {'NC12H26':0.384, 'MCYC6':0.234, 'IC16H34' : 0.148,'C7H8':0.234, 'C10H8':0.02}
        self.fuel_surrogate_S4                      = {'NC12H26':0.290, 'IC16H34' : 0.142,'C7H8':0.249, 'DECALIN':0.319, 'C10H8':0.02}
        self.fuel_surrogate_S5                      = {'NC12H26':0.371, 'IC8H18':0.02, 'IC16H34' : 0.206,'C7H8':0.259, 'DECALIN':0.145, 'C10H8':0.02}
        self.kinetic_mechanism                      = 'POLIMI_PRF_PAH_RFUELS_HT_1412.yaml'
        
        # critical temperatures   
        self.temperatures.flash                     = 311.15  # [K]
        self.temperatures.autoignition              = 483.15  # [K]
        self.temperatures.freeze                    = 233.15  # [K]
        self.temperatures.boiling                   = 0.0     # [K]  
        
        self.emission_indices.Production            = 0.4656  # [kg/kg Greet] 
        self.emission_indices.CO2                   = 3.16    # [kg/kg  fuel]
        self.emission_indices.H2O                   = 1.23    # [kg/kg  fuel] 
        self.emission_indices.SO2                   = 0.0012  # [kg/kg  fuel]
        self.emission_indices.NOx                   = 0.01514 # [kg/kg  fuel]
        self.emission_indices.Soot                  = 0.0012  # [kg/kg  fuel]

        self.global_warming_potential_100.CO2       = 1       # [CO2e/kg]  
        self.global_warming_potential_100.H2O       = 0.06    # [CO2e/kg]  
        self.global_warming_potential_100.SO2       = -226    # [CO2e/kg]  
        self.global_warming_potential_100.NOx       = 52      # [CO2e/kg]  
        self.global_warming_potential_100.Soot      = 1166    # [CO2e/kg]    
        self.global_warming_potential_100.Contrails = 11      # [kg/CO2e/km]  