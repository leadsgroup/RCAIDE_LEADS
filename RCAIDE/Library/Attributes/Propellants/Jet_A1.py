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
    """
    A class representing Jet A-1 aviation kerosene fuel properties and emissions characteristics.
    Similar to Jet A but with a lower freezing point for international operations.

    Attributes
    ----------
    tag : str
        Identifier for the propellant ('Jet_A1')
    
    reactant : str
        Oxidizer used for combustion ('O2')
    
    density : float
        Fuel density in kg/m³ (804.0)
    
    specific_energy : float
        Specific energy content in J/kg (43.15e6)
    
    energy_density : float
        Energy density in J/m³ (34692.6e6)
    
    lower_heating_value : float
        Lower heating value in J/kg (43.24e6)
    
    max_mass_fraction : Data
        Maximum fuel-to-oxidizer mass ratios
        
        - Air : float
            Maximum mass fraction with air (0.0633)
        - O2 : float
            Maximum mass fraction with pure oxygen (0.3022)
    
    temperatures : Data
        Critical temperatures in K
        
        - flash : float
            Flash point (311.15)
        - autoignition : float
            Autoignition temperature (483.15)
        - freeze : float
            Freezing point (226.15)
        - boiling : float
            Boiling point (0.0)
    
    emission_indices : Data
        Emission indices in kg/kg fuel
        
        - Production : float
            CO2 production rate (0.4656)
        - CO2 : float
            Carbon dioxide (3.16)
        - H2O : float
            Water vapor (1.34)
        - SO2 : float
            Sulfur dioxide (0.0012)
        - NOx : float
            Nitrogen oxides (0.01514)
        - Soot : float
            Particulate matter (0.0012)
    
    global_warming_potential_100 : Data
        100-year global warming potentials
        
        - CO2 : float
            Carbon dioxide (1)
        - H2O : float
            Water vapor (0.06)
        - SO2 : float
            Sulfur dioxide (-226)
        - NOx : float
            Nitrogen oxides (52)
        - Soot : float
            Particulate matter (1166)
        - Contrails : float
            Contrail formation (11)

    Notes
    -----
    This class implements properties for Jet A-1 aviation kerosene, the international 
    variant of Jet A with enhanced cold-weather performance. Properties are specified 
    at standard conditions (15°C, 1 atm).

    **Definitions**
    
    'Flash Point'
        Lowest temperature at which fuel vapors will ignite
    
    'Autoignition Temperature'
        Temperature at which fuel will ignite without external ignition source
    
    'Freeze Point'
        Temperature at which fuel begins to form solid crystals

    **Major Assumptions**
    
    * Properties are for standard temperature and pressure conditions
    * Surrogate model uses three-component representation
    * Detailed model includes complex hydrocarbon mixture
    * Emission indices are for typical aircraft cruise conditions

    References
    ----------
    [1] Randall C. Boehm, Zhibin Yang, David C. Bell, John Feldhausen, Joshua S. Heyne, "Lower heating value of jet fuel from hydrocarbon class concentration data and thermo-chemical reference data: An uncertainty quantification," Fuel, Volume 311, 2022, 122542, ISSN 0016-2361, https://doi.org/10.1016/j.fuel.2021.122542.
    [2] NASA's Engine Performance Program (NEPP) 
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

   
        self.use_high_fidelity_kinetics_model      =  True 
        self.fuel_surrogate_chemical_properties    = {'N-C12H26':0.6, 'A1CH3':0.2, 'A1':0.2}
        self.fuel_chemical_properties              = {'NC10H22':0.16449, 'NC12H26':0.34308, 'NC16H34':0.10335, 'IC8H18':0.08630, 'NC7H14':0.07945, 'C6H5C2H5': 0.07348, 'C6H5C4H9': 0.05812, 'C10H7CH3': 0.10972}      # [2] More accurate kinetic mechanism, slower simulation    
        self.air_chemical_properties               = {'O2':0.2095, 'N2':0.7809, 'AR':0.0096}
        self.surrogate_species_list                = ['CO', 'CO2', 'H2O']
        self.species_list                          = ['CO', 'CO2', 'H2O', 'NO', 'NO2', 'CSOLID']
        self.surrogate_chemical_kinetics           = 'Fuel_Surrogate.yaml'
        self.chemical_kinetics                     = 'Fuel.yaml'
        self.oxidizer                              = 'Air.yaml'
        
        # critical temperatures   
        self.temperatures.flash           = 311.15                 # K
        self.temperatures.autoignition    = 483.15                 # K
        self.temperatures.freeze          = 233.15                 # K
        self.temperatures.boiling         = 0.0                    # K  
        

        self.emission_indices.Production  = 0.4656   # kg/kg Greet 
        self.emission_indices.CO2         = 3.16    # kg/kg  fuel
        self.emission_indices.H2O         = 1.34    # kg/kg  fuel 
        self.emission_indices.SO2         = 0.0012  # kg/kg  fuel
        self.emission_indices.NOx         = 0.01514 # kg/kg  fuel
        self.emission_indices.Soot        = 0.0012  # kg/kg  fuel

        self.global_warming_potential_100.CO2       = 1     # CO2e/kg  
        self.global_warming_potential_100.H2O       = 0.06  # CO2e/kg  
        self.global_warming_potential_100.SO2       = -226  # CO2e/kg  
        self.global_warming_potential_100.NOx       = 52    # CO2e/kg  
        self.global_warming_potential_100.Soot      = 1166  # CO2e/kg    
        self.global_warming_potential_100.Contrails = 11 # kg/CO2e/km  