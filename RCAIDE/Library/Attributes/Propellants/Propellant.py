# RCAIDE/Library/Attributes/Propellants.py
# 
# 
# Created:  Sep 2023, M. Clarke
# Modified: 

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 

from RCAIDE.Framework.Core import Data
from RCAIDE.Library.Components.Mass_Properties import Mass_Properties

# ----------------------------------------------------------------------------------------------------------------------
#  Propellant
# ----------------------------------------------------------------------------------------------------------------------  
class Propellant(Data):
    """
    Base class for defining propellant properties and characteristics in propulsion systems.

    Attributes
    ----------
    tag : str
        Identifier for the specific propellant type
    reactant : str
        Primary oxidizer used for combustion
    density : float
        Fuel density in kg/m³
    specific_energy : float
        Specific energy content in J/kg
    energy_density : float
        Energy density in J/m³
    lower_heating_value : float
        Lower heating value in J/kg
    mass_properties : Mass_Properties
        Object containing mass-related properties
    max_mass_fraction : Data
        Maximum fuel-to-oxidizer mass ratios
            - Air : float
                Maximum mass fraction with air
            - O2 : float
                Maximum mass fraction with pure oxygen
    temperatures : Data
        Critical temperatures in K
            - flash : float
                Flash point temperature
            - autoignition : float
                Autoignition temperature
            - freeze : float
                Freezing point temperature
            - boiling : float
                Boiling point temperature
    emission_indices : Data
        Emission indices in kg/kg fuel
            - Production : float
                CO2 production rate
            - CO2 : float
                Carbon dioxide emissions
            - H2O : float
                Water vapor emissions
            - SO2 : float
                Sulfur dioxide emissions
            - NOx : float
                Nitrogen oxides emissions
            - Soot : float
                Particulate matter emissions
    global_warming_potential_100 : Data
        100-year global warming potentials
            - CO2 : float
                Carbon dioxide impact
            - H2O : float
                Water vapor impact
            - SO2 : float
                Sulfur dioxide impact
            - NOx : float
                Nitrogen oxides impact
            - Soot : float
                Particulate matter impact
            - Contrails : float
                Contrail formation impact

    Notes
    -----
    This base class provides a standardized structure for defining propellant 
    properties, including physical characteristics, combustion parameters, and 
    environmental impacts. It serves as a template for specific propellant 
    implementations.

    **Definitions**
    
    'Emission Index'
        Mass of pollutant produced per unit mass of fuel burned
    
    'Global Warming Potential'
        Relative measure of heat trapped in atmosphere compared to CO2
    
    'Lower Heating Value'
        Heat of combustion excluding latent heat of water vapor
    """

    def __defaults__(self):
        """This sets the default values. 
    
        Assumptions:
            None
        
        Source:
            None
        """    
        self.tag                       = 'Propellant'
        self.reactant                  = 'O2'
        self.density                   = 0.0                       # kg/m^3
        self.specific_energy           = 0.0                       # MJ/kg
        self.energy_density            = 0.0                       # MJ/m^3
        self.lower_heating_value       = 0.0                       # MJ/kg
        self.mass_properties           = Mass_Properties()
        self.max_mass_fraction         = Data({'Air' : 0.0, 'O2' : 0.0}) # kg propellant / kg oxidizer
        self.temperatures              = Data()
        self.temperatures.flash        = 0.0                       # K
        self.temperatures.autoignition = 0.0                       # K
        self.temperatures.freeze       = 0.0                       # K
        self.temperatures.boiling      = 0.0                       # K

        self.stoichiometric_fuel_air_ratio = 0         # [-] Stoichiometric Fuel to Air ratio
        self.heat_of_vaporization          = 0         # [J/kg] Heat of vaporization at standard conditions
        self.temperature                   = 0         # [K] Temperature of fuel
        self.pressure                      = 0         # [Pa] Pressure of fuel
        self.fuel_surrogate_S1             = {} # [-] Mole fractions of fuel surrogate species
        self.kinetic_mechanism             = '' # [-] Kinetic mechanism for fuel surrogate species
        self.oxidizer                      = ''
        
        # Emission Indices 
        self.emission_indices             =  Data() 
        self.emission_indices.Production  = 0
        self.emission_indices.CO2         = 0
        self.emission_indices.H2O         = 0
        self.emission_indices.SO2         = 0
        self.emission_indices.NOx         = 0
        self.emission_indices.Soot        = 0 

        self.global_warming_potential_100           =  Data() 
        self.global_warming_potential_100.CO2       = 0
        self.global_warming_potential_100.H2O       = 0
        self.global_warming_potential_100.SO2       = 0
        self.global_warming_potential_100.NOx       = 0
        self.global_warming_potential_100.Soot      = 0  
        self.global_warming_potential_100.Contrails = 0        