# RCAIDE/Library/Attributes/Propellants/Jet_A.py
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
class Jet_A(Propellant):
    """
    A class representing Jet A aviation kerosene fuel properties and emissions characteristics.

    Attributes
    ----------
    tag : str
        Identifier for the propellant ('Jet_A')
    reactant : str
        Oxidizer used for combustion ('O2')
    density : float
        Fuel density in kg/m³ (820.0)
    specific_energy : float
        Specific energy content in J/kg (43.02e6)
    energy_density : float
        Energy density in J/m³ (35276.4e6)
    lower_heating_value : float
        Lower heating value in J/kg (43.24e6)
    max_mass_fraction : Data
        Maximum fuel-to-oxidizer mass ratios
            - Air : float
                Maximum mass fraction with air (0.0633)
            - O2 : float
            Maximum mass fraction with pure oxygen (0.3022)
    use_high_fidelity_kinetics_model : bool
        Flag for using detailed chemical kinetics (True)
    fuel_surrogate_chemical_properties : dict
        Simplified surrogate composition
            - N-C12H26 : float
                Dodecane fraction (0.6)
            - A1CH3 : float
                Toluene fraction (0.2)
            - A1 : float
                Benzene fraction (0.2)
    fuel_chemical_properties : dict
        Detailed chemical composition for high-fidelity model
    emission_indices : Data
        Emission indices in kg/kg fuel
            - Production : float
                CO2 production rate (0.4656)
            - CO2 : float
                Carbon dioxide (3.16)
            - CO : float
                Carbon monoxide (0.00201)
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
    This class implements properties for Jet A aviation kerosene, including both 
    simplified and detailed chemical kinetics options. Properties are specified at 
    standard conditions (15°C, 1 atm).

    **Definitions**
    
    'Emission Index'
        Mass of pollutant produced per mass of fuel burned
    
    'Global Warming Potential'
        Relative measure of heat trapped in atmosphere compared to CO2
    
    'Surrogate Model'
        Simplified three-component representation of complex fuel mixture

    **Major Assumptions**
        * Properties are for standard temperature and pressure conditions (15C, 1atm)
        * Surrogate model uses three-component representation
        * Detailed model includes complex hydrocarbon mixture
        * Emission indices are for typical aircraft cruise conditions

    References
    ----------
    [1] D.S. Lee, D.W. Fahey, A. Skowron, M.R. Allen, U. Burkhardt, Q. Chen, S.J. Doherty, S. Freeman, P.M. Forster, J. Fuglestvedt, A. Gettelman, R.R. De León, L.L. Lim, M.T. Lund, R.J. Millar, B. Owen, J.E. Penner, G. Pitari, M.J. Prather, R. Sausen, L.J. Wilcox, "The contribution of global aviation to anthropogenic climate forcing for 2000 to 2018," Atmospheric Environment, Volume 244, 2021, 117834, ISSN 1352-2310, https://doi.org/10.1016/j.atmosenv.2020.117834.
    [2] Chevron Products Company. (n.d.). Aviation fuels. Chevron. https://www.chevron.com/-/media/chevron/operations/documents/aviation-tech-review.pdf 
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            None
        
        Source:
            Emission Indices from Lee, David S., et al. "The contribution of global aviation to anthropogenic climate forcing
            for 2000 to 2018." Atmospheric environment 244 (2021): 117834
            
            
            
            
        """    
        self.tag                          = 'Jet_A'
        self.reactant                     = 'O2'
        self.density                      = 820.0                          # kg/m^3 (15 C, 1 atm)
        self.specific_energy              = 43.02e6                        # J/kg
        self.energy_density               = 35276.4e6                      # J/m^3
        self.lower_heating_value          = 43.24e6                          # J/kg 
        self.max_mass_fraction            = Data({'Air' : 0.0633,'O2' : 0.3022})   # kg propellant / kg oxidizer
   
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
        self.emission_indices.CO2         = 3.16    # kg/kg
        self.emission_indices.CO          = 0.00201 # kg/kg
        self.emission_indices.H2O         = 1.34    # kg/kg  
        self.emission_indices.SO2         = 0.0012  # kg/kg
        self.emission_indices.NOx         = 0.01514 # kg/kg
        self.emission_indices.Soot        = 0.0012  # kg/kg
        
        self.global_warming_potential_100.CO2       = 1     # CO2e/kg  
        self.global_warming_potential_100.H2O       = 0.06  # CO2e/kg  
        self.global_warming_potential_100.SO2       = -226  # CO2e/kg  
        self.global_warming_potential_100.NOx       = 52    # CO2e/kg  
        self.global_warming_potential_100.Soot      = 1166  # CO2e/kg    
        self.global_warming_potential_100.Contrails = 11 # kg/CO2e/km  