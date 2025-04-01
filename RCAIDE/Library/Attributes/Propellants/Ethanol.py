# RCAIDE/Library/Attributes/Propellants/Ethanol.py
#  
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from .Propellant import Propellant 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Ethanol Propellant Class
# ----------------------------------------------------------------------------------------------------------------------  
class Ethanol(Propellant):
    """
    A class representing ethanol (C2H5OH) fuel properties for propulsion applications.

    Attributes
    ----------
    tag : str
        Identifier for the propellant ('Ethanol')
    reactant : str
        Oxidizer used for combustion ('O2')
    density : float
        Fuel density in kg/m³ (793.67)
    specific_energy : float
        Specific energy content in J/kg (2.68e7)
    energy_density : float
        Energy density in J/m³ (2.13e10)
    lower_heating_value : float
        Lower heating value in J/kg (2.67e7)
    use_high_fidelity_kinetics_model : bool
        Flag for using detailed chemical kinetics (False)
    fuel_surrogate_chemical_properties : dict
        Simplified chemical composition for surrogate model {'C2H5OH': 1.0}
    fuel_chemical_properties : dict
        Detailed chemical composition for high-fidelity model
    air_chemical_properties : dict
        Air composition for combustion calculations {'O2':0.2095, 'N2':0.7809, 'AR':0.0096}
    surrogate_species_list : list
        Species considered in surrogate model ['CO', 'CO2', 'H2O']
    species_list : list
        Species considered in detailed model ['CO', 'CO2', 'H2O', 'NO', 'NO2', 'CSOLID']
    global_warming_potential_100 : Data
        100-year global warming potential for emissions
            - CO2 : float
                GWP for carbon dioxide (1)
            - H2O : float
                GWP for water vapor (0.06)
            - SO2 : float
                GWP for sulfur dioxide (-226)
            - NOx : float
                GWP for nitrogen oxides (52)
            - Soot : float
                GWP for particulate matter (1166)
            - Contrails : float
                GWP for contrail formation (11)

    Notes
    -----
    This class implements properties for ethanol fuel, a renewable biofuel commonly 
    used in both pure form and as a gasoline additive. Properties are specified at 
    standard conditions (15°C, 1 atm).

    **Definitions**
    
    'Lower Heating Value'
        Heat of combustion excluding latent heat of water vapor
    
    'Global Warming Potential'
        Relative measure of heat trapped in atmosphere compared to CO2
    
    'Surrogate Model'
        Simplified chemical kinetics model using pure ethanol

    **Major Assumptions**
        * Properties are for anhydrous (pure) ethanol
        * Properties are for standard temperature and pressure conditions (15C, 1atm)
        * Air composition is standard atmospheric
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            Density at 15C, 1 atm
        
        Source: 
    
        """    
        self.tag                       = 'Ethanol'
        self.reactant                  = 'O2'
        self.density                   = 793.67                           # kg/m^3 (15 C, 1 atm)
        self.specific_energy           = 2.68e7                           # J/kg
        self.energy_density            = 2.13e10                          # J/m^3
        self.lower_heating_value       = 2.67e7                           # J/kg  
        

        self.use_high_fidelity_kinetics_model      =  False 
        self.fuel_surrogate_chemical_properties    = {'C2H5OH':1.0}        
        self.fuel_chemical_properties              = {'NC10H22':0.16449, 'NC12H26':0.34308, 'NC16H34':0.10335, 'IC8H18':0.08630, 'NC7H14':0.07945, 'C6H5C2H5': 0.07348, 'C6H5C4H9': 0.05812, 'C10H7CH3': 0.10972}      # [2] More accurate kinetic mechanism, slower simulation    
        self.air_chemical_properties               = {'O2':0.2095, 'N2':0.7809, 'AR':0.0096}
        self.surrogate_species_list                = ['CO', 'CO2', 'H2O']
        self.species_list                          = ['CO', 'CO2', 'H2O', 'NO', 'NO2', 'CSOLID']
        self.surrogate_chemical_kinetics           = 'Fuel_Surrogate.yaml'
        self.chemical_kinetics                     = 'Fuel.yaml'
        self.oxidizer                              = 'Air.yaml'
        
        self.global_warming_potential_100.CO2       = 1     # CO2e/kg  
        self.global_warming_potential_100.H2O       = 0.06  # CO2e/kg  
        self.global_warming_potential_100.SO2       = -226  # CO2e/kg  
        self.global_warming_potential_100.NOx       = 52    # CO2e/kg  
        self.global_warming_potential_100.Soot      = 1166  # CO2e/kg    
        self.global_warming_potential_100.Contrails = 11    # kg/CO2e/km          