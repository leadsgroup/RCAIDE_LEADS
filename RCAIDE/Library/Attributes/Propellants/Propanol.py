# RCAIDE/Library/Attributes/Propellants/Propanol.py
#  
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from .Propellant import Propellant 

# ---------------------------------------------------------------------------------------------------------------------- 
#  Propanol Propellant Class
# ----------------------------------------------------------------------------------------------------------------------  
class Propanol(Propellant):
    """
    A class representing propanol (C3H7OH) fuel properties and emissions characteristics 
    for propulsion applications.

    Attributes
    ----------
    tag : str
        Identifier for the propellant ('Propanol')
    reactant : str
        Oxidizer used for combustion ('O2')
    density : float
        Fuel density in kg/m³ (804.16)
    specific_energy : float
        Specific energy content in J/kg (3.34e7)
    energy_density : float
        Energy density in J/m³ (2.69e10)
    lower_heating_value : float
        Lower heating value in J/kg (3.07e7)
    use_high_fidelity_kinetics_model : bool
        Flag for using detailed chemical kinetics (False)
    fuel_surrogate_chemical_properties : dict
        Simplified chemical composition {'NC3H7OH': 1.0}
    fuel_chemical_properties : dict
        Detailed chemical composition for high-fidelity model 
            - NC10H22 : float
                n-Decane fraction (0.16449)
            - NC12H26 : float
                n-Dodecane fraction (0.34308)
            - NC16H34 : float
                n-Hexadecane fraction (0.10335)
            - IC8H18 : float
                iso-Octane fraction (0.08630)
            - NC7H14 : float
                n-Heptene fraction (0.07945)
            - C6H5C2H5 : float
                Ethylbenzene fraction (0.07348)
            - C6H5C4H9 : float
                Butylbenzene fraction (0.05812)
            - C10H7CH3 : float
                Methylnaphthalene fraction (0.10972)
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
    Propanol is an alcohol fuel that can be produced through biological or synthetic 
    processes. It has properties similar to other light alcohols.

    **Definitions**
    
    'Lower Heating Value'
        Heat of combustion excluding latent heat of water vapor
    
    'Global Warming Potential'
        Relative measure of heat trapped in atmosphere compared to CO2
    
    'Surrogate Model'
        Simplified single-component representation using pure propanol

    **Major Assumptions**
        * Properties are for standard temperature (20°C) and pressure (1 atm)
        * Air composition is standard atmospheric
    """

    def __defaults__(self):
        """This sets the default values.
    
        Assumptions:
            Density at 20C 1 atm
        
        Source: 
    
        """    
        self.tag                       = 'Propanol'
        self.reactant                  = 'O2'
        self.density                   = 804.16                            # kg/m^3 (15 C, 1 atm)
        self.specific_energy           = 3.34e7                           # J/kg
        self.energy_density            = 2.69e10                          # J/m^3
        self.lower_heating_value       = 3.07e7                            # J/kg  

        
        self.use_high_fidelity_kinetics_model      =  False 
        self.fuel_surrogate_chemical_properties    = {'NC3H7OH':1.0}
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