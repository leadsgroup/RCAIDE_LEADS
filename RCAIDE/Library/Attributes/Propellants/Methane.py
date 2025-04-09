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
    """
    A class representing methane (CH4) fuel properties and emissions characteristics 
    for propulsion applications.

    Attributes
    ----------
    tag : str
        Identifier for the propellant ('Methane')
    reactant : str
        Oxidizer used for combustion ('O2')
    density : float
        Fuel density in kg/m³ (422.6)
    specific_energy : float
        Specific energy content in J/kg (5.34e7)
    energy_density : float
        Energy density in J/m³ (2.26e10)
    lower_heating_value : float
        Lower heating value in J/kg (5.0e7)
    use_high_fidelity_kinetics_model : bool
        Flag for using detailed chemical kinetics (False)
    fuel_surrogate_chemical_properties : dict
        Simplified chemical composition {'CH4': 1.0}
    fuel_chemical_properties : dict
        Detailed chemical composition {'CH4': 1.0}
    air_chemical_properties : dict
        Air composition for combustion calculations
        {'O2': 0.2095, 'N2': 0.7809, 'AR': 0.0096}
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
    Methane is the simplest hydrocarbon fuel and main component of natural gas. 
    It offers reduced carbon emissions compared to conventional fuels due to its 
    high hydrogen-to-carbon ratio.

    **Definitions**
    
    'Lower Heating Value'
        Heat of combustion excluding latent heat of water vapor
    
    'Global Warming Potential'
        Relative measure of heat trapped in atmosphere compared to CO2
    
    'Specific Energy'
        Energy content per unit mass, higher than conventional jet fuels

    **Major Assumptions**
        * Properties are for standard temperature and pressure conditions (15C, 1atm)
        * Pure methane composition (no higher hydrocarbons)
        * Standard atmospheric composition for air 
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