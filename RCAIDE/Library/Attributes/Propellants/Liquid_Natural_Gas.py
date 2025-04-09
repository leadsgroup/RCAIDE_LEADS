# RCAIDE/Library/Attributes/Propellants/Liquid_Natural_Gas.py
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
class Liquid_Natural_Gas(Propellant):
    """
    A class representing Liquid Natural Gas (LNG) fuel properties and composition 
    for propulsion applications.

    Attributes
    ----------
    tag : str
        Identifier for the propellant ('Liquid_Natural_Gas')
    reactant : str
        Oxidizer used for combustion ('O2')
    density : float
        Fuel density in kg/m³ (414.2)
    specific_energy : float
        Specific energy content in J/kg (53.6e6)
    energy_density : float
        Energy density in J/m³ (22200.0e6)
    use_high_fidelity_kinetics_model : bool
        Flag for using detailed chemical kinetics (False)
    fuel_surrogate_chemical_properties : dict
        Simplified chemical composition for surrogate model
            - CH4 : float
                Methane fraction (0.85)
            - C2H6 : float
                Ethane fraction (0.1)
            - C3H8 : float
                Propane fraction (0.05)
    fuel_chemical_properties : dict
        Detailed chemical composition
            - CH4 : float
                Methane fraction (0.83)
            - C2H6 : float
                Ethane fraction (0.1)
            - C3H8 : float
                Propane fraction (0.05)
            - C4H10 : float
                Butane fraction (0.02)
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
    LNG is a cryogenic fuel consisting primarily of methane with small amounts of 
    heavier hydrocarbons. It requires storage at approximately -162°C but offers 
    reduced carbon emissions compared to conventional fuels.

    **Definitions**
    
    'Surrogate Model'
        Simplified three-component representation for faster calculations
    
    'Global Warming Potential'
        Relative measure of heat trapped in atmosphere compared to CO2
    
    'Energy Density'
        Energy content per unit volume, affected by cryogenic storage conditions

    **Major Assumptions**
        * Properties are for saturated liquid at atmospheric pressure
        * Composition represents typical LNG mixture
    """

    def __defaults__(self):
        """This sets the default values. 
    
    Assumptions:
        None
    
    Source:
        None
        """    
        self.tag             = 'Liquid_Natural_Gas'
        self.reactant        = 'O2'
        self.density         = 414.2                            # kg/m^3 
        self.specific_energy = 53.6e6                           # J/kg
        self.energy_density  = 22200.0e6                        # J/m^3
        
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