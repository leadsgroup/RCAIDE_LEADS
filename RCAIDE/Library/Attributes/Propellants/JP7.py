# RCAIDE/Library/Attributes/Propellants/JP7.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 

from .Propellant import Propellant 

# ---------------------------------------------------------------------------------------------------------------------- 
#  JP7 Propellant Class
# ----------------------------------------------------------------------------------------------------------------------  
class JP7(Propellant):
    """
    A class representing JP-7 high thermal stability jet fuel properties. This specialized 
    fuel was developed for high-speed aircraft operating at elevated temperatures.

    Attributes
    ----------
    tag : str
        Identifier for the propellant ('JP7')
    reactant : str
        Oxidizer used for combustion ('O2')
    density : float
        Fuel density in kg/m³ (803.0)
    specific_energy : float
        Specific energy content in J/kg (43.50e6)
    energy_density : float
        Energy density in J/m³ (34930.5e6)
    stoichiometric_fuel_to_air : float
        Stoichiometric fuel-to-air ratio (0.0674)
    temperatures : Data
        Critical temperatures in K
            - flash : float
                Flash point (333.15)
            - autoignition : float
                Autoignition temperature (555.15)
            - freeze : float
                Freezing point (514.15)

    Notes
    -----
    JP-7 is a specialized jet fuel developed for the SR-71 Blackbird and similar 
    high-speed aircraft. It features high thermal stability and low volatility for 
    operation at elevated temperatures.

    **Definitions**
    
    'Flash Point'
        Lowest temperature at which fuel vapors will ignite (60°C, significantly 
        higher than conventional jet fuels)
    
    'Thermal Stability'
        Resistance to thermal decomposition at elevated temperatures
    
    'Stoichiometric Fuel-to-Air Ratio'
        Ideal fuel-to-air mass ratio for complete combustion

    **Major Assumptions**
        * Properties are for standard temperature and pressure conditions (15C, 1atm)

    References
    ----------
    [1] Roberts, K. (2008). ANALYSIS AND DESIGN OF A HYPERSONIC SCRAMJET ENGINE WITH A STARTING MACH NUMBER OF 4.00 (thesis). UTA. University of Texas at Austin. Retrieved December 30, 2024, from https://arc.uta.edu/publications/td_files/Kristen%20Roberts%20MS.pdf. 
    """

    def __defaults__(self):
        """This sets the default values.

        Assumptions:
            None

        Source: 
            http://arc.uta.edu/publications/td_files/Kristen%20Roberts%20MS.pdf 
        """    
        self.tag                        = 'JP7'
        self.reactant                   = 'O2'
        self.density                    = 803.0                          # kg/m^3 (15 C, 1 atm)
        self.specific_energy            = 43.50e6                        # J/kg
        self.energy_density             = 34930.5e6                      # J/m^3
        self.stoichiometric_fuel_to_air = 0.0674            

        # critical temperatures
        self.temperatures.flash        = 333.15                 # K
        self.temperatures.autoignition = 555.15                 # K
        self.temperatures.freeze       = 514.15                 # K