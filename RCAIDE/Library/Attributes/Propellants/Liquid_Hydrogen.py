# RCAIDE/Library/Attributes/Liquid_Hydrogen.py
# 
# 
# Created:  Sep 2023, M. Clarke
# Modified: 
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
from .Propellant import Propellant 

# ----------------------------------------------------------------------------------------------------------------------
#  Liquid Hydrogen
# ----------------------------------------------------------------------------------------------------------------------  
class Liquid_Hydrogen(Propellant):
    """
    A class representing liquid hydrogen (LH2) fuel properties for aviation applications.

    Attributes
    ----------
    tag : str
        Identifier for the propellant ('Liquid_H2')
    reactant : str
        Oxidizer used for combustion ('O2')
    density : float
        Fuel density in kg/m³ (59.9)
    specific_energy : float
        Specific energy content in J/kg (141.86e6)
    energy_density : float
        Energy density in J/m³ (8491.0e6)
    stoichiometric_fuel_to_air : float
        Stoichiometric fuel-to-air ratio (0.0291)
    temperatures : Data
        Critical temperatures
            - autoignition : float
                Autoignition temperature in K (845.15)

    Notes
    -----
    Liquid hydrogen represents a zero-carbon aviation fuel option with the highest 
    specific energy of any fuel, but requires cryogenic storage at extremely low 
    temperatures (-253°C).

    **Definitions**
    
    'Specific Energy'
        Energy content per unit mass, approximately 3 times higher than kerosene
    
    'Energy Density'
        Energy content per unit volume, lower than conventional fuels due to low density
    
    'Stoichiometric Fuel-to-Air Ratio'
        Ideal fuel-to-air mass ratio for complete combustion to H2O

    **Major Assumptions**
        * Properties are for liquid hydrogen
        * No consideration of boil-off losses

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
        
        self.tag                        = 'Liquid_H2' 
        self.reactant                   = 'O2' 
        self.density                    = 70.85                            # [kg/m^3] 
        self.specific_energy            = 141.86e6                         # [J/kg] 
        self.energy_density             = 8491.0e6                         # [J/m^3] 
        self.stoichiometric_fuel_to_air = 0.029411 
        self.temperatures.autoignition  = 845.15                           # [K]         