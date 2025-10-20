# RCAIDE/Library/Attributes/Liquid_Hydrogen.py
# 
# 
# Created:  Sep 2023, M. Clarke
# Modified: 
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from pylab import fill
from .Propellant import Propellant 

import os
import numpy as np
from scipy.interpolate  import interp1d 

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
        
        self.tag                           = 'Liquid_H2' 
        self.reactant                      = 'O2' 
        self.density                       = 70.85                            # [kg/m^3]
        self.specific_energy               = 141.86e6                         # [J/kg] 
        self.energy_density                = 8491.0e6                         # [J/m^3] 
        self.gravimetric_efficiency        = .3
        self.stoichiometric_fuel_to_air    = 0.029411 
        self.temperatures.autoignition     = 845.15                           # [K]  
        self.stoichiometric_fuel_air_ratio = 0.029411         # [-] Stoichiometric Fuel to Air ratio
        self.heat_of_vaporization          = 0         # [J/kg] Heat of vaporization at standard conditions
        self.temperature                   = 20         # [K] Temperature of fuel
        self.pressure                      = 0         # [Pa] Pressure of fuel
        self.fuel_surrogate_S1             = {} # [-] Mole fractions of fuel surrogate species
        self.kinetic_mechanism             = '' # [-] Kinetic mechanism for fuel surrogate species
        self.oxidizer                      = ''       

        self.materials_properties = self.liquid_hydrogen_properties()

    def liquid_hydrogen_properties(self, T, prop_name):
        """
            Return interpolated liquid hydrogen property value at a given temperature.

            Parameters
            ----------
            T : float or ndarray
                Temperature(s) in Kelvin at which the property is requested.  
            prop_name : str
                Name of the property to retrieve from the hydrogen data file.  
                Valid keys include:
                    - "Temperature (K)"
                    - "Pressure (MPa)"
                    - "Density (kg/m3)"
                    - "Volume (m3/kg)"
                    - "Internal Energy (kJ/kg)"
                    - "Enthalpy (kJ/kg)"
                    - "Entropy (J/g*K)"
                    - "Cv (J/g*K)"
                    - "Cp (J/g*K)"
                    - "Sound Spd. (m/s)"
                    - "Joule-Thomson (K/MPa)"
                    - "Viscosity (Pa*s)"
                    - "Therm. Cond. (W/m*K)"
                    - "Phase"

            Returns
            -------
            prop_value : float or ndarray
                Interpolated property value(s) corresponding to the input temperature(s).  

            Notes
            -----
            * Property data is loaded from ``H2_properties.res`` using 
            :func:`load_hydrogen_properties`.  
            * Linear interpolation is applied between tabulated values.  
            * Extrapolation outside the data range is not supported (``fill_value=None``).  
            * Phase information is categorical and may not be suitable for interpolation.  

            See Also
            --------
            RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen.load_hydrogen_properties
         """
        data = load_hydrogen_properties()
        temps = np.array(data["Temperature (K)"], dtype=float)
        props = np.array(data[prop_name], dtype=float)
        interp = interp1d(temps, props, kind="linear", fill_value=None)
        
        return interp(T)

def load_hydrogen_properties(): 
    """
    Load hydrogen property data from the RES file.

    Parameters
    ----------
    None

    Returns
    -------
    hydrogen_data : dict
        Raw hydrogen property data loaded from ``H2_properties.res``.

    Notes
    -----
    Assumes hydrogen behaves as an ideal gas for the stored properties.  

    Source
    ------
    Internal RCAIDE resource file: ``H2_properties.res``

    See Also
    --------
    RCAIDE.load : Function used to load RES files
    """
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator     

    return RCAIDE.load(rel_path+ 'H2_properties.res')