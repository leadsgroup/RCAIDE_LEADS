# RCAIDE/Library/Attributes/Propellants/Liquid_Petroleum_Gas.py
# 
#
# Created:  Mar 2024, M. Clarke

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
import numpy as np
import os
import RCAIDE
from scipy.interpolate  import interp1d
from .Propellant import Propellant   

# ---------------------------------------------------------------------------------------------------------------------- 
#  Gaseous_Hydrogen Class
# ----------------------------------------------------------------------------------------------------------------------   
class Liquid_Petroleum_Gas(Propellant):
    """
    A class representing Liquid Petroleum Gas (LPG) fuel properties and composition 
    for propulsion applications. 

    Attributes
    ----------
    tag : str
        Identifier for the propellant ('Liquid_Petroleum_Gas')
    reactant : str
        Oxidizer used for combustion ('O2')
    density : float
        Fuel density in kg/m³ (509.26)
    specific_energy : float
        Specific energy content in J/kg (43.1e6)
    energy_density : float
        Energy density in J/m³ (21949.1e6)
    use_high_fidelity_kinetics_model : bool
        Flag for using detailed chemical kinetics (False)
    fuel_surrogate_chemical_properties : dict
        Simplified chemical composition for surrogate model
            - C3H8 : float
                Propane fraction (0.6)
            - C4H10 : float
                Butane fraction (0.4)
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
    LPG is stored as a liquid under moderate pressure at ambient temperature. The 
    composition can vary seasonally and by region, but typically consists of a 
    propane-butane mixture.

    **Definitions**
    
    'Surrogate Model'
        Simplified two-component representation using propane and butane
    
    'High-Fidelity Model'
        Detailed representation including aromatic and aliphatic hydrocarbons
    
    'Global Warming Potential'
        Relative measure of heat trapped in atmosphere compared to CO2

    **Major Assumptions**
        * Surrogate model captures main combustion characteristics
        * Air composition is standard atmospheric
    """

    def __defaults__(self):
        """This sets the default values. 
    
    Assumptions:
        None
    
    Source:
        None
        """    
        self.tag             = 'Liquid_Petroleum_Gas'
        self.reactant        = 'O2'
        self.density         = 509.26                           # kg/m^3 
        self.specific_energy = 43.1e6                           # J/kg
        self.energy_density  = 21949.1e6                        # J/m^3
        
        self.stoichiometric_fuel_air_ratio = 0         # [-] Stoichiometric Fuel to Air ratio
        self.heat_of_vaporization          = 0         # [J/kg] Heat of vaporization at standard conditions
        self.temperature                   = 0         # [K] Temperature of fuel
        self.pressure                      = 0         # [Pa] Pressure of fuel
        self.fuel_surrogate_S1             = {} # [-] Mole fractions of fuel surrogate species
        self.kinetic_mechanism             = '' # [-] Kinetic mechanism for fuel surrogate species
        self.oxidizer                      = ''  
        
        self.global_warming_potential_100.CO2       = 1     # CO2e/kg  
        self.global_warming_potential_100.H2O       = 0.06  # CO2e/kg  
        self.global_warming_potential_100.CO        = 1     # CO2e/kg  
        self.global_warming_potential_100.SO2       = -226  # CO2e/kg  
        self.global_warming_potential_100.NOx       = 52    # CO2e/kg  
        self.global_warming_potential_100.Soot      = 1166  # CO2e/kg    
        self.global_warming_potential_100.Contrails = 11    # kg/CO2e/km 

        self.materials_properties = self.propellant_properties()

    def propellant_properties(self, T, prop_name):
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
        data = load_lpg_properties()
        temps = np.array(data["Temperature (K)"], dtype=float)
        props = np.array(data[prop_name], dtype=float)
        interp = interp1d(temps, props, kind="linear", fill_value=None)
        
        return interp(T)

def load_lpg_properties(): 
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

    return RCAIDE.load(rel_path+ 'LPG_properties.res')         