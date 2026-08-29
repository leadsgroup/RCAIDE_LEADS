# RCAIDE/Library/Attributes/Liquid_Hydrogen.py
# 
# 
# Created:  Sep 2023, M. Clarke
# Modified: 
 
# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from .Propellant import Propellant 

import os
import numpy as np
from functools          import lru_cache
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
        self.cryogenic                     = True
        self.reactant                      = 'O2'
        self.density                       = 70.85                            # [kg/m^3]
        self.specific_energy               = 120e6  # [J/kg] Considering the lower heating value https://ntrs.nasa.gov/api/citations/20020085127/downloads/20020085127.pdf
        self.lower_heating_value           = 120e6                              # J/kg
        self.kinematic_viscosity           = 1.9e-7                           # [m^2/s] liquid hydrogen near its boiling point (~20 K)
        self.energy_density                = 8491.0e6                         # [J/m^3]
        self.stoichiometric_fuel_to_air    = 0.029411
        self.molecular_weight              = 2.016                           # [g/mol]
        self.temperatures.autoignition     = 845.15                           # [K]
        self.stoichiometric_fuel_air_ratio = 0.029411         # [-] Stoichiometric Fuel to Air ratio
        self.heat_of_vaporization          = 0         # [J/kg] Heat of vaporization at standard conditions
        self.temperature                   = 20         # [K] Temperature of fuel
        self.pressure                      = 0         # [Pa] Pressure of fuel
        self.fuel_surrogate_S1             = {} # [-] Mole fractions of fuel surrogate species
        self.kinetic_mechanism             = '' # [-] Kinetic mechanism for fuel surrogate species
        self.oxidizer                      = ''       

        self.emission_indices.Production  = 0.0      # kg/kg 
        self.emission_indices.CO2         = 0.0      # kg/kg
        self.emission_indices.CO          = 0.0      # kg/kg
        self.emission_indices.H2O         = 8.21     # kg/kg  
        self.emission_indices.SO2         = 0.0      # kg/kg
        self.emission_indices.NOx         = 0.0539   # kg/kg
        self.emission_indices.Soot        = 0.0      # kg/kg
        
        self.global_warming_potential_100.CO2       = 1     # CO2e/kg  
        self.global_warming_potential_100.H2O       = 0.06  # CO2e/kg  
        self.global_warming_potential_100.CO        = 1     # CO2e/kg  
        self.global_warming_potential_100.SO2       = -226  # CO2e/kg  
        self.global_warming_potential_100.NOx       = 52    # CO2e/kg  
        self.global_warming_potential_100.CO        = 1     # CO2e/kg  
        self.global_warming_potential_100.Soot      = 1166  # CO2e/kg    
        self.global_warming_potential_100.Contrails = 11 #  kg/CO2e/km
        
        self.materials_properties = self.cryogen_properties()

    def cryogen_properties(self, T, prop_name, phase='liquid'):
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
            phase : str
                Saturation branch to interpolate along, 'liquid' or 'vapor' (default 'liquid').

            Returns
            -------
            prop_value : float or ndarray
                Interpolated property value(s) corresponding to the input temperature(s).

            Notes
            -----
            * Property data is loaded from ``H2_properties.res`` using
            :func:`load_hydrogen_properties`.
            * The table holds both saturated-liquid and saturated-vapor rows at the same
              temperatures, so it is filtered to ``phase`` before interpolating.
            * Linear interpolation is applied between tabulated values.
            * Extrapolation outside the data range is not supported (``fill_value=None``).

            See Also
            --------
            RCAIDE.Library.Attributes.Propellants.Liquid_Hydrogen.load_hydrogen_properties
         """
        data = load_hydrogen_properties()
        phase_mask = np.array(data["Phase"]) == phase
        temps = np.array(data["Temperature (K)"], dtype=float)[phase_mask]
        props = np.array(data[prop_name], dtype=float)[phase_mask]
        interp = interp1d(temps, props, kind="linear", fill_value=None)

        return interp(T)

    def saturation_temperature(self, P):
        """
            Invert the saturated-liquid branch of the property table to return the
            saturation temperature at a given pressure.

            Parameters
            ----------
            P : float or ndarray
                Pressure(s) in MPa at which the saturation temperature is requested.

            Returns
            -------
            T_sat : float or ndarray
                Saturation temperature(s) [K].
        """
        data = load_hydrogen_properties()
        phase_mask = np.array(data["Phase"]) == 'liquid'
        temps = np.array(data["Temperature (K)"], dtype=float)[phase_mask]
        pressures = np.array(data["Pressure (MPa)"], dtype=float)[phase_mask]
        interp = interp1d(pressures, temps, kind="linear", fill_value=None)

        return interp(P)

    def property_table_range(self, phase='liquid'):
        """
            Valid (min, max) temperature [K] and pressure [MPa] range of the
            saturated-property table, for clamping solver iterates before they hit
            the table's hard edges (the table does not extrapolate).
        """
        data = load_hydrogen_properties()
        phase_mask = np.array(data["Phase"]) == phase
        temps = np.array(data["Temperature (K)"], dtype=float)[phase_mask]
        pressures = np.array(data["Pressure (MPa)"], dtype=float)[phase_mask]
        return (temps.min(), temps.max()), (pressures.min(), pressures.max())

    def compressibility_factor(self, T, phase='vapor'):
        """
            Real-gas compressibility factor Z = P/(rho*R*T), evaluated along the
            saturation dome from the table's own (real) saturated density and
            pressure at temperature T. An ideal-gas EOS (Z=1) underpredicts
            pressure substantially near saturation for hydrogen; multiplying an
            ideal-gas pressure estimate by this Z corrects for that without a new
            fluid-property dependency.

            Parameters
            ----------
            T : float or ndarray
                Temperature(s) in Kelvin.
            phase : str
                Saturation branch to evaluate on (default 'vapor', the ullage).

            Returns
            -------
            Z : float or ndarray
                Compressibility factor (dimensionless).
        """
        R_specific = 8314.462618 / self.molecular_weight  # J/(kg*K)
        P_sat   = self.cryogen_properties(T, "Pressure (MPa)", phase=phase) * 1e6  # Pa
        rho_sat = self.cryogen_properties(T, "Density (kg/m3)", phase=phase)
        return P_sat / (rho_sat * R_specific * T)

@lru_cache(maxsize=1)
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