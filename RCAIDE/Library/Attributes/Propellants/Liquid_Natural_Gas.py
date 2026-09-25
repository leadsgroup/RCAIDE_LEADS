# RCAIDE/Library/Attributes/Propellants/Liquid_Natural_Gas.py
# 
#
# Created:  April 2026, S. Shekar, S. Sharma

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from .Propellant import Propellant
from RCAIDE.Framework.Core.Physical_Constants import UNIVERSAL_GAS_CONSTANT
from RCAIDE.Framework.Core import Units

import os
import numpy as np
from functools          import lru_cache
from scipy.interpolate  import interp1d
# ----------------------------------------------------------------------------------------------------------------------
#  Liquid_Natural_Gas Class
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
        Lower heating value (LHV) specific energy content in J/kg (48.632e6)
    energy_density : float
        Energy density in J/m³ (22200.0e6)
    molecular_weight : float
        Molecular weight in g/mol (16.04, pure methane approximation)
    hydrogen_mass_fraction : float
        Mass fraction of hydrogen content (0.251)
    carbon_mass_fraction : float
        Mass fraction of carbon content (0.749)
    stoichiometric_fuel_air_ratio : float
        Stoichiometric fuel-to-air ratio [-] (1/17.2, methane combustion)
    heat_of_vaporization : float
        Heat of vaporization at standard conditions in J/kg (0, placeholder)
    temperature : float
        Temperature of fuel in K (0, placeholder)
    pressure : float
        Pressure of fuel in Pa (0, placeholder)
    fuel_surrogate_S1 : dict
        Mole fractions of fuel surrogate species [-]
    kinetic_mechanism : str
        Kinetic mechanism name for fuel surrogate combustion
    oxidizer : str
        Oxidizer species name
    emission_indices : Data
        Emission indices in kg/kg fuel
            - Production : float
                Upstream extraction and liquefaction CO2 (0.35)
            - CO2 : float
                Carbon dioxide (2.75)
            - CO : float
                Carbon monoxide (0.00100)
            - H2O : float
                Water vapor (2.20)
            - SO2 : float
                Sulfur dioxide (0.0)
            - NOx : float
                Nitrogen oxides (0.0126)
            - Soot : float
                Particulate matter (0.0)
    global_warming_potential_100 : Data
        100-year global warming potentials (CO2-equivalent per kg of species)
            - CO2 : float
                Carbon dioxide (1)
            - H2O : float
                Water vapor (0.06)
            - CO : float
                Carbon monoxide (1)
            - SO2 : float
                Sulfur dioxide (-226)
            - NOx : float
                Nitrogen oxides (52)
            - Soot : float
                Particulate matter (1166)
            - Contrails : float
                Contrail formation factor in kg CO2e/km (11)
    materials_properties : dict
        Interpolated thermophysical property data loaded from ``LNG_properties.res``

    Notes
    -----
    LNG is a cryogenic fuel consisting primarily of methane with small amounts of
    heavier hydrocarbons. It requires storage at approximately -162°C (-111 K) but
    offers reduced carbon emissions compared to conventional jet fuels.

    The ``specific_energy`` value uses the lower heating value (LHV) of LNG, which
    excludes the latent heat of water condensation in combustion products.

    **Definitions**

    'Global Warming Potential (GWP-100)'
        Relative measure of heat trapped in the atmosphere over 100 years compared
        to an equivalent mass of CO2

    'Energy Density'
        Energy content per unit volume, dependent on cryogenic storage conditions

    **Major Assumptions**
        * Properties are for saturated liquid at atmospheric pressure
        * Composition represents a typical LNG mixture

    References
    ----------
    [1] Lower and Higher Heating Values of Gas, Liquid, and Solid Fuels.
        NPRE 470 course resource, University of Illinois Urbana-Champaign, 2018.
        https://courses.grainger.illinois.edu/npre470/sp2018/web/Lower_and_Higher_Heating_Values_of_Gas_Liquid_and_Solid_Fuels.pdf
    """

    def __defaults__(self):
        """Set default values for Liquid Natural Gas propellant properties.

        Assumptions
        -----------
        * Density and energy properties correspond to saturated liquid LNG
          at atmospheric pressure.
        * Specific energy is based on the lower heating value (LHV) of LNG.
        * GWP-100 values follow standard aviation emission indices.
        * Placeholder values of 0 are assigned to fields that must be set
          by the user or a higher-fidelity model before use.

        Source
        ------
        Lower Heating Value:
            NPRE 470, University of Illinois Urbana-Champaign (2018).
            https://courses.grainger.illinois.edu/npre470/sp2018/web/
            Lower_and_Higher_Heating_Values_of_Gas_Liquid_and_Solid_Fuels.pdf
        """
        self.tag             = 'Liquid_Natural_Gas'
        self.cryogenic       = True
        self.reactant        = 'O2'
        self.density         = 414.2       # [kg/m^3]  saturated liquid at ~111 K
        self.specific_energy = 48.632e6    # [J/kg]    lower heating value (LHV) 
        self.energy_density  = 22200.0e6   # [J/m^3]
        self.gravimetric_efficiency = 0.7
        self.lower_heating_value    = 45e6
        self.molecular_weight       = 16.04   # [g/mol] pure methane (CH4) approximation
        self.hydrogen_mass_fraction = 0.251   # [-]    mass fraction of hydrogen content (CH4)
        self.carbon_mass_fraction   = 0.749   # [-]    mass fraction of carbon content (CH4)
        self.kinematic_viscosity    = 1.9e-7  # [m^2/s] kinematic viscosity of liquid methane near its normal boiling point (~111 K)

        self.stoichiometric_fuel_air_ratio = 1/17.2   # [-]    stoichiometric fuel-to-air ratio for methane combustion
        self.heat_of_vaporization          = 0   # [J/kg] heat of vaporization at standard conditions (placeholder)
        self.temperature                   = 0   # [K]    fuel temperature (placeholder)
        self.pressure                      = 0   # [Pa]   fuel pressure (placeholder)
        self.fuel_surrogate_S1             = {}  # [-]    mole fractions of fuel surrogate species
        self.kinetic_mechanism             = ''  #        kinetic mechanism name for combustion model
        self.oxidizer                      = ''  #        oxidizer species name

        self.emission_indices.Production  = 0.35     # kg/kg  upstream extraction + liquefaction (GREET)
        self.emission_indices.CO2         = 2.75     # kg/kg  stoichiometric floor for CH4 (44/16); LNG blend ≈ 2.75–2.78
        self.emission_indices.CO          = 0.00100  # kg/kg  ~1.0 g/kg, typical gas turbine at cruise
        self.emission_indices.H2O         = 2.20     # kg/kg
        self.emission_indices.SO2         = 0.0      # kg/kg
        self.emission_indices.NOx         = 0.0126   # kg/kg
        self.emission_indices.Soot        = 0.0      # kg/kg
        
        self.global_warming_potential_100.CO2       = 1     # [CO2e/kg]    carbon dioxide
        self.global_warming_potential_100.H2O       = 0.06  # [CO2e/kg]    water vapor
        self.global_warming_potential_100.CO        = 1     # [CO2e/kg]    carbon monoxide
        self.global_warming_potential_100.SO2       = -226  # [CO2e/kg]    sulfur dioxide (cooling effect)
        self.global_warming_potential_100.NOx       = 52    # [CO2e/kg]    nitrogen oxides
        self.global_warming_potential_100.Soot      = 1166  # [CO2e/kg]    soot / black carbon
        self.global_warming_potential_100.Contrails = 11    # [CO2e/km]    contrail radiative forcing

        self.materials_properties = self.cryogen_properties()

    def cryogen_properties(self, T, prop_name, phase='liquid'):
        """
        Return an interpolated LNG thermophysical property value at a given temperature.

        Parameters
        ----------
        T : float or ndarray
            Temperature(s) in Kelvin at which the property is requested.
        prop_name : str
            Name of the property column to retrieve from the LNG data file.
            Valid keys include:

                - ``"Temperature (K)"``
                - ``"Pressure (MPa)"``
                - ``"Density (kg/m3)"``
                - ``"Volume (m3/kg)"``
                - ``"Internal Energy (kJ/kg)"``
                - ``"Enthalpy (kJ/kg)"``
                - ``"Entropy (J/g*K)"``
                - ``"Cv (J/g*K)"``
                - ``"Cp (J/g*K)"``
                - ``"Sound Spd. (m/s)"``
                - ``"Joule-Thomson (K/MPa)"``
                - ``"Viscosity (Pa*s)"``
                - ``"Therm. Cond. (W/m*K)"``
                - ``"Phase"``
        phase : str
            Saturation branch to interpolate along, 'liquid' or 'vapor' (default 'liquid').

        Returns
        -------
        prop_value : float or ndarray
            Interpolated property value(s) corresponding to the input temperature(s).

        Notes
        -----
        * Property data is loaded from ``LNG_properties.res`` via
          :func:`load_lng_properties`.
        * The table holds both saturated-liquid and saturated-vapor rows at the same
          temperatures, so it is filtered to ``phase`` before interpolating.
        * Linear interpolation is applied between tabulated data points.
        * Extrapolation outside the tabulated temperature range is not supported
          (``fill_value=None``).

        See Also
        --------
        load_lng_properties
        """
        return _property_interpolator(prop_name, phase)(T)

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
        return _saturation_temperature_interpolator()(P)

    def property_table_range(self, phase='liquid'):
        """
        Valid (min, max) temperature [K] and pressure [MPa] range of the
        saturated-property table, for clamping solver iterates before they hit
        the table's hard edges (the table does not extrapolate).
        """
        data = load_lng_properties()
        phase_mask = np.array(data["Phase"]) == phase
        temps = np.array(data["Temperature (K)"], dtype=float)[phase_mask]
        pressures = np.array(data["Pressure (MPa)"], dtype=float)[phase_mask]
        return (temps.min(), temps.max()), (pressures.min(), pressures.max())

    def compressibility_factor(self, T, phase='vapor'):
        """
        Real-gas compressibility factor Z = P/(rho*R*T), evaluated along the
        saturation dome from the table's own (real) saturated density and
        pressure at temperature T. An ideal-gas EOS (Z=1) underpredicts
        pressure substantially near saturation; multiplying an ideal-gas
        pressure estimate by this Z corrects for that without a new fluid-
        property dependency.

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
        R_specific = UNIVERSAL_GAS_CONSTANT / self.molecular_weight  # J/(kg*K)
        P_sat   = self.cryogen_properties(T, "Pressure (MPa)", phase=phase) * Units.MPa
        rho_sat = self.cryogen_properties(T, "Density (kg/m3)", phase=phase)
        return P_sat / (rho_sat * R_specific * T)

@lru_cache(maxsize=1)
def load_lng_properties():
    """
    Load liquid natural gas property data from the RES file.

    Parameters
    ----------
    None

    Returns
    -------
    lng_data : dict
        Raw liquid natural gas property data loaded from ``LNG_properties.res``.

    Notes
    -----
    Assumes liquid natural gas is pure Methane and behaves as an ideal fluid for the stored properties.  

    Source
    ------
    Internal RCAIDE resource file: ``LNG_properties.res``

    See Also
    --------
    RCAIDE.load : Function used to load RES files
    """
    ospath    = os.path.abspath(__file__)
    separator = os.path.sep
    rel_path  = os.path.dirname(ospath) + separator

    return RCAIDE.load(rel_path+ 'LNG_properties.res')

# Cached: cryogen_properties calls this many times per boil-off RHS evaluation.
@lru_cache(maxsize=None)
def _property_interpolator(prop_name, phase):
    data = load_lng_properties()
    phase_mask = np.array(data["Phase"]) == phase
    temps = np.array(data["Temperature (K)"], dtype=float)[phase_mask]
    props = np.array(data[prop_name], dtype=float)[phase_mask]
    return interp1d(temps, props, kind="linear", fill_value=None)

@lru_cache(maxsize=None)
def _saturation_temperature_interpolator():
    data = load_lng_properties()
    phase_mask = np.array(data["Phase"]) == 'liquid'
    temps = np.array(data["Temperature (K)"], dtype=float)[phase_mask]
    pressures = np.array(data["Pressure (MPa)"], dtype=float)[phase_mask]
    return interp1d(pressures, temps, kind="linear", fill_value=None)