# RCAIDE/Library/Attributes/Propellants/Liquid_Natural_Gas.py
# 
#
# Created:  April 2026, S. Shekar, S. Sharma

# ---------------------------------------------------------------------------------------------------------------------- 
#  Imports
# ---------------------------------------------------------------------------------------------------------------------- 
import RCAIDE
from .Propellant import Propellant   

import os
import numpy as np
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
    stoichiometric_fuel_air_ratio : float
        Stoichiometric fuel-to-air ratio [-] (0, placeholder)
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
        self.reactant        = 'O2'
        self.density         = 414.2       # [kg/m^3]  saturated liquid at ~111 K
        self.specific_energy = 48.632e6    # [J/kg]    lower heating value (LHV) 
        self.energy_density  = 22200.0e6   # [J/m^3]
        
        self.stoichiometric_fuel_air_ratio = 0   # [-]    stoichiometric fuel-to-air ratio (placeholder)
        self.heat_of_vaporization          = 0   # [J/kg] heat of vaporization at standard conditions (placeholder)
        self.temperature                   = 0   # [K]    fuel temperature (placeholder)
        self.pressure                      = 0   # [Pa]   fuel pressure (placeholder)
        self.fuel_surrogate_S1             = {}  # [-]    mole fractions of fuel surrogate species
        self.kinetic_mechanism             = ''  #        kinetic mechanism name for combustion model
        self.oxidizer                      = ''  #        oxidizer species name
        
        self.global_warming_potential_100.CO2       = 1     # [CO2e/kg]    carbon dioxide
        self.global_warming_potential_100.H2O       = 0.06  # [CO2e/kg]    water vapor
        self.global_warming_potential_100.CO        = 1     # [CO2e/kg]    carbon monoxide
        self.global_warming_potential_100.SO2       = -226  # [CO2e/kg]    sulfur dioxide (cooling effect)
        self.global_warming_potential_100.NOx       = 52    # [CO2e/kg]    nitrogen oxides
        self.global_warming_potential_100.Soot      = 1166  # [CO2e/kg]    soot / black carbon
        self.global_warming_potential_100.Contrails = 11    # [CO2e/km]    contrail radiative forcing

        self.materials_properties = self.cryogen_properties()

    def cryogen_properties(self, T, prop_name):
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

        Returns
        -------
        prop_value : float or ndarray
            Interpolated property value(s) corresponding to the input temperature(s).

        Notes
        -----
        * Property data is loaded from ``LNG_properties.res`` via
          :func:`load_lng_properties`.
        * Linear interpolation is applied between tabulated data points.
        * Extrapolation outside the tabulated temperature range is not supported
          (``fill_value=None``).
        * The ``"Phase"`` column is categorical and is not suitable for
          numerical interpolation.

        See Also
        --------
        load_lng_properties
        """
        data = load_lng_properties()
        temps = np.array(data["Temperature (K)"], dtype=float)
        props = np.array(data[prop_name], dtype=float)
        interp = interp1d(temps, props, kind="linear", fill_value=None)
        
        return interp(T)

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