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
    [1] Penner, J.E. et al., "Aviation and the Global Atmosphere," IPCC Special Report,
        Cambridge University Press, 1999. Canonical source for aviation EI derivation
        from fuel H/C ratios; CO2 and H2O indices follow directly from stoichiometry.

    [2] Detsios, N. et al., "Liquefied Natural Gas for Civil Aviation,"
        Energies, 13(22), 5925, 2020. https://doi.org/10.3390/en13225925
        Source for LNG combustion emission indices including H2O, SO2, NOx, and soot.

    [3] Timko, M.T. et al., "Gas Turbine Engine Emissions — Part I: Volatile Organic
        Compounds and Nitrogen Oxides," Energy & Fuels, 24, 5883–5896, 2010.
        https://doi.org/10.1021/ef100846d
        CO emission index ~0.5–2.0 g/kg at cruise; 1.0 g/kg used as mid-range.

    [4] International Energy Agency (IEA), "Assessing Emissions from LNG Supply and
        Abatement Options," IEA, 2022.
        https://www.iea.org/reports/assessing-emissions-from-lng-supply-and-abatement-options
        Upstream supply-chain CO2 for LNG ≈ 0.35 kg CO2/kg LNG.

    [5] Wang, M. et al., "GREET Life-Cycle Model," Argonne National Laboratory,
        ANL/ESD/12-3, 2022. https://publications.anl.gov/anlpubs/2022/10/179090.pdf
        Well-to-gate upstream CO2 consistent with Production index of 0.35 kg/kg.

    [6] Lower and Higher Heating Values of Gas, Liquid, and Solid Fuels.
        NPRE 470 course resource, University of Illinois Urbana-Champaign, 2018.
        https://courses.grainger.illinois.edu/npre470/sp2018/web/
        Lower_and_Higher_Heating_Values_of_Gas_Liquid_and_Solid_Fuels.pdf
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
        LHV (specific_energy):
            NPRE 470, University of Illinois Urbana-Champaign (2018) [6].

        Emission indices:
            CO2, H2O — stoichiometric (CH4 + 2O2 → CO2 + 2H2O); Penner et al. IPCC 1999 [1].
            H2O value of 2.20 accounts for LNG blend (vs 2.25 for pure CH4); Detsios et al. 2020 [2].
            CO  — mid-range cruise value 1.0 g/kg; Timko et al. Energy & Fuels 2010 [3].
            SO2 — zero: LNG contains negligible sulfur; Detsios et al. 2020 [2].
            NOx — 12.6 g/kg; note Detsios et al. 2020 [2] reports ~60% NOx reduction
                  vs Jet-A for LNG combustion (implying ~5–6 g/kg); value may need revision.
            Soot — zero: methane has no aromatic content; Detsios et al. 2020 [2].
            Production — 0.35 kg CO2/kg LNG upstream; IEA 2022 [4] and GREET ANL [5].
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

        self.emission_indices.Production  = 0.35     # kg/kg  well-to-gate upstream CO2; IEA 2022 [4], GREET ANL [5]
        self.emission_indices.CO2         = 2.75     # kg/kg  stoichiometric: CH4 + 2O2 → CO2 + 2H2O → 44/16; Penner et al. 1999 [1]
        self.emission_indices.CO          = 0.00100  # kg/kg  1.0 g/kg mid-range cruise; Timko et al. 2010 [3]
        self.emission_indices.H2O         = 2.20     # kg/kg  LNG blend (pure CH4 gives 2.25); Detsios et al. 2020 [2]
        self.emission_indices.SO2         = 0.0      # kg/kg  LNG contains negligible sulfur; Detsios et al. 2020 [2]
        self.emission_indices.NOx         = 0.0126   # kg/kg  12.6 g/kg; note: LNG typically 40–60% lower than Jet-A [2]
        self.emission_indices.Soot        = 0.0      # kg/kg  methane has no aromatic content; Detsios et al. 2020 [2]
        
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