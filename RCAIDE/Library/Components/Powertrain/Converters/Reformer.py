# RCAIDE/Components/Powertrain/Converters/Reformer.py
# 
# 
# Created:  Jan 2025, M. Clarke, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ---------------------------------------------------------------------------------------------------------------------- 
# RCAIDE imports
import RCAIDE
from RCAIDE.Framework.Core              import Data
from RCAIDE.Library.Methods.Powertrain.Converters.Reformer.append_reformer_conditions import append_reformer_conditions
from RCAIDE.Library.Methods.Powertrain.Converters.Reformer.compute_reformer_performance import compute_reformer_performance
from .Converter  import Converter
import numpy as np
import scipy as sp

# ---------------------------------------------------------------------------------------------------------------------- 
#  Reformer
# ----------------------------------------------------------------------------------------------------------------------  
class Reformer(Converter):
    """
    Autothermal reformer that converts a hydrocarbon fuel (working_fluid) into hydrogen-rich reformate.

    Attributes
    ----------
    tag : str
        Identifier for the reformer component, defaults to 'reformer'

    working_fluid : RCAIDE.Library.Attributes.Propellants.Propellant
        Hydrocarbon fuel being reformed, for example Jet_A, Liquid_Natural_Gas or Liquid_Petroleum_Gas. It must
        define density, molecular_weight, hydrogen_mass_fraction, carbon_mass_fraction, lower_heating_value
        and stoichiometric_fuel_air_ratio. Must be set before use.
    efficiency : float
        Thermal reformer efficiency, LHV of hydrogen produced over LHV of fuel fed [-].
        Default 0.78, methane autothermal reforming [1]; the DOE fuel processor targets are 0.69 (2005) to
        0.80 (2010) [3]
    steam_density : float
        Density of water [g/cm^3]
    air_density : float
        Density of air [g/cm^3]
    steam_molecular_weight : float
        Average molecular weight of steam [g/g-mol]
    carbon_molecular_weight : float
        Average molecular weight of carbon [g/g-mol]
    hydrogen_molecular_weight : float
        Average molecular weight of hydrogen [g/g-mol]
    contact_time : float
        Catalyst contact time [sec^-1]
    hydrogen_lower_heating_value : float
        Lower heating value of Hydrogen [kJ/g-mol]
    catalyst_bed_volume : float
        Catalyst bed volume [cm^3]
    design_steam_to_fuel_volumetric_ratio : float
        Design steam feed rate to fuel feed rate volumetric ratio [-]
    design_air_to_fuel_volumetric_ratio : float
        Design air feed rate to fuel feed rate volumetric ratio [-]
    design_power : float
        Electrical output of the fuel cell the reformer feeds [W] (set by Reformer_Fuel_Cell)
    specific_power : float
        design_power per unit reformer mass [W/kg]. Default 1/0.0343 kg/kW, the
        autothermal jet fuel reformer of [2] (Table 8); excludes the balance of plant
        (combustor, heat exchangers, turbomachinery, power electronics)

    Notes
    -----
    Steam and air are metered in proportion to the fuel feed (design_steam_to_fuel_volumetric_ratio and
    design_air_to_fuel_volumetric_ratio), so the fuel feed rate is the only free variable.

    References
    ----------
    [1] Halabi, M. H., de Croon, M. H. J. M., van der Schaaf, J., Cobden, P. D., and Schouten, J. C.,
        "Modeling and analysis of autothermal reforming of methane to hydrogen in a fixed bed reformer,"
        Chemical Engineering Journal, Vol. 137, No. 3, 2008, pp. 568-578, doi:10.1016/j.cej.2007.05.019.
    [2] Mak, A., and Meier, J., "Fuel Cell Auxiliary Power Study, Volume 1: RASER Task Order 5,"
        NASA/CR-2007-214461/VOL1, Honeywell Engines, Systems & Services, 2007.
    [3] U.S. Department of Energy, "Hydrogen, Fuel Cells & Infrastructure Technologies Program
        Multi-Year Research, Development and Demonstration Plan," 2007, Sec. 3.4 (fuel processor
        efficiency targets 69% in 2005 and 80% in 2010, LHV basis).
    """

    def __defaults__(self): 
        self.tag                                      = 'reformer'
        self.working_fluid                            = None
        self.efficiency                               = 0.78
        self.steam_density                            = 1        # [g/cm**3]  Density of water
        self.air_density                              = 0.001293 # [g/cm**3]  Density of air
        self.steam_molecular_weight                   = 18.01    # [g/g-mol]  Average molecular weight of steam
        self.carbon_molecular_weight                  = 12.01    # [g/g-mol]  Average molecular weight of carbon
        self.hydrogen_molecular_weight                = 2.016    # [g/g-mol]  Average molecular weight of hydrogen
        self.contact_time                             = 0.074    # [sec**-1]  Catalyst contact time
        self.hydrogen_lower_heating_value             = 240.2    # [kJ/g-mol] Lower heating value of Hydrogen
        self.catalyst_bed_volume                      = 9.653    # [cm**3]    Catalyst bed volume
        self.design_steam_to_fuel_volumetric_ratio    = 3.7037  # [-] design Q_S / Q_F
        self.design_air_to_fuel_volumetric_ratio      = 2222.2  # [-] design Q_A / Q_F
        self.design_power                             = None
        self.specific_power                           = 1000. / 0.0343

    def append_operating_conditions(self, segment):
        """Attach reformer operating conditions to the segment's energy conditions."""
        append_reformer_conditions(self, segment)
        return

    def compute_performance(self,state,network=None):
        reformer_conditions = state.conditions.energy.converters[self.tag]
        inputs, outputs, stored_results_flag, stored_converter_tag = compute_reformer_performance(self, reformer_conditions, state)
        return inputs, outputs, stored_results_flag, stored_converter_tag