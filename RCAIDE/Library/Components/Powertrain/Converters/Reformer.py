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
    Reformer Component Class

    This class models an autothermal reformer that converts a hydrocarbon fuel (working_fluid)
    into hydrogen-rich reformate gas. It inherits from the base Converter class and implements
    reformer-specific attributes and methods.

    Attributes
    ----------
    tag : str
        Identifier for the reformer component, defaults to 'reformer'

    working_fluid : RCAIDE.Library.Attributes.Propellants.Propellant
        Hydrocarbon fuel being reformed. Its density, molecular_weight,
        hydrogen_mass_fraction, carbon_mass_fraction, lower_heating_value, and
        stoichiometric_fuel_air_ratio are read directly from this object, so any
        propellant with those properties defined (e.g. Jet_A, Jet_A1,
        Liquid_Natural_Gas, Liquid_Petroleum_Gas) can be reformed. Defaults to None
        and must be set explicitly before use.
    reformate_hydrogen_mole_fraction : float
        Mole fraction of hydrogen content in reformate [mol]
    reformate_carbon_monoxide_mole_fraction : float
        Mole fraction of carbon monoxide content in reformate [mol]
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
    carbon_monoxide_lower_heating_value : float
        Lower heating value of Carbon Monoxide [kJ/g-mol]
    catalyst_bed_volume : float
        Catalyst bed volume [cm^3]
    eta : float
        Reformer efficiency [-]
    design_steam_to_fuel_volumetric_ratio : float
        Design steam feed rate to fuel feed rate volumetric ratio [-]
    design_air_to_fuel_volumetric_ratio : float
        Design air feed rate to fuel feed rate volumetric ratio [-]

    Notes
    -----
    The reformer model includes parameters for:
        * Fuel composition and properties
        * Reformate composition
        * Reformer geometry and performance characteristics
        * Thermodynamic properties of reactants/products

    A real autothermal reformer meters its steam and air feed proportionally to
    the fuel feed rate to hold a fixed operating point (design_steam_to_fuel_volumetric_ratio,
    design_air_to_fuel_volumetric_ratio), so the fuel feed rate is the one free
    variable a solve (e.g. Reformer_Fuel_Cell, working backward from a target
    hydrogen production rate) needs to determine.

    """

    def __defaults__(self): 
        self.tag                                      = 'reformer'
        self.working_fluid                            = None
        self.reformate_hydrogen_mole_fraction         = 0.9      # [mol] mole fraction of hydrogen content in reformate
        self.reformate_carbon_monoxide_mole_fraction  = 0.3      # [mol] mole fraction of carbon monoxide content in reformate
        self.steam_density                            = 1        # [g/cm**3]  Density of water
        self.air_density                              = 0.001293 # [g/cm**3]  Density of air
        self.steam_molecular_weight                   = 18.01    # [g/g-mol]  Average molecular weight of steam
        self.carbon_molecular_weight                  = 12.01    # [g/g-mol]  Average molecular weight of carbon
        self.hydrogen_molecular_weight                = 2.016    # [g/g-mol]  Average molecular weight of hydrogen
        self.contact_time                             = 0.074    # [sec**-1]  Catalyst contact time
        self.hydrogen_lower_heating_value             = 240.2    # [kJ/g-mol] Lower heating value of Hydrogen
        self.carbon_monoxide_lower_heating_value      = 283.1    # [kJ/g-mol] Lower heating value of Carbon Monoxide
        self.catalyst_bed_volume                      = 9.653    # [cm**3]    Catalyst bed volume
        self.design_steam_to_fuel_volumetric_ratio    = 3.7037  # [-] design Q_S / Q_F
        self.design_air_to_fuel_volumetric_ratio      = 2222.2  # [-] design Q_A / Q_F

    def append_operating_conditions(self, segment):
        """Attach reformer operating conditions to the segment's energy conditions."""
        append_reformer_conditions(self, segment)
        return

    def compute_performance(self,state,network=None):
        reformer_conditions = state.conditions.energy.converters[self.tag]
        inputs, outputs, stored_results_flag, stored_converter_tag = compute_reformer_performance(self, reformer_conditions, state)
        return inputs, outputs, stored_results_flag, stored_converter_tag