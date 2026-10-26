# RCAIDE/Library/Methods/Powertrain/Converters/Reformer/compute_reformer_performance.py
#
# Created:  Jan 2025, M. Clarke, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# package imports
from RCAIDE.Framework.Core import Units
import numpy as np

# ----------------------------------------------------------------------------------------------------------------------
#  compute_reformer_performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_reformer_performance(reformer,reformer_conditions, state):
    """
    Computes the performance of an autothermal reformer converting a hydrocarbon fuel
    (reformer.working_fluid) into hydrogen-rich reformate.

    Parameters
    ----------
    reformer : Reformer
        Reformer component; its working_fluid is the fuel being reformed.
    reformer_conditions : Conditions
        Reformer operating conditions, including the fuel, steam and air volume flow rates.

    Returns
    -------
    inputs : Conditions
        Chemical power of the fuel fed to the reformer.
    outputs : Conditions
        Chemical power of the hydrogen in the reformate.
    stored_results_flag : bool
        Flag indicating that results are stored.
    stored_converter_tag : str
        Tag of the reformer with stored results.

    Notes
    -----
    The hydrogen produced is the fuel's chemical power (mass flow times lower heating value) times
    reformer.efficiency, divided by the molar heating value of hydrogen; 78% for methane autothermal
    reforming (Halabi et al., Chem. Eng. J. 137, 2008). Chemical powers are converted from kJ/hr to W.
    The input and output chemical powers differ by the conversion loss. reformer_conditions is also
    updated with the effluent flow, efficiencies, space velocities, feed ratios and
    hydrogen_mass_flow_rate, the hydrogen a downstream fuel cell consumes.

    **Major Assumptions**
        * Steady state operation
        * Complete mixing of reactants
        * Uniform catalyst bed temperature
        * No pressure drop across catalyst bed
        * Ideal gas behavior for air and reformate
        * Standard conditions (1 atm, 273.15 K) for gas flow rates

    See Also
    --------
    RCAIDE.Library.Components.Powertrain.Converters.Reformer
    RCAIDE.Library.Components.Powertrain.Converters.Reformer_Fuel_Cell
    """
    fuel   = reformer.working_fluid
    rho_F  = fuel.density / 1000.                    # [kg/m**3] -> [g/cm**3]
    MW_F   = fuel.molecular_weight                   # [g/g-mol]
    x_H    = fuel.hydrogen_mass_fraction              # [-]
    x_C    = fuel.carbon_mass_fraction                # [-]
    LHV_F  = fuel.lower_heating_value / 1e6           # [J/kg] -> [kJ/g]
    A_F_st = 1. / fuel.stoichiometric_fuel_air_ratio  # [lb_Air/lb_fuel] stoichiometric air-to-fuel mass ratio
    rho_S  = reformer.steam_density              # [g/cm**3]
    rho_A  = reformer.air_density                # [g/cm**3]
    MW_S   = reformer.steam_molecular_weight      # [g/g-mol]
    MW_C   = reformer.carbon_molecular_weight     # [g/g-mol]
    MW_H2  = reformer.hydrogen_molecular_weight   # [g/g-mol]
    LHV_H2 = reformer.hydrogen_lower_heating_value             # [kJ/g-mol]

    Q_F = reformer_conditions.fuel_volume_flow_rate/(Units.cm**3/Units.hr)   # [cm**3/hr] fuel feed rate
    Q_S = reformer_conditions.steam_volume_flow_rate/(Units.cm**3/Units.hr)  # [cm**3/hr] Deionized water feed rate
    Q_A = reformer_conditions.air_volume_flow_rate/(Units.cm**3/Units.min)   # [sccm]     Air feed rate

    # Molar Feed Rates
    F_F = Q_F * rho_F / MW_F  # [g-mol/hr] molar flow rate of fuel
    F_S = Q_S * rho_S / MW_S  # [g-mol/hr] molar flow rate of steam
    F_A = Q_A / 22414         # [g-mol/hr] molar flow rate of air
    F_C = Q_F * rho_F * x_C / MW_C # [g-mol/hr] molar flow rate of carbon

    # Effluent Gas Molar Flow Rate
    Q_R = (Q_F/60) + (Q_S/60)  + Q_A # [sccm] Reformer effluent gas feed rate
    F_R = Q_R * 60 / 22414           # [g-mol/hr] reformate effluent gas molar flow rate

    # Space Velocity
    GHSV = ((F_F + F_S + F_A) / reformer.catalyst_bed_volume) * 22410 # [hr**-1] gas hourly space velocity
    LHSV = Q_F / reformer.catalyst_bed_volume                         # [hr**-1] liquid hourly space velocity

    # Steam to Carbon, Oxygen to Carbon and Equivalence Ratio
    S_C = F_S / F_C                                    # [mol_H20/mol_C] Steam-to-Carbon feed ratio
    O_C = 2 * 0.21 * F_A / F_C                         # [mol_O/mol_C] Oxygen-to-Carbon feed ratio
    phi = A_F_st * (Q_F * rho_F) / ((Q_A * 60) * rho_A) # [-] Fuel to Air ratio

    # Chemical power in as fuel and out as hydrogen (see Notes)
    input_chemical_power  = (Q_F * rho_F * LHV_F) / 3.6                # [W]
    output_chemical_power = reformer.efficiency * input_chemical_power  # [W]
    eta_ref               = reformer.efficiency * 100 * np.ones_like(Q_F)  # [%] reformer efficiency

    # Hydrogen produced, and its share of the hydrogen fed in the fuel and steam
    H2_molar_flow_rate      = output_chemical_power * 3.6 / LHV_H2                     # [g-mol/hr]
    hydrogen_mass_flow_rate = (H2_molar_flow_rate * MW_H2) / 3.6e6                     # [g/hr] -> [kg/s]
    X_H2                    = (H2_molar_flow_rate / (((Q_F * rho_F * x_H)/MW_H2) + F_S)) * 100  # [%] hydrogen conversion efficiency

    reformer_conditions.effluent_gas_flow_rate         = Q_R
    reformer_conditions.reformer_efficiency            = eta_ref
    reformer_conditions.hydrogen_conversion_efficiency = X_H2
    reformer_conditions.space_velocity                 = GHSV
    reformer_conditions.liquid_space_velocity          = LHSV
    reformer_conditions.steam_to_carbon_feed_ratio     = S_C
    reformer_conditions.oxygen_to_carbon_feed_ratio    = O_C
    reformer_conditions.fuel_to_air_ratio              = phi
    reformer_conditions.hydrogen_mass_flow_rate        = hydrogen_mass_flow_rate

    stored_results_flag            = True
    stored_converter_tag           = reformer.tag

    reformer_conditions.inputs.power.propulsive        = 0.0 * state.ones_row(1)
    reformer_conditions.inputs.power.mechanical        = 0.0 * state.ones_row(1)
    reformer_conditions.inputs.power.electrical        = 0.0 * state.ones_row(1)
    reformer_conditions.inputs.power.chemical          = input_chemical_power
    reformer_conditions.inputs.power.pneumatic         = 0.0 * state.ones_row(1)
    reformer_conditions.inputs.power.hydraulic         = 0.0 * state.ones_row(1)
    reformer_conditions.inputs.power.thermal           = 0.0 * state.ones_row(1)

    reformer_conditions.outputs.power.propulsive       = 0.0 * state.ones_row(1)
    reformer_conditions.outputs.power.mechanical       = 0.0 * state.ones_row(1)
    reformer_conditions.outputs.power.electrical       = 0.0 * state.ones_row(1)
    reformer_conditions.outputs.power.chemical         = output_chemical_power
    reformer_conditions.outputs.power.pneumatic        = 0.0 * state.ones_row(1)
    reformer_conditions.outputs.power.hydraulic        = 0.0 * state.ones_row(1)
    reformer_conditions.outputs.power.thermal          = 0.0 * state.ones_row(1)

    return  reformer_conditions.inputs, reformer_conditions.outputs, stored_results_flag, stored_converter_tag
