# RCAIDE/Library/Methods/Powertrain/Converters/Reformer/compute_reformer_performance.py
#
# Created:  Jan 2025, M. Clarke, M. Guidotti

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
# package imports
from RCAIDE.Framework.Core import Units

# ----------------------------------------------------------------------------------------------------------------------
#  compute_reformer_performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_reformer_performance(reformer,reformer_conditions, state):
    """
    Computes performance characteristics of an autothermal reformer converting a
    hydrocarbon fuel (reformer.working_fluid) to hydrogen-rich reformate.

    Parameters
    ----------
    reformer : Reformer
        Reformer component containing physical and operational parameters, including
        working_fluid (the fuel being reformed -- any Propellant with density,
        molecular_weight, hydrogen_mass_fraction, carbon_mass_fraction,
        lower_heating_value, and stoichiometric_fuel_air_ratio defined, e.g. Jet_A,
        Jet_A1, Liquid_Natural_Gas, Liquid_Petroleum_Gas)
    reformer_conditions : Conditions
        Container for reformer operating conditions including feed rates
        (fuel_volume_flow_rate, steam_volume_flow_rate, air_volume_flow_rate)

    Returns
    -------
    inputs : Conditions
        Reformer input power (chemical power drawn in as fuel)
    outputs : Conditions
        Reformer output power (chemical power produced as hydrogen-rich reformate)
    stored_results_flag : bool
        Flag indicating if results are stored
    stored_converter_tag : str
        Tag of the reformer with stored results

    Notes
    -----
    In addition to updating reformer_conditions in-place with performance parameters
    (effluent_gas_flow_rate, reformer_efficiency, hydrogen_conversion_efficiency,
    space_velocity, liquid_space_velocity, steam_to_carbon_feed_ratio,
    oxygen_to_carbon_feed_ratio, fuel_to_air_ratio, hydrogen_mass_flow_rate), this
    function calculates key performance metrics for an autothermal reformer including:
        - Molar flow rates of reactants and products
        - Space velocities
        - Feed ratios
        - Conversion efficiencies

    reformer_conditions.inputs.power.chemical and .outputs.power.chemical are set
    from the fuel feed's chemical power and the reformate's chemical power
    respectively; they differ by the reformer's conversion losses, so the net
    contribution to the network's power balance reflects reformer_efficiency
    rather than being reported as zero.

    LHV_fuel is formed from working_fluid.lower_heating_value (mass-basis,
    J/kg, per the RCAIDE.Library.Attributes.Propellants.Propellant convention)
    scaled to MJ/kg, and is combined directly with the fuel's molar flow rate
    (F_F) in the efficiency and input-power formulas below -- this specific
    combination (mass-basis heating value against a molar flow rate) is the
    convention this reformer model was originally fit against, not a
    dimensionally pure molar heating value. LHV_H2 and LHV_CO, by contrast,
    are genuinely molar (kJ/g-mol) and are combined with the molar reformate
    flow rate (F_R). Chemical powers are converted from kJ/hr to W
    (kJ/hr / 3.6 = W).

    hydrogen_mass_flow_rate is the mass flow rate of hydrogen leaving in the
    reformate stream -- this is what a downstream fuel cell (e.g. within a
    Reformer_Fuel_Cell composite) consumes as its own fuel input.

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
    LHV_F  = fuel.lower_heating_value / 1e6           # [J/kg] -> numeric convention this model was fit against (see Notes)
    A_F_st = 1. / fuel.stoichiometric_fuel_air_ratio  # [lb_Air/lb_fuel] stoichiometric air-to-fuel mass ratio
    y_H2   = reformer.reformate_hydrogen_mole_fraction         # [mol] mole fraction of hydrogen content in reformate
    y_CO   = reformer.reformate_carbon_monoxide_mole_fraction  # [mol] mole fraction of carbon monoxide content in reformate
    rho_S  = reformer.steam_density              # [g/cm**3]
    rho_A  = reformer.air_density                # [g/cm**3]
    MW_S   = reformer.steam_molecular_weight      # [g/g-mol]
    MW_C   = reformer.carbon_molecular_weight     # [g/g-mol]
    MW_H2  = reformer.hydrogen_molecular_weight   # [g/g-mol]
    LHV_H2 = reformer.hydrogen_lower_heating_value             # [kJ/g-mol]
    LHV_CO = reformer.carbon_monoxide_lower_heating_value      # [kJ/g-mol]

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

    # Reformer efficiency
    eta_ref = ((y_H2 * LHV_H2 + y_CO * LHV_CO) * F_R / (Q_F * rho_F * LHV_F)) * 100 # [-] Reformer efficiency

    # Hydrogen conversion efficiency
    X_H2 = ((y_H2 * F_R)/ (((Q_F * rho_F * x_H)/MW_H2) + F_S)) * 100 # [-] Hydrogen conversion efficiency

    # Hydrogen mass flow rate produced in the reformate stream
    H2_molar_flow_rate = y_H2 * F_R                                # [g-mol/hr]
    hydrogen_mass_flow_rate = (H2_molar_flow_rate * MW_H2) / 3.6e6 # [g/hr] -> [kg/s]

    # Chemical power drawn in as fuel and produced as hydrogen-rich reformate (see Notes).
    # input_chemical_power uses the fuel's mass flow rate (Q_F * rho_F), matching the
    # same LHV_F pairing eta_ref uses -- not F_F (molar flow rate), which would silently
    # scale this to the wrong magnitude despite LHV_F/rho_F/MW_F all having sane values.
    input_chemical_power  = (Q_F * rho_F * LHV_F) / 3.6              # [W]
    output_chemical_power = ((y_H2 * LHV_H2 + y_CO * LHV_CO) * F_R) / 3.6 # [W]

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
