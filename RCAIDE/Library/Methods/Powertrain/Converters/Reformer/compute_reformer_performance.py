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
def compute_reformer_performance(reformer,reformer_conditions):
    """
    Computes performance characteristics of an autothermal reformer converting jet fuel to hydrogen-rich reformate.

    Parameters
    ----------
    reformer : Reformer
        Reformer component containing physical and operational parameters
    reformer_conditions : Conditions
        Container for reformer operating conditions including feed rates

    Returns
    -------
    None
        Updates reformer_conditions in-place with computed performance parameters:
            - effluent_gas_flow_rate : float
                Reformer effluent gas flow rate [sccm]
            - reformer_efficiency : float
                Overall reformer efficiency [%]
            - hydrogen_conversion_efficiency : float
                Hydrogen conversion efficiency [%]
            - space_velocity : float
                Gas hourly space velocity [hr^-1]
            - liquid_space_velocity : float
                Liquid hourly space velocity [hr^-1]
            - steam_to_carbon_feed_ratio : float
                Molar ratio of steam to carbon [mol_H2O/mol_C]
            - oxygen_to_carbon_feed_ratio : float
                Molar ratio of oxygen to carbon [mol_O/mol_C]
            - fuel_to_air_ratio : float
                Equivalence ratio [-]

    Notes
    -----
    This function calculates key performance metrics for an autothermal reformer including:
        - Molar flow rates of reactants and products
        - Space velocities
        - Feed ratios
        - Conversion efficiencies

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
    """

    Q_F = reformer_conditions.fuel_volume_flow_rate/(Units.cm**3/Units.hr)   # [cm**3/hr] Jet-A feed rate              
    Q_S = reformer_conditions.steam_volume_flow_rate/(Units.cm**3/Units.hr)  # [cm**3/hr] Deionized water feed rate            
    Q_A = reformer_conditions.air_volume_flow_rate/(Units.cm**3/Units.min)   # [sccm]     Air feed rate

    # Molar Feed Rates
    F_F = Q_F * reformer.rho_F / reformer.MW_F                # [g-mol/hr] molar flow rate of Jet-A
    F_S = Q_S * reformer.rho_S / reformer.MW_S                # [g-mol/hr] molar flow rate of steam
    F_A = Q_A / 22414                                         # [g-mol/hr] molar flow rate of air
    F_C = Q_F * reformer.rho_F * reformer.x_C / reformer.MW_C # [g-mol/hr] molar flow rate of carbon

    # Effluent Gas Molar Flow Rate
    Q_R = (Q_F/60) + (Q_S/60)  + Q_A # [sccm] Reformer effluent gas feed rate
    F_R = Q_R * 60 / 22414           # [g-mol/hr] reformate effluent gas molar flow rate

    # Space Velocity
    GHSV = ((F_F + F_S + F_A) / reformer.V_cat) * 22410 # [hr**-1] gas hourly space velocity
    LHSV = Q_F / reformer.V_cat                         # [hr**-1] liquid hourly space velocity

    # Steam to Carbon, Oxygen to Carbon and Equivalence Ratio 
    S_C = F_S / F_C                                                                      # [mol_H20/mol_C] Steam-to-Carbon feed ratio
    O_C = 2 * 0.21 * F_A / F_C                                                           # [mol_O/mol_C] Oxygen-to-Carbon feed ratio
    phi = reformer.A_F_st_Jet_A * (Q_F * reformer.rho_F) / ((Q_A * 60) * reformer.rho_A) # [-] Fuel to Air ratio

    # Reformer efficiency
    eta_ref = ((reformer.y_H2 * reformer.LHV_H2 + reformer.y_CO * reformer.LHV_CO) * F_R / (Q_F * reformer.rho_F * reformer.LHV_F)) * 100 # [-] Reformer efficiency

    # Hydrogen conversion efficiency
    X_H2 = ((reformer.y_H2 * F_R)/ (((Q_F * reformer.rho_F * reformer.x_H)/(reformer.MW_H2)) + F_S)) * 100 # [-] Hydrogen conversion efficiency  

    reformer_conditions.effluent_gas_flow_rate         = Q_R
    reformer_conditions.reformer_efficiency            = eta_ref
    reformer_conditions.hydrogen_conversion_efficiency = X_H2
    reformer_conditions.space_velocity                 = GHSV
    reformer_conditions.liquid_space_velocity          = LHSV
    reformer_conditions.steam_to_carbon_feed_ratio     = S_C
    reformer_conditions.oxygen_to_carbon_feed_ratio    = O_C
    reformer_conditions.fuel_to_air_ratio              = phi

    return