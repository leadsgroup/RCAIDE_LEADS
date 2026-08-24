# RCAIDE/Library/Methods/Powertrain/Converters/Fuel_Cells/Proton_Exchange_Membrane/compute_fuel_cell_performance.py
#
# Created:  Jan 2025, M. Clarke
# Modified: Jun 2026, M. Clarke

# ----------------------------------------------------------------------------------------------------------------------
#  IMPORT
# ----------------------------------------------------------------------------------------------------------------------
import numpy as np
from scipy.optimize import minimize_scalar

# ----------------------------------------------------------------------------------------------------------------------
#  Compute Fuel Cell Performance
# ----------------------------------------------------------------------------------------------------------------------
def compute_fuel_cell_performance(fuel_cell_stack, state, network):
    """Computes the performance of a PEM fuel cell stack across all control points.

    Iteratively solves for the current density at each control point that matches
    the electrical power demand, then stores the converged electrochemical and
    thermodynamic results in the segment conditions.

    Parameters
    ----------
    fuel_cell_stack : RCAIDE.Library.Components.Powertrain.Converters.Fuel_Cell
        Fuel cell stack object containing cell-level properties (membrane area,
        catalyst loading, electrical configuration, etc.).
    state : RCAIDE.Framework.Mission.Common.State
        Mission segment state containing freestream conditions, numerics,
        and energy converter conditions.
    network : RCAIDE.Framework.Networks.Network
        Energy network to which this converter belongs.

    Returns
    -------
    inputs : Conditions
        Input power conditions for the fuel cell stack.
    outputs : Conditions
        Output power conditions (electrical power produced).
    stored_results_flag : bool
        Always True; indicates results are available for reuse by identical converters.
    stored_converter_tag : str
        Tag of this fuel cell stack, used to look up stored results.

    Notes
    -----
    The power-matching uses a fixed-point iteration:

    .. math::
        i^{(k+1)} = i^{(k)} + \\alpha \\left( P_{demand} - P_{net}(i^{(k)}) \\right)

    where :math:`\\alpha = 0.05` is the relaxation factor and :math:`P_{net}` is
    the net cell power from ``evaluate_PEM``.

    The power this stack must supply is its own electrical distributor's
    total demand, split between battery/fuel-cell sources by that
    distributor's psi and between multiple fuel-cell stacks on the same bus
    by ``power_split_ratio``. The distributor is looked up from
    ``fuel_cell_stack.assigned_distributors`` rather than assumed, since a
    fuel cell stack may share the vehicle with other electrically-isolated
    buses resolved to a different psi.

    **Major Assumptions**
        * Uniform temperature across all cells in the stack
        * Hydrogen supplied at constant rated pressure
        * Standard atmospheric air composition (23.3 %  O2 by mass)
        * Ideal water management (no flooding or dehydration)

    References
    ----------
    [1] O'Hayre, R., Cha, S.-W., Colella, W., & Prinz, F. B. (2016).
        *Fuel Cell Fundamentals* (3rd ed.). John Wiley & Sons.
    """
    # ---------------------------------------------------------------------------------
    # Unpack freestream conditions
    # ---------------------------------------------------------------------------------
    M0 = state.conditions.freestream.mach_number
    P0 = state.conditions.freestream.pressure
    T0 = state.conditions.freestream.temperature
    a  = state.conditions.freestream.speed_of_sound
    R  = 287  # specific gas constant for air [J/(kg·K)]

    # Ratio of specific heats from speed of sound
    gamma = (a ** 2) / (T0 * R)

    # Isentropic stagnation quantities
    stagnation_pressure    = P0 * ((1. + (gamma - 1.) / 2. * M0 * M0) ** (gamma / (gamma - 1.)))
    stagnation_temperature = T0 * (1. + ((gamma - 1.) / 2. * M0 * M0))

    # ---------------------------------------------------------------------------------
    # Fuel cell stack configuration
    # ---------------------------------------------------------------------------------
    fuel_cell = fuel_cell_stack.fuel_cell
    n_series  = fuel_cell_stack.electrical_configuration.series
    n_parallel = fuel_cell_stack.electrical_configuration.parallel
    n_total   = n_series * n_parallel

    # ---------------------------------------------------------------------------------
    # Retrieve converter conditions and power demand
    # ---------------------------------------------------------------------------------
    fc_conds = state.conditions.energy.converters[fuel_cell_stack.tag]

    electrical_distributor_tag = None
    for d_tag in fuel_cell_stack.assigned_distributors[0]:
        if network.distributors[d_tag].domain == 'electrical':
            electrical_distributor_tag = d_tag
    psi = state.conditions.energy.battery_fuel_cell_power_split_ratio[electrical_distributor_tag]
    total_electrical_demand = state.conditions.energy.distributors[electrical_distributor_tag].outputs.power.electrical
    P_stack  = total_electrical_demand * fuel_cell_stack.power_split_ratio * (1. - psi)
    P_cell   = P_stack[:, 0] / n_total  # power demand per individual cell [W] 

    fc_conds.stagnation_temperature[:] = stagnation_temperature
    fc_conds.stagnation_pressure[:]    = stagnation_pressure

    # ---------------------------------------------------------------------------------
    # Vectorized Newton-Raphson to find current density matching power demand
    # ---------------------------------------------------------------------------------
    # The residual has a vertical asymptote at the limiting current density i_lim (the
    # concentration-loss term calculate_concentration_losses_LT/HT diverges there), so an
    # unbounded Newton step taken from the shallow part of the power curve can overshoot
    # past i_lim in a single iteration. Past that point, partial pressures go negative and
    # activation losses (which raise P_O2 to a fractional power) return NaN, which then
    # never recovers. Clamp every iterate to a physically valid bracket [i_floor, i_ceiling]
    # so the solve stays well-defined even when the demanded power exceeds what the stack
    # can actually deliver (in which case it saturates at i_ceiling instead of diverging).
    i_floor   = 1e-6
    stack_T   = fc_conds.stack_temperature[:, 0]
    P_O2_ceiling = calculate_P_O2(fuel_cell_stack, fuel_cell.rated_air_pressure, stack_T,
                                   fuel_cell.oxygen_relative_humidity, fuel_cell.air_excess_ratio, 0., 0.)
    if fuel_cell.type == "LT":
        i_ceiling = calculate_limiting_current_density_LT(fuel_cell_stack, stack_T, P_O2_ceiling,
                                                            fuel_cell.oxygen_relative_humidity, fuel_cell.air_excess_ratio, 0., 0.)
    else:
        i_ceiling = calculate_limiting_current_density_HT(fuel_cell_stack, stack_T, P_O2_ceiling,
                                                            fuel_cell.oxygen_relative_humidity, fuel_cell.air_excess_ratio, 0., 0.)
    i_ceiling = 0.999 * i_ceiling

    i_vec = np.clip(np.ones_like(P_cell), i_floor, i_ceiling)  # initial guess [A/cm^2]
    di    = 1e-6                                                # finite-difference step for Jacobian

    for _ in range(50):
        # Evaluate residual: R(i) = P_net(i) - P_demand
        fc_conds.current_density[:, 0] = i_vec
        _, _, P_net, _, _, _, _, _, _ = evaluate_PEM(fuel_cell_stack, fc_conds)
        residual = P_net - P_cell

        if np.all(np.abs(residual) < 1E-8):
            break

        # Finite-difference Jacobian: dP_net/di
        fc_conds.current_density[:, 0] = i_vec + di
        _, _, P_net_pert, _, _, _, _, _, _ = evaluate_PEM(fuel_cell_stack, fc_conds)
        dPdi = (P_net_pert - P_net) / di

        # Newton update with safeguard against zero derivative, clamped to stay physical
        i_vec -= residual / np.where(np.abs(dPdi) > 1e-30, dPdi, 1e-30)
        i_vec  = np.clip(i_vec, i_floor, i_ceiling)

    # ---------------------------------------------------------------------------------
    # Final evaluation at converged current densities
    # ---------------------------------------------------------------------------------
    fc_conds.current_density[:, 0] = i_vec
    mdot_H2, V_fc, net_power, _, _, _, mdot_air_in, mdot_air_out, _ = evaluate_PEM(fuel_cell_stack, fc_conds)

    # Scale single-cell quantities to full stack
    I_cell  = net_power / V_fc        # cell current [A]
    I_stack = I_cell * n_parallel     # stack current (parallel cells share current) [A]

    fc_conds.power[:, 0]                    = net_power * n_total
    fc_conds.current[:, 0]                  = I_stack
    fc_conds.voltage_open_circuit[:, 0]     = V_fc * n_series
    fc_conds.voltage_under_load[:, 0]       = V_fc * n_series
    fc_conds.H2_mass_flow_rate[:, 0]        = mdot_H2 * n_total
    fc_conds.inlet_H2_mass_flow_rate[:, 0]  = mdot_H2
    fc_conds.inlet_air_mass_flow_rate[:, 0] = mdot_air_in
    fc_conds.outlet_air_mass_flow_rate[:, 0] = mdot_air_out

    # Chemical (hydrogen) power draw, fed to the fuel line so assigned fuel tanks
    # can compute their own mass depletion.
    fc_conds.inputs.power.chemical = fc_conds.H2_mass_flow_rate * fuel_cell_stack.fuel_cell.propellant.lower_heating_value
    fc_conds.fuel_mass_flow_rate   = fc_conds.H2_mass_flow_rate

    stored_results_flag  = True
    stored_converter_tag = fuel_cell_stack.tag

    fc_conds.outputs.power.electrical = fc_conds.power * fuel_cell_stack.electrical_efficiency

    return fc_conds.inputs, fc_conds.outputs, stored_results_flag, stored_converter_tag


# ----------------------------------------------------------------------------------------------------------------------
#  Evaluate PEM Cell Performance
# ----------------------------------------------------------------------------------------------------------------------
def evaluate_PEM(fuel_cell_stack, fc_conds):
    """Evaluates PEM fuel cell electrochemistry for all control points simultaneously.

    Computes cell voltage via the Nernst equation with activation, ohmic, and
    concentration losses, then determines gross/net power and hydrogen consumption.
    The compressor-expander module (CEM) parasitic losses are subtracted to yield
    the net electrical power.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack object with cell-level electrochemical properties.
    fc_conds : Conditions
        Fuel cell operating conditions (current density, temperatures, pressures).

    Returns
    -------
    mdot_H2 : ndarray
        Hydrogen mass flow rate per cell [kg/s].
    voltage : ndarray
        Cell output voltage [V].
    net_power : ndarray
        Net electrical power per cell after CEM and parasitic losses [W].
    gross_power : ndarray
        Gross electrical power per cell (voltage × current density × area) [W].
    gross_heat : ndarray
        Heat generated per cell from voltage losses [W].
    compressor_power : ndarray
        Power consumed by the compressor-expander module [W].
    mdot_air_in : ndarray
        Air mass flow rate entering the cathode [kg/s].
    mdot_air_out : ndarray
        Air mass flow rate exiting the cathode (less consumed O2) [kg/s].
    expander_power : ndarray
        Power recovered by the expander [W].
    """
    fuel_cell        = fuel_cell_stack.fuel_cell
    i                = fc_conds.current_density[:, 0]   # current density [A/cm^2]
    air_excess_ratio = fuel_cell.air_excess_ratio

    # Pressure drops across stack and humidifier
    fc_conds.pressure_drop[:, 0] = calculate_P_drop_stack(fuel_cell_stack, i)
    if fuel_cell.type == "LT":
        fc_conds.humidifier.pressure_drop[:, 0] = calculate_P_drop_hum(fuel_cell_stack, i)
    else:
        fc_conds.humidifier.pressure_drop[:, 0] = 0

    # Stoichiometric mass flow rates (Faraday's law)
    mdot_air_in = i * fuel_cell.interface_area * fuel_cell.O2_molar_mass / (4 * fuel_cell.Faraday_constant * fuel_cell.O2_mass_frac) * air_excess_ratio
    mdot_H2     = i * fuel_cell.interface_area / (2 * fuel_cell.Faraday_constant) * fuel_cell.H2_molar_mass

    # Electrochemical voltage model
    voltage, V_loss = calculate_voltage(i, fuel_cell_stack, fc_conds)
    gross_power = voltage * i * fuel_cell.interface_area
    gross_heat  = V_loss  * i * fuel_cell.interface_area

    # Compressor-expander module losses
    fc_conds.inlet_air_mass_flow_rate[:, 0] = mdot_air_in
    compressor_power, mdot_air_out, expander_power = evaluate_CEM(fuel_cell_stack, fc_conds)

    # Net power = gross - CEM parasitic - other parasitic
    parasitic_power = fuel_cell.gamma_para * gross_power
    net_power       = gross_power - compressor_power - parasitic_power

    return mdot_H2, voltage, net_power, gross_power, gross_heat, compressor_power, mdot_air_in, mdot_air_out, expander_power


# ----------------------------------------------------------------------------------------------------------------------
#  Evaluate Compressor-Expander Module (CEM)
# ----------------------------------------------------------------------------------------------------------------------
def evaluate_CEM(fuel_cell_stack, fc_conds):
    """Evaluates the compressor-expander module (CEM) for all control points.

    The CEM compresses ambient air to the required fuel cell cathode pressure,
    and an expander recovers energy from the cathode exhaust. The net CEM power
    is the difference between compressor input and expander output.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack object containing CEM efficiency parameters.
    fc_conds : Conditions
        Operating conditions including stagnation state and air mass flow rates.

    Returns
    -------
    p_req : ndarray
        Net power required by the CEM (compressor - expander) [W].
    mdot_air_out : ndarray
        Mass flow rate of air exiting the cathode [kg/s].
    exp_p_ext : ndarray
        Power extracted by the expander [W].
    """
    Cp  = 1004   # specific heat of air [J/(kg·K)]
    gam = 1.4    # ratio of specific heats for air

    fuel_cell        = fuel_cell_stack.fuel_cell
    CEM              = fuel_cell.compressor_expander_module
    Tt_in            = fc_conds.stagnation_temperature[:, 0]
    Pt_in            = fc_conds.stagnation_pressure[:, 0]
    mdot_air_in      = fc_conds.inlet_air_mass_flow_rate[:, 0]
    p_drop_hum       = fc_conds.humidifier.pressure_drop[:, 0]
    pressure_drop    = fc_conds.pressure_drop[:, 0]
    FC_air_p         = fuel_cell.rated_air_pressure
    air_excess_ratio = fuel_cell.air_excess_ratio

    # Compressor: raise ambient air to cathode supply pressure (p_air_FC already includes
    # the humidifier pressure drop the compressor must overcome, so it is not added again here).
    # Ram air can already meet or exceed the required cathode pressure at low altitude/high
    # speed; the compressor cannot do negative work in that case, so the ratio is floored at 1.
    p_air_FC          = FC_air_p + p_drop_hum
    pressure_ratio     = np.maximum(p_air_FC / Pt_in, 1.0)
    comp_p_req         = mdot_air_in * Cp * Tt_in * (pressure_ratio ** ((gam - 1) / gam) - 1) / CEM.compressor_efficiency
    input_p    = comp_p_req / CEM.motor_efficiency

    # Expander: recover energy from cathode exhaust
    p_exp        = p_air_FC - pressure_drop - p_drop_hum
    Tt_exp       = Tt_in * (p_exp / Pt_in) ** ((gam - 1) / gam)
    mdot_air_out = mdot_air_in - mdot_air_in / air_excess_ratio * 0.233  # O2 consumed
    exp_p_ext    = mdot_air_out * Cp * Tt_exp * (1 - (Pt_in / p_exp) ** ((gam - 1) / gam)) * CEM.expander_efficiency
    output_p     = exp_p_ext * CEM.generator_efficiency

    # Net CEM power
    p_req = input_p - output_p

    # Store CEM conditions
    fc_conds.outlet_air_pressure[:, 0]              = p_air_FC
    fc_conds.compressor_inlet_pressure[:, 0]        = Pt_in
    fc_conds.compressor_pressure_ratio[:, 0]        = pressure_ratio
    fc_conds.compressor_inlet_mass_flow_rate[:, 0]  = mdot_air_in
    fc_conds.compressor_power[:, 0]                 = comp_p_req
    fc_conds.expander_outlet_mass_flow_rate[:, 0]   = mdot_air_out
    fc_conds.expander_inlet_pressure[:, 0]          = p_exp
    fc_conds.expander_pressure_ratio[:, 0]          = Pt_in / p_exp
    fc_conds.expander_power[:, 0]                   = exp_p_ext
    fc_conds.motor_compressor_power[:, 0]           = input_p
    fc_conds.expander_generator_power[:, 0]         = output_p
    fc_conds.compressor_expander_module_power[:, 0] = p_req

    return p_req, mdot_air_out, exp_p_ext


# ----------------------------------------------------------------------------------------------------------------------
#  Cell Voltage Model
# ----------------------------------------------------------------------------------------------------------------------
def calculate_voltage(i, fuel_cell_stack, fc_conds):
    """Computes the cell output voltage and total voltage loss.

    Applies the Nernst equation for the reversible potential, then subtracts
    activation, ohmic, and concentration overpotentials plus any degradation.

    .. math::
        V_{cell} = E_{cell} - \\eta_{act} - \\eta_{ohmic} - \\eta_{conc} - V_{deg}

    Parameters
    ----------
    i : ndarray
        Current density at each control point [A/cm^2].
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack object with electrochemical parameters.
    fc_conds : Conditions
        Operating conditions (stack temperature, pressure drops, degradation).

    Returns
    -------
    V_cell : ndarray
        Net cell voltage [V].
    V_loss : ndarray
        Total voltage loss (activation + ohmic + concentration + degradation) [V].
    """
    fuel_cell         = fuel_cell_stack.fuel_cell
    stack_temperature = fc_conds.stack_temperature[:, 0]
    P_H2_input        = fuel_cell.rated_H2_pressure
    P_air             = fuel_cell.rated_air_pressure
    RH                = fuel_cell.oxygen_relative_humidity
    air_excess_ratio  = fuel_cell.air_excess_ratio
    pressure_drop     = fc_conds.pressure_drop[:, 0]
    degradation       = fc_conds.degradation[:, 0]

    # Partial pressures at electrode surfaces
    P_O2   = calculate_P_O2(fuel_cell_stack, P_air, stack_temperature, RH, air_excess_ratio, pressure_drop, i)
    P_H2   = calculate_P_H2(fuel_cell_stack, P_H2_input, stack_temperature, RH, i)

    # Reversible Nernst potential
    E_cell = calculate_E_cell(fuel_cell_stack, stack_temperature, P_H2, P_O2)

    # Overpotentials (model selection based on LT vs HT PEM)
    if fuel_cell.type == "LT":
        eta_ohmic = calculate_ohmic_losses_LT(fuel_cell_stack, stack_temperature, i)
        eta_conc  = calculate_concentration_losses_LT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, pressure_drop, i)
    elif fuel_cell.type == "HT":
        eta_ohmic = calculate_ohmic_losses_HT(fuel_cell_stack, stack_temperature, i)
        eta_conc  = calculate_concentration_losses_HT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, pressure_drop, i)
    eta_act = calculate_activation_losses(fuel_cell_stack, stack_temperature, P_O2, i)

    V_cell = E_cell - eta_act - eta_ohmic - eta_conc - degradation * fuel_cell.maximum_deg
    V_loss = eta_act + eta_ohmic + eta_conc + degradation * fuel_cell.maximum_deg
    return V_cell, V_loss


# ----------------------------------------------------------------------------------------------------------------------
#  Rated Current Density Setter
# ----------------------------------------------------------------------------------------------------------------------
def set_rated_current_density(fuel_cell_stack, rated_current_density, rated_power_density):
    """Stores the rated current density and power density on the fuel cell object.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack whose rated point is being set.
    rated_current_density : float
        Current density at the maximum-power operating point [A/cm^2].
    rated_power_density : float
        Power density at the maximum-power operating point [W/cm^2].
    """
    fuel_cell                       = fuel_cell_stack.fuel_cell
    fuel_cell.rated_current_density = rated_current_density
    fuel_cell.rated_power_density   = rated_power_density
    return


# ----------------------------------------------------------------------------------------------------------------------
#  Pressure Drop Models
# ----------------------------------------------------------------------------------------------------------------------
def calculate_P_drop_hum(fuel_cell_stack, i):
    """Calculates humidifier pressure drop, scaled quadratically from rated conditions.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with rated pressure drop properties.
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Humidifier pressure drop [bar].
    """
    fuel_cell = fuel_cell_stack.fuel_cell
    return (i / fuel_cell.rated_current_density) ** 2 * fuel_cell.rated_p_drop_hum


def calculate_P_drop_stack(fuel_cell_stack, i):
    """Calculates stack cathode pressure drop, scaled quadratically from rated conditions.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with rated pressure drop properties.
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Stack pressure drop [bar].
    """
    fuel_cell = fuel_cell_stack.fuel_cell
    return (i / fuel_cell.rated_current_density) ** 2 * fuel_cell.rated_p_drop_fc


# ----------------------------------------------------------------------------------------------------------------------
#  Partial Pressure Models
# ----------------------------------------------------------------------------------------------------------------------
def calculate_P_O2(fuel_cell_stack, P_air, stack_temperature, RH, air_excess_ratio, P_drop, i):
    """Calculates oxygen partial pressure at the cathode catalyst layer.

    Accounts for water vapor pressure, cathode pressure drop, air excess
    ratio, and current-dependent concentration gradient (empirical N factor).

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack (unused directly, kept for API consistency).
    P_air : ndarray or float
        Cathode inlet air pressure [bar].
    stack_temperature : ndarray or float
        Stack temperature [K].
    RH : float
        Relative humidity of the cathode air stream [0-1].
    air_excess_ratio : float
        Cathode stoichiometric ratio (> 1 means excess air).
    P_drop : ndarray or float
        Cathode pressure drop [bar].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Oxygen partial pressure at the catalyst layer [bar].
    """
    T_C       = stack_temperature - 273.15
    log_P_H2O = -2.1794 + 0.02953 * T_C - 9.1837e-5 * T_C**2 + 1.4454e-7 * T_C**3
    P_H2O     = 10 ** log_P_H2O
    N         = 0.291 * i / (stack_temperature ** 0.832)
    P_O2      = 0.21 * (P_air - P_drop / 2 - RH * P_H2O) * ((1 + (air_excess_ratio - 1) / air_excess_ratio) / 2) / np.exp(N)
    return P_O2


def calculate_P_H2(fuel_cell_stack, P_H2_input, stack_temperature, RH, i):
    """Calculates hydrogen partial pressure at the anode catalyst layer.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack (unused directly, kept for API consistency).
    P_H2_input : float
        Anode inlet hydrogen pressure [bar].
    stack_temperature : ndarray or float
        Stack temperature [K].
    RH : float
        Relative humidity of the anode stream [0-1].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Hydrogen partial pressure at the catalyst layer [bar].
    """
    T_C       = stack_temperature - 273.15
    log_P_H2O = -2.1794 + 0.02953 * T_C - 9.1837e-5 * T_C**2 + 1.4454e-7 * T_C**3
    P_H2O     = 10 ** log_P_H2O
    P_H2      = 0.5 * (P_H2_input / np.exp(1.653 * i / stack_temperature**1.334) - RH * P_H2O)
    return P_H2


# ----------------------------------------------------------------------------------------------------------------------
#  Nernst Potential
# ----------------------------------------------------------------------------------------------------------------------
def calculate_E_cell(fuel_cell_stack, stack_temperature, P_H2, P_O2):
    """Calculates the reversible (Nernst) cell potential.

    .. math::
        E = 1.229 - 8.45 \\times 10^{-4}(T - 298.15)
            + \\frac{RT}{4\\alpha F}\\left(\\ln P_{H_2} + 0.5 \\ln P_{O_2}\\right)

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with thermodynamic constants.
    stack_temperature : ndarray or float
        Stack temperature [K].
    P_H2 : ndarray or float
        Hydrogen partial pressure [bar].
    P_O2 : ndarray or float
        Oxygen partial pressure [bar].

    Returns
    -------
    ndarray or float
        Reversible cell potential [V].
    """
    fuel_cell = fuel_cell_stack.fuel_cell
    # P_H2 and P_O2 can go non-positive when the Newton-Raphson current-density
    # solve (in compute_fuel_cell_performance) overshoots into an unphysical
    # region (e.g. demand far exceeding the stack's rated capacity). Floor them
    # so log() returns a large negative value (very low, but finite, cell
    # voltage) instead of NaN, keeping the solve well-defined everywhere.
    P_H2_safe = np.maximum(P_H2, 1e-8)
    P_O2_safe = np.maximum(P_O2, 1e-8)
    E_cell    = 1.229 - 8.45e-4 * (stack_temperature - 298.15) + \
        fuel_cell.Universal_gas_constant * stack_temperature / (4 * fuel_cell.alpha * fuel_cell.Faraday_constant) * (np.log(P_H2_safe) + 0.5 * np.log(P_O2_safe))
    return E_cell


# ----------------------------------------------------------------------------------------------------------------------
#  Activation Losses
# ----------------------------------------------------------------------------------------------------------------------
def calculate_activation_losses(fuel_cell_stack, stack_temperature, P_O2, i):
    """Calculates activation overpotential using the Tafel equation.

    .. math::
        \\eta_{act} = \\frac{RT}{2\\alpha F} \\ln\\left(\\frac{i}{i_0}\\right)

    where the exchange current density :math:`i_0` depends on catalyst loading,
    specific area, O2 partial pressure, and temperature (Arrhenius form).

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with catalyst and kinetic parameters.
    stack_temperature : ndarray or float
        Stack temperature [K].
    P_O2 : ndarray or float
        Oxygen partial pressure [bar].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Activation voltage loss [V].
    """
    fuel_cell = fuel_cell_stack.fuel_cell
    A_const   = fuel_cell.Universal_gas_constant * stack_temperature / (2 * fuel_cell.alpha * fuel_cell.Faraday_constant)
    i0        = fuel_cell.i0ref * fuel_cell.L_c * fuel_cell.a_c * (P_O2 / fuel_cell.i0ref_P_ref) ** fuel_cell.gamma * \
        np.exp(-fuel_cell.E_C / (fuel_cell.Universal_gas_constant * stack_temperature) * (1 - (stack_temperature / fuel_cell.i0ref_T_ref)))
    eta_act   = A_const * np.log(i / i0)
    return eta_act


# ----------------------------------------------------------------------------------------------------------------------
#  Ohmic Losses
# ----------------------------------------------------------------------------------------------------------------------
def calculate_ohmic_losses_LT(fuel_cell_stack, stack_temperature, i):
    """Calculates ohmic losses for low-temperature PEM using Nafion membrane conductivity.

    Uses the empirical Springer model for Nafion membrane resistivity as a function
    of temperature, current density, and effective water content (lambda_eff).

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with membrane properties (thickness, lambda_eff).
    stack_temperature : ndarray or float
        Stack temperature [K].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Ohmic voltage loss [V].
    """
    fuel_cell  = fuel_cell_stack.fuel_cell
    t_m        = fuel_cell.t_m
    lambda_eff = fuel_cell.lambda_eff
    num        = 181.6 * (1 + 0.03 * i + 0.062 * (stack_temperature / 303) ** 2 * i ** 2.5)
    denom      = (lambda_eff - 0.634 - 3 * i) * np.exp(4.18 * (stack_temperature - 303) / stack_temperature)
    rho        = num / denom
    eta_ohmic  = (rho * t_m) * i
    return eta_ohmic


def calculate_ohmic_losses_HT(fuel_cell_stack, stack_temperature, i):
    """Calculates ohmic losses for high-temperature PEM using PBI membrane conductivity.

    Interpolates membrane conductivity linearly between 100 C and 200 C using
    the c1 and c2 fitting parameters.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with PBI membrane properties (thickness, c1, c2).
    stack_temperature : ndarray or float
        Stack temperature [K].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Ohmic voltage loss [V].
    """
    fuel_cell = fuel_cell_stack.fuel_cell
    t_m       = fuel_cell.t_m
    c1        = fuel_cell.c1
    c2        = fuel_cell.c2
    s         = (stack_temperature - 373.15) / 100 * (c2 - c1) + c1
    rho       = 1 / s
    eta_ohmic = (rho * t_m) * i
    return eta_ohmic


# ----------------------------------------------------------------------------------------------------------------------
#  Concentration Losses
# ----------------------------------------------------------------------------------------------------------------------
def calculate_concentration_losses_LT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i):
    """Calculates concentration overpotential for low-temperature PEM.

    Returns a large penalty (10 V) when current density exceeds the limiting
    current density, preventing non-physical solutions.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with charge-transfer coefficient (alpha).
    stack_temperature : ndarray or float
        Stack temperature [K].
    P_O2 : ndarray or float
        Oxygen partial pressure [bar].
    RH : float
        Relative humidity [0-1].
    air_excess_ratio : float
        Cathode stoichiometric ratio.
    P_drop : ndarray or float
        Cathode pressure drop [bar].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Concentration voltage loss [V].
    """
    fuel_cell = fuel_cell_stack.fuel_cell
    i_lim     = calculate_limiting_current_density_LT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i)
    # np.where for vectorized branching; np.maximum prevents log(0)
    eta_conc  = np.where(
        i >= i_lim,
        10.0,
        (1 + 1 / fuel_cell.alpha) * fuel_cell.Universal_gas_constant * stack_temperature / (2 * fuel_cell.Faraday_constant) * np.log(i_lim / np.maximum(i_lim - i, 1e-30))
    )
    return eta_conc


def calculate_limiting_current_density_LT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i):
    """Calculates limiting current density for LT-PEM via linear interpolation.

    Interpolates between reference O2 partial pressures at 1 atm and 2.5 atm
    to estimate the limiting current density at the actual O2 partial pressure.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with current density limit multiplier.
    stack_temperature : ndarray or float
        Stack temperature [K].
    P_O2 : ndarray or float
        Oxygen partial pressure [bar].
    RH : float
        Relative humidity [0-1].
    air_excess_ratio : float
        Cathode stoichiometric ratio.
    P_drop : ndarray or float
        Cathode pressure drop [bar].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Limiting current density [A/cm^2].
    """
    fuel_cell        = fuel_cell_stack.fuel_cell
    P_O2_ref_1_atm   = calculate_P_O2(fuel_cell_stack, 1, stack_temperature, RH, air_excess_ratio, P_drop, i)
    P_O2_ref_2_5_atm = calculate_P_O2(fuel_cell_stack, 2.5, stack_temperature, RH, air_excess_ratio, P_drop, i)
    i_lim            = (2.25 - 1.65) * (P_O2 - P_O2_ref_1_atm) / (P_O2_ref_2_5_atm - P_O2_ref_1_atm) + 1.65
    return fuel_cell.current_density_limit_multiplier * i_lim


def calculate_concentration_losses_HT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i):
    """Calculates concentration overpotential for high-temperature PEM.

    Same structure as the LT version but uses a different charge-transfer
    coefficient weighting (1.8/alpha instead of 1/alpha).

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with charge-transfer coefficient (alpha).
    stack_temperature : ndarray or float
        Stack temperature [K].
    P_O2 : ndarray or float
        Oxygen partial pressure [bar].
    RH : float
        Relative humidity [0-1].
    air_excess_ratio : float
        Cathode stoichiometric ratio.
    P_drop : ndarray or float
        Cathode pressure drop [bar].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Concentration voltage loss [V].
    """
    fuel_cell = fuel_cell_stack.fuel_cell
    i_lim     = calculate_limiting_current_density_HT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i)
    eta_conc  = np.where(
        i >= i_lim,
        10.0,
        (1 + 1.8 / fuel_cell.alpha) * fuel_cell.Universal_gas_constant * stack_temperature / (2 * fuel_cell.Faraday_constant) * np.log(i_lim / np.maximum(i_lim - i, 1e-30))
    )
    return eta_conc


def calculate_limiting_current_density_HT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, P_drop, i):
    """Calculates limiting current density for HT-PEM via linear interpolation.

    Interpolates between reference O2 partial pressures at 1 atm and 2 atm.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with current density limit multiplier.
    stack_temperature : ndarray or float
        Stack temperature [K].
    P_O2 : ndarray or float
        Oxygen partial pressure [bar].
    RH : float
        Relative humidity [0-1].
    air_excess_ratio : float
        Cathode stoichiometric ratio.
    P_drop : ndarray or float
        Cathode pressure drop [bar].
    i : ndarray or float
        Current density [A/cm^2].

    Returns
    -------
    ndarray or float
        Limiting current density [A/cm^2].
    """
    fuel_cell      = fuel_cell_stack.fuel_cell
    P_O2_ref_1_atm = calculate_P_O2(fuel_cell_stack, 1, stack_temperature, RH, air_excess_ratio, P_drop, i)
    P_O2_ref_2_atm = calculate_P_O2(fuel_cell_stack, 2, stack_temperature, RH, air_excess_ratio, P_drop, i)
    i_lim          = (2.15 - 1.4) * (P_O2 - P_O2_ref_1_atm) / (P_O2_ref_2_atm - P_O2_ref_1_atm) + 1.4
    return fuel_cell.current_density_limit_multiplier * i_lim


# ----------------------------------------------------------------------------------------------------------------------
#  Sizing Helpers (scalar — used with scipy.optimize.minimize_scalar)
# ----------------------------------------------------------------------------------------------------------------------
def evaluate_max_gross_power(fuel_cell_stack, fuel_cell_conditions, t_idx):
    """Finds the current density that maximizes gross power for a single operating point.

    Used during fuel cell sizing to determine the rated operating point.
    Calls ``scipy.optimize.minimize_scalar`` on the negative power-density curve
    bounded between 20 % and 95 % of the limiting current density.

    Parameters
    ----------
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack to size.
    fuel_cell_conditions : Conditions
        Conditions at the design point (scalar, indexed by t_idx).
    t_idx : int
        Time index for the single design point.

    Returns
    -------
    rated_current_density : float
        Current density at peak power [A/cm^2].
    rated_power_density : float
        Peak power density [W/cm^2].
    """
    fuel_cell         = fuel_cell_stack.fuel_cell
    FC_air_p          = fuel_cell.rated_air_pressure
    stack_temperature = fuel_cell_conditions.stack_temperature[t_idx, 0]
    RH                = fuel_cell.oxygen_relative_humidity
    air_excess_ratio  = fuel_cell.air_excess_ratio

    P_O2 = calculate_P_O2(fuel_cell_stack, FC_air_p, stack_temperature, RH, air_excess_ratio, fuel_cell.rated_p_drop_fc, 0)

    if fuel_cell.type == "LT":
        i_lim = calculate_limiting_current_density_LT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, 0, i=0)
    elif fuel_cell.type == "HT":
        i_lim = calculate_limiting_current_density_HT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, 0, i=0)

    res = minimize_scalar(evaluate_power_func, args=(fuel_cell_stack, fuel_cell_conditions, t_idx), bounds=(0.2 * i_lim, 0.95 * i_lim))
    rated_current_density = res.x
    rated_power_density   = -res.fun

    return rated_current_density, rated_power_density


def evaluate_power_func(i, fuel_cell_stack, fuel_cell_conditions, t_idx):
    """Objective function for sizing: returns negative power density at current density i.

    Scalar function passed to ``minimize_scalar``. Computes the full voltage model
    at a single operating point and returns ``-V_cell * i`` (negative because
    the optimizer minimizes).

    Parameters
    ----------
    i : float
        Current density to evaluate [A/cm^2].
    fuel_cell_stack : Fuel_Cell
        Fuel cell stack with electrochemical parameters.
    fuel_cell_conditions : Conditions
        Conditions at the design point.
    t_idx : int
        Time index for the single design point.

    Returns
    -------
    float
        Negative power density ``-V_cell * i`` [W/cm^2].
    """
    fuel_cell         = fuel_cell_stack.fuel_cell
    stack_temperature = fuel_cell_conditions.stack_temperature[t_idx, 0]
    P_H2_input        = fuel_cell.rated_H2_pressure
    P_air             = fuel_cell.rated_air_pressure
    RH                = fuel_cell.oxygen_relative_humidity
    air_excess_ratio  = fuel_cell.air_excess_ratio
    pressure_drop     = fuel_cell_conditions.pressure_drop[t_idx, 0]
    degradation       = fuel_cell_conditions.degradation[t_idx, 0]

    P_O2   = calculate_P_O2(fuel_cell_stack, P_air, stack_temperature, RH, air_excess_ratio, pressure_drop, i)
    P_H2   = calculate_P_H2(fuel_cell_stack, P_H2_input, stack_temperature, RH, i)
    E_cell = calculate_E_cell(fuel_cell_stack, stack_temperature, P_H2, P_O2)

    if fuel_cell.type == "LT":
        eta_ohmic = calculate_ohmic_losses_LT(fuel_cell_stack, stack_temperature, i)
        eta_conc  = calculate_concentration_losses_LT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, pressure_drop, i)
    elif fuel_cell.type == "HT":
        eta_ohmic = calculate_ohmic_losses_HT(fuel_cell_stack, stack_temperature, i)
        eta_conc  = calculate_concentration_losses_HT(fuel_cell_stack, stack_temperature, P_O2, RH, air_excess_ratio, pressure_drop, i)
    eta_act = calculate_activation_losses(fuel_cell_stack, stack_temperature, P_O2, i)

    V_cell = E_cell - eta_act - eta_ohmic - eta_conc - degradation * fuel_cell.maximum_deg
    return -V_cell * i
